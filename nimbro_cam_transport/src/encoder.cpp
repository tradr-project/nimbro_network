#include <functional>
#include <memory>

#include <nimbro_cam_transport/encoder.h>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

extern "C"
{
#include <x264.h>
}

#include <nimbro_cam_transport/rgb_to_yuv420.h>

namespace nimbro_cam_transport
{

typedef nimbro_cam_transport::H264PublisherConfig Config;
typedef dynamic_reconfigure::Server<Config> ReconfigureServer;

struct H264EncoderPIMPL
{
  ros::Time lastImageTime = ros::Time(0);
  ros::Duration minTimeBetweenImages;

  int width;
  double aspectRatio;

  std::vector<uint8_t> inBuf;
  x264_t* encoder = nullptr;
  int encoderConfiguredHeight;

  x264_picture_t inputPicture;
  x264_picture_t outPicture;

  double crf;

  ros::NodeHandle node;
  std::unique_ptr<ReconfigureServer> reconfigure_server;
  Config config;
};

H264Encoder::H264Encoder(const ros::NodeHandle& node)
{
    this->data = std::make_unique<H264EncoderPIMPL>();

    this->data->node = node;

    this->data->reconfigure_server = std::make_unique<ReconfigureServer>(this->data->node);
    ReconfigureServer::CallbackType f = std::bind(&H264Encoder::configCb, this, std::placeholders::_1, std::placeholders::_2);
    this->data->reconfigure_server->setCallback(f);

    this->data->aspectRatio = 0.0; // it has to be determined from the first image
}

void H264Encoder::reloadEncoder()
{
    // it has to be determined from the first image
    if (this->data->aspectRatio == 0.0)
        return;

    if (this->data->encoder) {
        x264_encoder_close(this->data->encoder);
        this->data->encoder = nullptr;
    }

    int height = static_cast<int>(this->data->width * this->data->aspectRatio);

    x264_param_t params;
    x264_param_default(&params);
    x264_param_apply_profile(&params, "high");
    x264_param_default_preset(&params, "ultrafast", "zerolatency");

    params.i_width = this->data->width;
    params.i_height = height;
    params.b_repeat_headers = 1;
    params.b_intra_refresh = 1;
    params.i_fps_num = 1;
    params.i_fps_den = 10;
    params.i_frame_reference = 1;
    params.i_keyint_max = 20;
    params.i_bframe = 0;
    params.b_open_gop = 0;
    // 	params.rc.i_rc_method = X264_RC_CRF;
    // // 	params.rc.i_qp_min = params.rc.i_qp_max = 47;
    // 	params.rc.i_vbv_buffer_size = 6;
    // 	params.rc.i_vbv_max_bitrate = 6000;
    // 	params.rc.i_bitrate = 6;
    params.rc.i_rc_method = X264_RC_CRF;
    params.rc.f_rf_constant = this->data->crf;
    params.rc.i_vbv_buffer_size = 1000;
    params.rc.i_vbv_max_bitrate = 1000;
    params.rc.i_bitrate = 1000;
    params.i_threads = 4;

    this->data->encoder = x264_encoder_open(&params);

    x264_picture_init(&this->data->inputPicture);
    x264_picture_init(&this->data->outPicture);

    this->data->encoderConfiguredHeight = height;

    this->data->inBuf.resize(this->data->width*height + this->data->width*height/2);
}

sensor_msgs::CompressedImagePtr H264Encoder::handleImage(
        const sensor_msgs::ImageConstPtr &img)
{
    ros::Time now = ros::Time::now();
    if(now - this->data->lastImageTime < this->data->minTimeBetweenImages)
        return nullptr;
    this->data->lastImageTime = now;

    ros::Time start = ros::Time::now();

    cv_bridge::CvImageConstPtr cvImg;
    try
    {
       cvImg = cv_bridge::toCvShare(img, "bgr8");
    } catch (cv_bridge::Exception& e) {
	ROS_ERROR_THROTTLE(5.0, "Failed converting image to bgr8: %s", e.what());
        return nullptr;
    }

    this->data->aspectRatio = ((double)cvImg->image.rows) / cvImg->image.cols;

    int height = static_cast<int>(this->data->width * this->data->aspectRatio);

    cv::Mat resized;
    cv::resize(cvImg->image, resized, cv::Size(this->data->width, height), CV_INTER_AREA);

    if(!this->data->encoder)
    {
        this->reloadEncoder();
    }
    else if(height != this->data->encoderConfiguredHeight)
    {
        ROS_WARN("Image dimensions changed!");
        this->reloadEncoder();
    }

    RGB_to_YUV420(resized.data, this->data->inBuf.data(), this->data->width, height);

    this->data->inputPicture.img.plane[0] = this->data->inBuf.data();
    this->data->inputPicture.img.plane[1] = this->data->inBuf.data() + this->data->width*height;
    this->data->inputPicture.img.plane[2] = this->data->inBuf.data() + this->data->width*height + this->data->width*height/4;

    this->data->inputPicture.img.i_stride[0] = this->data->width;
    this->data->inputPicture.img.i_stride[1] = this->data->width/2;
    this->data->inputPicture.img.i_stride[2] = this->data->width/2;

    this->data->inputPicture.img.i_csp = X264_CSP_I420;
    this->data->inputPicture.img.i_plane = 3;

    x264_nal_t* nals;
    int numNals;

    x264_encoder_encode(this->data->encoder, &nals, &numNals, &this->data->inputPicture, &this->data->outPicture);

    std::size_t size = 0;
    for(int i = 0; i < numNals; ++i)
        size += nals[i].i_payload;

    sensor_msgs::CompressedImagePtr msg(new sensor_msgs::CompressedImage);

    msg->header = img->header;
    msg->format = "h264";

    msg->data.resize(size);
    unsigned int off = 0;

    for(int i = 0; i < numNals; ++i)
    {
        memcpy(msg->data.data() + off, nals[i].p_payload, nals[i].i_payload);
        off += nals[i].i_payload;
    }

    ROS_DEBUG("took %f", (ros::Time::now() - start).toSec());
    return msg;
}

void H264Encoder::configCb(nimbro_cam_transport::H264PublisherConfig &config,
                           uint32_t level)
{
    this->data->crf = config.crf;
    this->data->width = config.width;
    this->data->minTimeBetweenImages = ros::Duration(1.0 / config.max_rate);

    this->reloadEncoder();
}

H264Encoder::~H264Encoder()
{
    // cannot be substituted with "= default" because of PIMPL pointer
}

}
