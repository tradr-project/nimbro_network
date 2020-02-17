
tf_throttle
-----------

`tf_throttle` creates snapshots of the tf tree at fixed intervals. This reduces
the usually high bandwidth `/tf` topic down to manageable levels.

Subscriptions:
 - `/tf`: Transform input

Publications:
 - `~tf`: Snapshot output. This can be mapped to `/tf` on the receiving machine.

Parameters:
 - `rate` (float): Snapshot rate (default: 4.0)
 - `frame_ids` (list of strings): Only these frame IDs will be republished. 
                                  Empty list means all frames are republished.
                                  (default: [])
 - `send_static_tfs` (bool): Whether to include static transforms (default: true).
 - `only_child_frames` (bool): If true, only frames from `frame_id` will only 
                               match against child frames.

