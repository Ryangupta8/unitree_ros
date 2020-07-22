#!/bin/bash
rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "header: 
  seq: 0
  stamp: 
    secs: 1
    nsecs: 0
  frame_id: map
pose:
    pose:
      position:
        x: 92.567
        y: 99.557
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    covariance: [0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]"
