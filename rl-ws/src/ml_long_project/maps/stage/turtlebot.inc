define kinect ranger
(
  sensor
  (
    range_max 6.5
    fov 360
    samples 640
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

define turtlebot3 position
(
  pose [ 0.0 0.0 0.0 0.0 ]
  
  localization "gps"
  
  odom_error [0 0 0 0 0 0]

  size [ 0.2552 0.2552 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "gray"
  
  kinect(pose [ -0.1 0.0 -0.11 0.0 ])
)
