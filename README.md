# MIE443_Lab3

To set up:
  Turtlebot_bringup minimal.launch has to be running
  Turtlebot_bringup 3dsensors.launch has to be running
  
  launch subscriber node to begin running
  
  The robot is always trying to maintain a meter of distance between itself and the color it's tracking also well as centering the object in it's point of view.
  Currently it is programmed to follow a bright yellow tape color but this can be changed by adjusting the HSV values in the rgb callback.

  Warnings:
  Sept 7th: It is only being controlled by a P controller so it is jumpy to start.
