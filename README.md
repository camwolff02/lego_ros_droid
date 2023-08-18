# lego_ros_droid
Researching packages to use:
Camera:
  - https://index.ros.org/r/v4l2_camera/ (raspberry pi cam)
  - https://medium.com/swlh/raspberry-pi-ros-2-camera-eef8f8b94304
  - https://jeffzzq.medium.com/ros2-image-pipeline-tutorial-3b18903e7329
  - https://github.com/clydemcqueen/opencv_cam
  - official CameraInfo node: image_proc/image_pipeline

Aruco:
  - https://gitlab.com/raymondchaneee/aruco_ros2
  - https://index.ros.org/p/aruco/
  - https://index.ros.org/search/?term=aruco

large motor techincal specifications: https://education.lego.com/v3/assets/blt293eea581807678a/bltb9abb42596a7f1b3/5f8801b5f4c5ce0e93db1587/le_spike-prime_tech-fact-sheet_45602_1hy19.pdf


Setting it up:
  Steam Deck:
    Install ubuntu: https://www.youtube.com/watch?v=kkkyNA31KOA
    Install humble: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

    sudo apt install usbutils 
    sudo apt install net-tools 
    sudo apt install ros-humble-ros2-control 
    sudo apt install ros-humble-ros2-controllers 
    sudo apt install ros-humble-gazebo-ros2-control 
    sudo apt install ros-humble-gazebo-ros 
    sudo apt install ros-humble-xacro 
    sudo apt install ros-humble-joint-trajectory-controller 
    sudo apt install ros-humble-joint-state-broadcaster 
    sudo apt install ros-humble-joint-state-publisher-gui

    sudo apt install ros-humble-navigation2
    sudo apt install ros-humble-nav2-bringup
    sudo apt install ros-humble-turtlebot3-gazebo

  Raspberry Pi:
    Install Raspian, set up SSH during install
    Enable serial port, disable serial console (for build hat)
    pip3 install buildhat

      

  Install on both:



TO RUN SIMPLE CAMERA PUB: ros2 run image_tools cam2image
TO RUN CAMERA SUB: ros2 run rqt_image_view rqt_image_view

