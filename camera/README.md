# JENGA VISION 

Install April Tag packages: 
`vcs import --recursive --input src/jengavision/camera/config/jenga_vision.repo src/`

How to run:

1. Plug into the Franka and the realsense camera.
2. `ssh student@station`, then `ros2 launch franka_moveit_config moveit.launch.py use_rviz:=false robot_ip:=panda0.robot`
3. `ros2 launch franka_moveit_config rviz.launch.py robot_ip:=panda0.robot`
4. From the workspace containing `plan_execute` package, run `ros2 run plan_execute cv_test`
5. Open and source another terminal in `plan_execute` package for service calls
6. From terminal 5 run `ros2 service call /calibrate std_srvs/srv/Empty '{}'` to move the robot into the calibration position. Insert the april tag into the grippers.
7. From the workspace containing `camera` package run `ros2 launch camera jenga_vision.launch.py calibrate:=true`. After you see a message that indicates that `tf.yaml` has been written to, you can CTRL-C. The file `tf.yaml` will be saved in `${root_workspace}/install/camera/share/camera` and will be loaded automatically in future runs. Be careful colcon building the `camera` package after this point as it may get overwritten!
8. From terminal 5 run `ros2 service call /ready std_srvs/srv/Empty '{}'` to return the robot to the ready position. Remove the april tag from the grippers.
9. From the workspace containing `camera` package run `ros2 launch camera jenga_vision.launch.py`.
10. In the pop up window, ensure that the tower is visible in the color frame and make sure that it is inside the bounding square (if not, adjust the size with the trackbars)
11. Open and source another terminal in `camera` package for service calls
12. From terminal 11 run `ros2 service call /calib std_srvs/srv/Empty`. This will find the top of the jenga tower and the floor.
13. Remove a piece about halfway from the tower (and ensure you can see it from the camera frame). From terminal 11 run `ros2 service call /scan std_srvs/srv/Empty`. This will look for the piece in between the top of the tower and the table.
14. Theoretically this should be it!