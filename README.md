##### This reposotory includes environmental perception, self-localization, path planning and motion control modules for RMUA.

###### 1.motor controller(my_control)

​	Interface: 

​		Input: desired_posotion desired_yaw

​		Output: RotorPWM

​	Flowchart:![](./pictures/ControllerFlowchart.png)

​	Problem: 

​		A. It lacks odometry(nav_msgs/Odometry), so it cannot be used by now.		![](./pictures/problem1.png)

​		In rotors_simulator package, we can get odometry of uav from /gazebo![](./pictures/rotors_simulator_odometry.png)

​	But RMUA simulator doesn't provide Odometry, so we have to realize an odometry by provided rostopics in the followings.![](./pictures/RMUA_rostopics.png)



​	
