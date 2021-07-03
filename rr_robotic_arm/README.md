# ROS1 package for simulation & control of the modelled robotic arm
This is the ROS1 package for the simulation and control

To run the solution for the exercise 2, one should follow the below mentioned consecutive commands (for
ROS1 package) :

- Launch the required nodes with the provided launch file by executing the following command in the
terminal: 
> roslaunch rr_robotic_arm rrbot_gazebo.launch
- Load the joint controller configurations from YAML file to parameter server with the following
Command: 
> roslaunch rr_robotic_arm rrbot_control1.launch
- Run your ROS1 Node for exercise 2 by executing the following command in the
Terminal:
> rosrun rr_robotic_arm ex2.py

![](https://github.com/RUFFY-369/Technical_assignment_TUD_internship/blob/main/imgs/ex2_normal.gif)

To run the same solution for the exercise 2 but on manipulator with soft joints, do as follows:

> roslaunch rr_robotic_arm rrbot_gazebo_soft.launch

> roslaunch rr_robotic_arm rrbot_control1.launch

> rosrun rr_robotic_arm ex2.py

![](https://github.com/RUFFY-369/Technical_assignment_TUD_internship/blob/main/imgs/ex2_soft.gif)



To run the solution for the exercise 3, one should follow the below mentioned consecutive commands (for
ROS1 package) :
- Launch the required nodes with the provided launch file by executing the following command in the
terminal:
>  roslaunch rr_robotic_arm rrbot_gazebo.launch
- Load the joint controller configurations from YAML file to parameter server with the following
Command:
> roslaunch rr_robotic_arm rrbot_control.launch
- Run your ROS1 Node for exercise 2 by executing the following command in the
Terminal:
> rosrun rr_robotic_arm ex3.py

![](https://github.com/RUFFY-369/Technical_assignment_TUD_internship/blob/main/imgs/ex3_normal.gif)

To run the same solution for the exercise 3 but on manipulator with soft joints, do as follows:

> roslaunch rr_robotic_arm rrbot_gazebo_soft.launch

> roslaunch rr_robotic_arm rrbot_control.launch

> rosrun rr_robotic_arm ex3.py

![](https://github.com/RUFFY-369/Technical_assignment_TUD_internship/blob/main/imgs/ex3_soft.gif)