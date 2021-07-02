# Technical_assignment_TUD_internship
This repository consists of the ROS1 & ROS2 packages developed for simulating & controlling a 2DOF serial manipulator.Three different exercises of the technical assignment for the TUD internship is solved and the ROS2 package is the solution for the programming part.

The robotic arm consits of two revolute joints and two rigid segments. Each link has a total mass of 1kg with uniform mass distribution and is of length 1m. The two segments are connected to each other with a rotational or revolute joint which can also be actuated
(e.g., we can apply a torque to the rotational joint). We connect the two segments with an additional rotational and actuated joint to the ground plane. Gravity is acting in vertical
direction - so rectangular to the ground plane.

The inverse as well as forward kinematics of this manipulator has been studied and an analytical approach is used to implement inverse kinematics in 'ex3.py' python script
