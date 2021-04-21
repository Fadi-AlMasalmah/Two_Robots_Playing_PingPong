Project: Two_Robots_Playing_PingPong
# MATLAB Ping Pong
This is a MATLAB/Simulink simulation of two 7-axis KUKA robots playing ping pong.
This project is a Master's degree project. The goal of that project was to build
a realistic simulation for a table-tennis game with accounting for the physics of 
the environment, and the dynamics of the robotic arm. For that, we did the following tasks:
1 - Built kinematic and Dynamic Models for a 7-DOF Robot (KUKA - IIWA7). 
2 - Designed a control system for the Robotic arm using computed torque method.
3 - Built mathematical models for the physics of the game (spin, aerodynamics, collision)
4 - Designed an algorithm for hitting position and trajectory optimization.

(A presentation of the project is available in Russian language in "PowerPoint" folder)

# Comment
This project used this repository (https://github.com/phibenz/Matlab_Table_Tennis) as a beginning point 

## Dependencies
[MATLAB](https://de.mathworks.com/help/matlab/)  
[Simulink](https://de.mathworks.com/help/simulink/index.html)  
[Simscape](https://de.mathworks.com/help/physmod/simscape/index.html)  
[Simscape Multibody Contact ForcesLibrary](https://de.mathworks.com/matlabcentral/fileexchange/47417-simscape-multibody-contact-forces-library)  
[Robotics Toolbox](https://www.mathworks.com/products/robotics.html)  

## Usage
`Launch_Game_TwoRobots.m` starts the simulation

## Videos
[![Two Robots Playing Ping pong](https://github.com/Fadi-AlMasalmah/Two_Robots_Playing_PingPong/blob/master/Video/1.jpg)](https://youtu.be/covMtJ-vK6g "KUKA IIWA7 playing ping pong (Kuka IIWA7 playing ping pong (MATLAB simulation))")

[![Two Robots Playing Ping pong 2](https://github.com/Fadi-AlMasalmah/Two_Robots_Playing_PingPong/blob/master/Video/2.jpg)](https://youtu.be/ley3xYENL5k)
