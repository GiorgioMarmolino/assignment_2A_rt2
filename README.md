### RESEARCH TRACK - ASSIGNMENT 2 - "Interactive Client Node"

Made by: Marmolino Giorgio
a.y. 2024/2025
Project fully developed in python3

## Project introduction
In this project, a mobile robot moves within a 3D simulated environment containing edges and obstacles (represented by walls). The robot's goal is to reach a target position set by the user. 

In this version, the action client node has been developed in an interactive and more user friendly **Jupyter notebook**, allowing the user to set a target or cancel it. 
The notebook also displays a plot showing the robot’s current position and the path it has traveled. Specifically, this plot enables the user to monitor the robot’s position and view its path. A dropdown menu allows the user to toggle the visibility of the robot's trajectory and to reset the path using the "reset trace" button. At the end of the script, along with the position plot, widgets are displayed showing the distance to the nearest obstacle and a bar chart indicating the number of targets successfully reached versus those that were not. This project has been developed starting from my original package created for the [second assignment](https://github.com/GiorgioMarmolino/assignment2_1_rt) of the Research Track 1 course.

## Prerequisites
Before executing the project you are required to: 
1) Install Gazebo, the 3D simulator for the ROS;
2) Install Rviz, a tool for ROS visualization that allows the user to view the simulated mobile robot model with all its sensors and related informations; it is usefull for debug purposes;
3) Intall Xterm;
4) Install python3, since the code is written in python3;
5) Install Robot Operating System (known as ROS);
6) Install Jupyter Notebook or Jupyter Lab

## Running the project
After cloning this repository into your src/ folder of your ROS1 workspace:

```bash
git clone https://github.com/GiorgioMarmolino/assignment_2A_rt2
```
go back in the main folder of your ROS-WS and build this package:

```bash
catkin_make --only-pkg-with-deps assignment_2A_rt2
source devel/setup.bash
```
If you are using **Docker**, you can launch Jupyter Notebook using:
```bash
jupyter notebook --allow-root --ip 0.0.0.0 --NotebookApp.token='' --NotebookApp.password=''
```
so that no login is required. Then open a browser and go to:

```bash
http://localhost:8888/
```
Now you are free to navigate in your ROS-WS. Then, open a terminal in Jupyter Notebook in the ROS-WS and launch the project using the launch file (project package have still the old name since the project is based on the previous assignment):

```bash
roslaunch assignment2_1_rt assignment1.launch
```
Now, run the Jupyter Notebook (can be find in the path: *assignment2_1_rt/notebooks/assignment_2A_rt2.ipynb*) using the *"restart and run all"* button.

## Interface interaction

This should get your Jupyter‑based action client up and running smoothly. Once it’s running, you are free to:

- **Assign** or **cancel** a target `(x, y)` goal using `ipywidgets`.
- Live data showing the robot’s **current position** and the **distance to the closest obstacle**.
- A real-time animated plot (animated in real time using `FuncAnimation`) showing the **robot’s current position** and the **robot's previous path** through the environment.
- A bar chart (updating as the robot navigates) that tracks the **number of reached and not-reached targets**.