# Path Planning

This project is a part of Udacity's *Self-Driving Car Nanodegree* program. In this project, 
the goal is to design a path planner that is able to create smooth, safe paths for the car 
to follow along a 3 lane highway with traffic. A successful path planner will be able 
to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all 
by using localization, sensor fusion, and map data.

## Project structure

In the code, the task is split into 3 stages, each implemented as a separate class:

* `Predictor` takes the current vehicle's position as well as the information about other 
cars from the sensor fusion data. The goal of this class is to analyze the situation around 
my own vehicle and generate the prediction if vehicle's current, left and right lanes are 
available for maneuvering. 

* `Behavior` takes the prediction information and calculates which maneuver to perform. Available 
maneuvers are: keep the lane at the maximum speed; change left or right; return to the center lane;
follow the car ahead, adjusting own velocity to avoid collision. 

* `Trajectory` takes the recommendations provided by the behavior unit and generates the trajectory for 
the car to execute the maneuver. While generating the trajectory, this class makes sure that the car 
stays within the speed limit and performs the maneuver smoothly. Effectively, the trajectory is a cubic 
spline.

## Demonstration

This video recording demonstrates how the car performs in the simulator, being controlled by my path
planner implementation: 

[![Alt text](https://img.youtube.com/vi/dsJcPh4kx3I/0.jpg)](https://youtu.be/dsJcPh4kx3I)

