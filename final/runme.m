% RBE 501 - Robot Dynamics - Spring 2025
% Final Exam
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 04/29/2025
clear, close all, clc
addpath('lib');

% Define the environment
g = [0 0 -9.81]';

% Create the robot and display it in the home configuration
[robot, jointLimits] = make_robot();
[S, M] = make_kinematics_model();
[Mlist, Glist] = make_dynamics_model(robot);

% Load the joint variables
load joint_variables.mat; % this creates a vector called `waypoints` - each column is a vector of joint variables
robot.plot(waypoints', 'trail', {'r', 'LineWidth', 5});

% Gravity Compensation
% YOUR CODE HERE

% Torque-Based Motion Control
% YOUR CODE HERE
