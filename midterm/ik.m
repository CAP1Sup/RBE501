% RBE 501 - Robot Dynamics - Spring 2025
% Midterm Exam - 03/03/2025
% Worcester Polytechnic Institute
% Instructor: L. Fichera <lfichera@wpi.edu>

clear, clc, close all
addpath('utils');

%% First, execute make_kinematics_model() to load the S and M matrices
n = 6; % number of degrees of freedom
robot = make_robot();
[S,M] = make_kinematics_model();

% Display the robot in its home configuration
robot.teach(zeros(1,n)), hold on;

%% Load the Path that the robot has to trace
load ik_data.mat
scatter3(path(1,:), path(2,:), path(3,:), 'filled');


%% Solve the inverse kinematics
% Initialize a matrix to store the IK solution
qList = zeros(size(targetPose,2), n);

% *** YOUR CODE HERE
