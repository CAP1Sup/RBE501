% RBE 501 - Robot Dynamics - Spring 2025
% Homework 3, Problem 2
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <loris@wpi.edu>
% Last modified: 03/17/2025
clear, clc, close all
nTests = 20;

%% Create the manipulator
mdl_stanford
stanf

% Display the manipulator
stanf.teach(zeros(1, 6)), hold on;

% Load the path that we wish to follow and display it
load hw3problem2_path.mat
scatter3(path(1, :), path(2, :), path(3, :), 'filled');

%% YOUR CODE HERE
