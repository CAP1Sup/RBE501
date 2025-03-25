%% RBE/ME 501 - Robot Dynamics - Spring 2025
%  Homework 3, Problem 3
%  Instructor: L. Fichera, <loris@wpi.edu>
close all; clear, clc
addpath('utils');

%% Initialize the model of the robot
% Screw Axes:
S = [0 0 1 0 0 0;
     0 1 0 -0.32 0 0;
     0 1 0 -0.545 0 0;
     0 0 1 0.035 0 0;
     0 1 0 -0.77 0 0;
     0 0 1 0.035 0 0]';

% Home configuration:
R = eye(3);
p = [0 0.035 0.835]';
M = [R p; 0 0 0 1];

%% Load the test configurations
load target_poses.mat
nPts = length(V);

%% Calculate the IK
% Initialize a matrix to store the IK solutions
q = zeros(nPts, 6);

failCount = 0;
startTime = tic;

for ii = 1:nPts
    q0 = zeros(1, 6);
    q(ii, :) = ikin(S, M, q0, V(:, ii));

    if q(ii, :) == zeros(1, 6)
        fprintf('IK failed for target pose %d\n', ii);
        failCount = failCount + 1;
    else
        fprintf('IK succeeded for target pose %d\n', ii);
    end

end

fprintf('Total number of failed IK solutions: %d\n', failCount);
fprintf('Total number of successful IK solutions: %d\n', nPts - failCount);
fprintf('Percentage of successful IK solutions: %.2f%%\n', (nPts - failCount) / nPts * 100);
fprintf('Elapsed time: %.2f seconds\n', toc(startTime));
