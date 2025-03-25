% RBE 501 - Robot Dynamics - Spring 2025
% Homework 3, Problem 2
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <loris@wpi.edu>
% Last modified: 03/17/2025
clear, clc, close all
addpath('utils');

plotOn = false;
nTests = 20;
maxIKIterations = 10000;

%% Create the manipulator
mdl_stanford
robot = stanf;

% Display the manipulator in the home configuration
robot.teach(zeros(1, 6)), hold on;

% Joint limits
qlim = robot.qlim;

%% Part A - Calculate the screw axes
% Let us calculate the screw axis for each joint
% Put all the axes into a 6xn matrix S, where n is the number of joints
S_body = transpose([0, 0, 1, 0, -0.154, 0;
                    -1, 0, 0, 0, 0.263, 0;
                    0, 0, 0, 0, 0, 1;
                    0, 0, 1, 0, 0, 0;
                    0, 1, 0, 0.263, 0, 0;
                    0, 0, 1, 0, 0, 0; ]);

%% Part B - Calculate the forward kinematics with the Product of Exponentials formula in the body frame
% First, let us calculate the homogeneous transformation matrix M for the
% home configuration
M = [0, 1, 0, 0;
     -1, 0, 0, 0.154;
     0, 0, 1, 0.675;
     0, 0, 0, 1];

fprintf('---------------------Forward Kinematics Test---------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');
joints = size(qlim, 1);

% Test the forward kinematics for 20 random sets of joint variables
for ii = 1:nTests
    fprintf(repmat('\b', 1, nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii / nTests * 100));

    % Generate a random configuration
    q = zeros(1, joints);

    for j = 1:joints
        q(j) = qlim(j, 1) + (qlim(j, 2) - qlim(j, 1)) * rand();
    end

    % Calculate the forward kinematics
    T = fkine(S_body, M, q, 'body');

    if plotOn
        robot.teach(q);
        title('Forward Kinematics Test');
    end

    assert(all(all(abs(double(robot.fkine(q)) - T) < 1e-10)));
end

fprintf('\nTest passed successfully.\n');

%% Part C - Calculate the Body Jacobian of the manipulator
fprintf('-------------------Differential Kinematics Test------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');
joints = size(qlim, 1);

% Test the correctness of the Jacobian for 20 random sets of joint
% variables
for ii = 1:nTests
    fprintf(repmat('\b', 1, nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii / nTests * 100));

    % Generate a random configuration
    q = zeros(1, joints);

    for j = 1:joints
        q(j) = qlim(j, 1) + (qlim(j, 2) - qlim(j, 1)) * rand();
    end

    % Calculate the Jacobian in the body frame
    J_b = jacobe(S_body, M, q, 'body');

    if plotOn
        robot.teach(q);
        title('Differential Kinematics Test');
    end

    % Test the correctness of the Jacobian
    J_test = [J_b(4:6, :); J_b(1:3, :)]; % swap the rotation and translation components
    assert(all(all(abs(double(robot.jacobe(q)) - J_test) < 1e-10)));
end

fprintf('\nTest passed successfully.\n');

%% Part D - Inverse Kinematics
% Load the path that we wish to follow and display it
load hw3problem2_path.mat
scatter3(path(1, :), path(2, :), path(3, :), 'filled');

% Initialize a matrix to store the IK solution
qList = zeros(size(path, 2), size(S_body, 2));
failCount = 0;
nPoses = size(path, 2);
fprintf('Calculating poses: ')
percent = fprintf('0%%');

for poseI = 1:nPoses
    fprintf(repmat('\b', 1, percent));
    percent = fprintf('%0.f%%', ceil(poseI / nPoses * 100));

    % Seed the current pose with the robot's previous pose
    if poseI == 1
        qList(poseI, :) = zeros(1, size(S_body, 2));
    else
        qList(poseI, :) = qList(poseI - 1, :);
    end

    T = fkine(S_body, M, qList(poseI, :)', 'body');
    currentPose = ht2pose(T);

    % Generate the goal pose
    TGoal = [eye(3), path(:, poseI); 0 0 0 1];
    goalPose = ht2pose(TGoal);

    iterations = 0;

    while norm(goalPose - currentPose) > 1e-3 && iterations < maxIKIterations
        J = jacobe(S_body, M, qList(poseI, :), 'body');

        alpha = 0.25; % Damping factor

        % Gradient descent step
        deltaQ = alpha * transpose(J) * (goalPose - currentPose);

        qList(poseI, :) = qList(poseI, :) + deltaQ';

        % Prevent the prismatic joint from exceeding its limits
        if qList(poseI, 3) < qlim(3, 1)
            qList(poseI, 3) = qlim(3, 1);
        end

        % Update the current pose
        T = fkine(S_body, M, qList(poseI, :), 'body');
        currentPose = ht2pose(T);

        iterations = iterations + 1; % Increment iteration counter

    end

    % Check if the maximum number of iterations was reached
    if iterations >= maxIKIterations
        fprintf("\n");
        fprintf('Maximum number of iterations reached for configuration %d.\n', poseI);
        fprintf('Target joint variables: %s\n', mat2str(qList(poseI, :)));
        fprintf('Target pose: %s\n', mat2str(path(:, poseI)));
        fprintf('Jacobian: %s\n', mat2str(J));
        failCount = failCount + 1;
    end

end

if failCount == 0
    fprintf('\nPose calculation finished.\n');
else
    fprintf('\nThere were %d failed poses.\n', failCount);
end

% Print the joint variables for the poses
fprintf('Joint variables for the poses:\n');

for ii = 1:size(qList, 1)
    fprintf('Pose %d: %s\n', ii, mat2str(qList(ii, :)));
end

close all;
robot.plot(repmat([qList; flip(qList)], 10, 1), 'trail', {'r', 'LineWidth', 5});
