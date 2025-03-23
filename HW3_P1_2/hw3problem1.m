% RBE 501 - Robot Dynamics - Spring 2025
% Homework 3, Problem 1
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <loris@wpi.edu>
% Last modified: 03/17/2025
clear, clc, close all

nTests = 20;

%% Create the manipulator
% Link length values (meters)
L1 = 0.3;
L2 = 0.3;
L3 = 0.3;

robot = SerialLink([Revolute('a', 0, 'd', L1, 'alpha', pi / 2, 'offset', pi / 2), ...
                        Revolute('a', L2, 'd', 0, 'alpha', 0), ...
                        Revolute('a', L3, 'd', 0, 'alpha', pi / 2, 'offset', -pi / 2)], ...
    'name', 'RRR Manipulator');

% Joint limits
qlim = [-pi / 2 pi / 2; % q(1)
        -pi / 4 pi / 2; % q(2)
        -pi / 12 pi / 3]; % q(3)

% Display the manipulator in the home configuration
q = zeros(1, 3);
robot.teach(q);

%% Part A - Calculate the screw axes
% Let us calculate the screw axis for each joint
% Put all the axes into a 6xn matrix S, where n is the number of joints
% S_body = ...

%% Part B - Calculate the forward kinematics with the Product of Exponentials formula in the body frame
% First, let us calculate the homogeneous transformation matrix M for the
% home configuration
R_home = [0 0 -1; 1 0 0; 0 -1 0]';
t_home = [0 L2 L1 - L3]';
M = [R_home t_home; 0 0 0 1];

fprintf('---------------------Forward Kinematics Test---------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

% Test the forward kinematics for 20 random sets of joint variables
for ii = 1:nTests
    fprintf(repmat('\b', 1, nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii / nTests * 100));

    % Generate a random configuration
    q = [qlim(1, 1) + (qlim(1, 2) - qlim(1, 1)) * rand(), ...
             qlim(2, 1) + (qlim(2, 2) - qlim(2, 1)) * rand(), ...
             qlim(3, 1) + (qlim(3, 2) - qlim(3, 1)) * rand()];

    % Calculate the forward kinematics
    % T = ...

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

% Test the correctness of the Jacobian for 20 random sets of joint
% variables
for ii = 1:nTests
    fprintf(repmat('\b', 1, nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii / nTests * 100));

    % Generate a random configuration
    q = [qlim(1, 1) + (qlim(1, 2) - qlim(1, 1)) * rand(), ...
             qlim(2, 1) + (qlim(2, 2) - qlim(2, 1)) * rand(), ...
             qlim(3, 1) + (qlim(3, 2) - qlim(3, 1)) * rand()];

    % Calculate the Jacobian in the body frame
    % J_b = ...

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
% Load the path that we wish the robot to follow and display it
load hw3problem1_path.mat
robot.plot(zeros(1, 3)), hold on;
scatter3(path(1, :), path(2, :), path(3, :), 'filled');

%YOUR INVERSE KINEMATICS SOLVER GOES HERE
