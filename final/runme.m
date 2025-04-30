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
payload = 0.2; % kg

% Create the robot and display it in the home configuration
[robot, jointLimits] = make_robot();
[S, M] = make_kinematics_model();
[Mlist, Glist] = make_dynamics_model(robot);

% Load the joint variables
load joint_variables.mat; % this creates a vector called `waypoints` - each column is a vector of joint variables
robot.plot(waypoints', 'trail', {'r', 'LineWidth', 5});

% Gravity Compensation
fprintf('-----------------------Gravity Compensation-----------------------\n');

% Initialize the parameters for the RNE algorithm
clear params
params.g = g; % gravity vector
params.S = S; % screw axes
params.M = Mlist; % link frames
params.G = Glist; % inertial properties
params.jointPos = zeros(3, 1); % desired joint positions
params.jointVel = zeros(3, 1); % desired joint velocities
params.jointAcc = zeros(3, 1); % desired joint accelerations
T = fkine(S, M, params.jointPos);
Ftip_inS = [cross(T(1:3, 4), -g * payload); -g * payload];
params.Ftip = adjoint(T)' * Ftip_inS;

% Invoke the RNE algorithm to calculate the joint torques needed for
% gravity compensation
tau = rne(params);

fprintf('Joint Torques: ');
fprintf('[%f %f %f] Nm\n', tau(1), tau(2), tau(3));

% Torque-Based Motion Control
% Now, for each pair of consecutive waypoints, we will first calculate a
% trajectory between these two points, and then calculate the torque
% profile necessary to move from one point to the next.
fprintf('Generating the Trajectory and Torque Profiles... ');
nbytes = fprintf('0%%');

n = size(S, 2);

% Initialize the variables where we will store the torque profiles, joint
% positions, and time, so that we can display them later
tau_acc = [];
jointPos_acc = [];
t_acc = [];

for jj = 1:size(waypoints, 2) - 1
    fprintf(repmat('\b', 1, nbytes));
    nbytes = fprintf('%3.0f%%', 100 * (jj / (size(waypoints, 2) - 1)));

    % Initialize the time vector
    dt = 1e-3; % time step [s]
    t = 0:dt:0.1; % total time [s]

    % Initialize the arrays where we will accumulate the output of the robot
    % dynamics
    jointPos_prescribed = zeros(n, size(t, 2)); % Joint Variables (Prescribed)
    jointVel_prescribed = zeros(n, size(t, 2)); % Joint Velocities (Prescribed)
    jointAcc_prescribed = zeros(n, size(t, 2)); % Joint Accelerations (Prescribed)
    tau_prescribed = zeros(n, size(t, 2)); % Joint Torques

    jointPos_actual = zeros(n, size(t, 2)); % Joint Variables (Actual)
    jointVel_actual = zeros(n, size(t, 2)); % Joint Velocities (Actual)

    % For each joint
    for ii = 1:n
        % Calculate a trajectory using a quintic polynomial
        params_traj.t = [0 t(end)]; % start and end time of each movement step
        params_traj.dt = dt;
        params_traj.q = [waypoints(ii, jj) waypoints(ii, jj + 1)];
        params_traj.v = [0 0];
        params_traj.a = [0 0];

        traj = make_trajectory('quintic', params_traj);

        % Generate the joint profiles (position, velocity, and
        % acceleration)
        jointPos_prescribed(ii, :) = traj.q;
        jointVel_prescribed(ii, :) = traj.v;
        jointAcc_prescribed(ii, :) = traj.a;
    end

    % Initialize the parameters for both inverse and forward dynamics
    params_rne.g = g; % gravity
    params_rne.S = S; % screw axes
    params_rne.M = Mlist; % link frames
    params_rne.G = Glist; % inertial properties
    params_fdyn.g = g; % gravity
    params_fdyn.S = S; % screw axes
    params_fdyn.M = Mlist; % link frames
    params_fdyn.G = Glist; % inertial properties

    % Initialize the (actual) joint variables
    jointPos_actual(:, 1) = jointPos_prescribed(:, 1);
    jointVel_actual(:, 1) = jointVel_actual(:, 1);

    for ii = 1:size(t, 2) - 1
        % Calculate the joint torques using the RNE algorithm
        params_rne.jointPos = jointPos_prescribed(:, ii);
        params_rne.jointVel = jointVel_prescribed(:, ii);
        params_rne.jointAcc = jointAcc_prescribed(:, ii);
        T = fkine(S, M, params_rne.jointPos);
        Ftip_inS = [cross(T(1:3, 4), -g * payload); -g * payload];
        params_rne.Ftip = adjoint(T)' * Ftip_inS;

        tau_prescribed(:, ii) = rne(params_rne);

        % Feed the torques to the forward dynamics model and perform one
        % simulation step
        params_fdyn.jointPos = jointPos_actual(:, ii);
        params_fdyn.jointVel = jointVel_actual(:, ii);
        params_fdyn.tau = tau_prescribed(:, ii);
        T = fkine(S, M, params_fdyn.jointPos);
        Ftip_inS = [cross(T(1:3, 4), -g * payload); -g * payload];
        params_fdyn.Ftip = adjoint(T)' * Ftip_inS;

        jointAcc = fdyn(params_fdyn);

        % Integrate the joint accelerations to get velocity and
        % position
        jointVel_actual(:, ii + 1) = dt * jointAcc + jointVel_actual(:, ii);
        jointPos_actual(:, ii + 1) = dt * jointVel_actual(:, ii) + jointPos_actual(:, ii);
    end

    tau_prescribed(:, end) = tau_prescribed(:, end - 1);

    tau_acc = [tau_acc tau_prescribed];
    jointPos_acc = [jointPos_acc jointPos_actual];
    t_acc = [t_acc t + t(end) * (jj - 1)];
end

fprintf('\nDone. Simulating the robot...\n');

title('Inverse Dynamics Control');
robot.plot(jointPos_acc(:, 1:10:end)', 'trail', {'r', 'LineWidth', 2});
fprintf('Done.\n');

%% Display the Joint Torques
figure, hold on, grid on
plot(t_acc, tau_acc(1, :), 'Linewidth', 2);
plot(t_acc, tau_acc(2, :), 'Linewidth', 2);
plot(t_acc, tau_acc(3, :), 'Linewidth', 2);
title('Torque Profiles');
xlabel('Time [s]'), ylabel('Torque [Nm]');
legend({'Joint 1', 'Joint 2', 'Joint 3'});
set(gca, 'FontSize', 14);
hold off;
