% RBE 501 - Robot Dynamics - Spring 2025
% Midterm Exam - 03/03/2025
% Worcester Polytechnic Institute
% Instructor: L. Fichera <lfichera@wpi.edu>

clear, clc, close all
addpath('utils');

%% First, execute make_kinematics_model() to load the S and M matrices
plotOn = false;
n = 6; % number of degrees of freedom
maxIKIterations = 1000000;
robot = make_robot();
[S, M] = make_kinematics_model();

% Display the robot in its home configuration
robot.teach(zeros(1, n)), hold on;

%% Load the Path that the robot has to trace
load ik_data.mat
scatter3(path(1, :), path(2, :), path(3, :), 'filled');

%% Solve the inverse kinematics
% Initialize a matrix to store the IK solution
qList = zeros(size(targetPose, 2), n);
failCount = 0;
nPoses = size(targetPose, 2);
fprintf('Calculating poses: ')
percent = fprintf('0%%');

for poseI = 1:nPoses
    fprintf(repmat('\b', 1, percent));
    percent = fprintf('%0.f%%', ceil(poseI / nPoses * 100));

    % Seed the current pose with the robot's starting pose
    T = fkine(S, M, qList(poseI, :)');
    currentPose = ht2pose(T);

    iterations = 0;

    while norm(targetPose(:, poseI) - currentPose) > 1e-3 && iterations < maxIKIterations
        J = jacob0(S, qList(poseI, :));

        alpha = 0.01; % Scaling factor

        % Newton Raphson with scaling
        deltaQ = alpha * pinv(J) * (targetPose(:, poseI) - currentPose);

        qList(poseI, :) = qList(poseI, :) + deltaQ';

        T = fkine(S, M, qList(poseI, :));
        currentPose = ht2pose(T);

        iterations = iterations + 1; % Increment iteration counter

    end

    % Check if the maximum number of iterations was reached
    if iterations >= maxIKIterations
        fprintf("\n");
        fprintf('Maximum number of iterations reached for configuration %d.\n', poseI);
        fprintf('Target joint variables: %s\n', mat2str(qList(poseI, :)));
        fprintf('Target pose: %s\n', mat2str(targetPose(:, poseI)));
        fprintf('Jacobian: %s\n', mat2str(J));
        failCount = failCount + 1;
    end

end

if failCount == 0
    fprintf('\nPose calculation finished.\n');
else
    fprintf('\nThere were %d failed poses.\n', failCount);
end

close all;
robot.plot(repmat([qList; flip(qList)], 10, 1), 'trail', {'r', 'LineWidth', 5});
