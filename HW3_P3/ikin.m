function q = ikin(S, M, currentQ, targetPose)
    % IKIN - Inverse Kinematics for a given manipulator configuration.
    %
    % Syntax:
    %    q = ikin(S, M, currentQ, targetPose)
    %
    % Inputs:
    %    S          - Screw axes of the manipulator in the given frame (6xn matrix)
    %    M          - Home configuration of the end-effector (4x4 matrix)
    %    currentQ   - Current joint variables (nx1 vector)
    %    targetPose  - Desired end-effector pose (6x1 vector)
    %
    % Outputs:
    %    q          - Joint variables that achieve the desired end-effector pose
    %
    % Example:
    %    S = [0 0 1 0 0 0; 0 1 0 -0.5 0 0; 0 1 0 -1 0 0]';
    %    M = [1 0 0 1; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    %    currentQ = [pi/2; pi/4; pi/6];
    %    targetPose = [pi/2; pi/4; pi/6; -1; -2; -3];
    %    q = ikin(S, M, currentQ, targetPose)
    %
    % See also: FKINE, JACOBE, INVADJOINT

    % Parameters
    maxIKIterations = 250; % Maximum number of iterations for IK
    lambda = 0.1; % Damping factor for the least squares method
    scaling = 0.5; % Scaling factor for joint angle change
    qlim = [[-180, 180];
             [-125, 125];
             [-138, 138];
             [-270, 270];
             [-120, 133.5];
             [-270, 270]]; % Joint limits

    % Convert joint limits to radians
    qlim = qlim * pi / 180;

    % Seed the current pose with the robot's starting pose
    T = fkine(S, M, currentQ);
    currentPose = ht2pose(T);

    iterations = 0;

    while norm(targetPose - currentPose) > 1e-6 && iterations < maxIKIterations
        J = jacobe(S, M, currentQ, 'space');

        % Damped least squares
        deltaQ = scaling * pinv(J' * J + lambda ^ 2 * eye(size(J, 2))) * J' * (targetPose - currentPose);

        currentQ = currentQ + deltaQ';

        % Ensure the joints are within limits
        for i = 1:length(currentQ)

            if currentQ(i) < qlim(i, 1)
                currentQ(i) = qlim(i, 1);
            elseif currentQ(i) > qlim(i, 2)
                currentQ(i) = qlim(i, 2);
            end

        end

        % Update the current pose
        T = fkine(S, M, currentQ);
        currentPose = ht2pose(T);

        iterations = iterations + 1;
    end

    % Check if the maximum number of iterations was reached
    if iterations >= maxIKIterations
        %fprintf("\n");
        %fprintf('Maximum number of iterations reached for configuration \n');
        %fprintf('Joint variables: %s\n', mat2str(currentQ));
        %fprintf('Target pose: %s\n', mat2str(targetPose));
        %fprintf('Current pose: %s\n', mat2str(currentPose));
        %fprintf('Jacobian: %s\n', mat2str(J));
        %fprintf('Jacobian determinant: %s\n', num2str(det(J)));
        q = zeros(size(currentQ)); % Return zeros if max iterations reached
        return
    end

    % Return the joint variables that achieve the desired end-effector pose
    q = currentQ;
end
