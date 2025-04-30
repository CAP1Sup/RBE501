function J_a = jacoba(S, M, q, frame)
    % JACOBA - Computes the analytical Jacobian matrix for a given manipulator
    %
    % Syntax: J_a = jacoba(S, M, q, frame)
    %
    % Inputs:
    %    S     - 6xn matrix of screw axes in the given frame
    %    M     - 4x4 homogeneous transformation matrix representing the home configuration of the end-effector
    %    q     - nx1 vector of joint variables
    %    frame - 'body' or 'space' to indicate the frame of screw axes
    %
    % Outputs:
    %    J_a - 3xn analytical Jacobian matrix
    %
    % Example:
    %    S = [0 0 1 0 0 0; 0 1 0 -0.5 0 0; 0 1 0 -1 0 0]';
    %    M = [1 0 0 1; 0 1 0 0; 0 0 1 1; 0 0 0 1];
    %    q = [pi/2; pi/4; pi/6];
    %    J_a = jacoba(S, M, q, 'space')
    %
    %
    % See also: TWISTSBODY2SPACE, JACOB0, FKINE, SKEW

    if strcmp(frame, 'body')
        % Convert the twists to the space frame
        S = twistsbody2space(S, M);
    end

    J_s = jacob0(S, q);
    EE_T = fkine(S, M, q, 'space');
    J_a = J_s(4:6, :) - skew(EE_T(1:3, 4)) * J_s(1:3, :);
end
