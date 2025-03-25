function J_b = jacobe(S, M, q, frame)
    % JACOBE - Computes the body Jacobian for a given manipulator configuration.
    %
    % Syntax:
    %    J_b = jacobe(S, M, q, frame)
    %
    % Inputs:
    %    S     - Screw axes of the manipulator in the given frame (6xn matrix)
    %    M     - Home configuration of the end-effector (4x4 matrix)
    %    q     - Joint variables (nx1 vector
    %    frame - 'body' or 'space' to indicate the frame of screw axes
    %
    % Outputs:
    %    J_b - Body Jacobian (6xn matrix)
    %
    % Example:
    %    S = [0 0 1 0 0 0; 0 1 0 -0.5 0 0; 0 1 0 -1 0 0]';
    %    M = [1 0 0 1; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    %    q = [pi/2; pi/4; pi/6];
    %    J_b = jacobe(S, M, q, 'space')
    %
    % See also: TWISTSBODY2SPACE, FKINE, JACOB0, INVADJOINT

    if strcmp(frame, 'body')
        % Convert the twists to the space frame
        S = twistsbody2space(S, M);
    end

    S2B = fkine(S, M, q, 'space');
    J_s = jacob0(S, q);
    J_b = invadjoint(S2B) * J_s;
end
