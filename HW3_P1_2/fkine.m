function T = fkine(S, M, q, frame)
    % FKINE Computes the forward kinematics for a serial chain robot.
    %
    %   T = FKINE(S, M, q, frame) computes the homogeneous transformation matrix
    %   T representing the pose of the end-effector given the screw axes S, the
    %   home configuration M, the joint variables q, and the frame type.
    %
    %   Inputs:
    %       S     - 6xn matrix of screw axes for the robot in the given frame.
    %       M     - 4x4 homogeneous transformation matrix representing the home
    %               configuration of the end-effector.
    %       q     - nx1 vector of joint variables.
    %       frame - String specifying the frame type. 'body' for body frame,
    %               otherwise space frame.
    %
    %   Output:
    %       T     - 4x4 homogeneous transformation matrix representing the pose
    %               of the end-effector.
    %   Example:
    %       S = [0 0 1 0 0 0; 0 1 0 -0.5 0 0.5]';
    %       M = [1 0 0 1; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    %       q = [pi/2; pi/4];
    %       T = fkine(S, M, q, 'space');
    %
    %   See also TWIST2HT

    T = eye(4);

    for joint = 1:size(S, 2)
        T = T * twist2ht(S(:, joint), q(joint));
    end

    if strcmp(frame, 'body')
        T = M * T;
    else
        T = T * M;
    end

end
