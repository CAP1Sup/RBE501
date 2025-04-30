function T = fkine(S, M, q)
    % FKINE Computes the forward kinematics for a serial chain robot
    %
    %   T = FKINE(S, M, q) computes the homogeneous transformation matrix T
    %   representing the pose of the end-effector given the screw axes S, the
    %   home configuration M, and the joint variables q.
    %
    %   Inputs:
    %       S - 6xn matrix of screw axes for the robot in the space frame
    %       M - 4x4 homogeneous transformation matrix representing the home
    %           configuration of the end-effector
    %       q - nx1 vector of joint variables
    %
    %   Output:
    %       T - 4x4 homogeneous transformation matrix representing the pose of
    %           the end-effector in the space frame
    %
    %   Example:
    %       S = [0 0 1 0 0 0; 0 1 0 -0.5 0 0.5]';
    %       M = [1 0 0 1; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    %       q = [pi/2; pi/4];
    %       T = fkine(S, M, q);
    %
    %   See also TWIST2HT

    T = eye(4);

    for joint = 1:size(S, 2)
        T = T * twist2ht(S(:, joint), q(joint));
    end

    T = T * M;
end
