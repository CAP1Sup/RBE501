function J = jacob0(S, q)
    % JACOB0 Compute the space Jacobian for a serial chain robot
    %
    %   J = JACOB0(S, q) computes the space Jacobian for a serial chain robot
    %   given the screw axes in the space frame when the manipulator is at the
    %   home position and the joint variables.
    %
    %   Inputs:
    %       S - 6xn matrix, where each column is a screw axis in the space frame
    %           when the manipulator is at the home position.
    %       q - nx1 vector of joint variables.
    %
    %   Output:
    %       J - 6xn space Jacobian matrix.
    %
    %   Example:
    %       S = [0 0 1 0 0 0; 0 1 0 -0.5 0 0; 0 1 0 -1 0 0]';
    %       q = [pi/2; pi/4; pi/6];
    %       J = jacob0(S, q);
    %
    %   See also TWIST2HT, ADJOINT

    joints = size(S, 2);
    J = zeros(6, joints);
    T = eye(4);

    for joint = 1:joints
        T = T * twist2ht(S(:, joint), q(joint));
        J(:, joint) = adjoint(T) * S(:, joint);
    end

end
