function R = axisangle2rot(omega, theta)
    % AXISANGLE2ROT Converts an axis-angle representation to a rotation matrix.
    %
    %   R = AXISANGLE2ROT(omega, theta) computes the rotation matrix R
    %   corresponding to a rotation of theta radians about the axis defined by
    %   the 3D vector omega.
    %
    %   Inputs:
    %       omega - A 3x1 vector representing the axis of rotation.
    %       theta - A scalar representing the angle of rotation in radians.
    %
    %   Outputs:
    %       R - A 3x3 rotation matrix.
    %
    %   Example:
    %       omega = [1; 0; 0];
    %       theta = pi/2;
    %       R = axisangle2rot(omega, theta);
    %
    %   See also: SKEW

    R = eye(3) + sin(theta) * skew(omega) + (1 - cos(theta)) * skew(omega) * skew(omega);
end
