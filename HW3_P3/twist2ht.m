function T = twist2ht(S, theta)
    % TWIST2HT Convert a twist to a homogeneous transformation matrix.
    %
    %   T = twist2ht(S, theta) converts a twist S and a joint displacement
    %   theta to a homogeneous transformation matrix T.
    %
    %   Inputs:
    %   - S: A 6x1 vector representing the twist. The first three elements
    %        are the angular velocity vector (w), and the last three elements
    %        are the linear velocity vector (v).
    %   - theta: A scalar representing the joint displacement.
    %
    %   Outputs:
    %   - T: A 4x4 homogeneous transformation matrix.
    %
    %   Example:
    %   S = [0; 0; 1; 1; 0; 0];
    %   theta = pi/4;
    %   T = twist2ht(S, theta);
    %
    %   See also AXISANGLE2ROT, SKEW.

    % Readability
    w = S(1:3);
    v = S(4:6);

    % Convert to rotation and position
    R = axisangle2rot(w, theta);
    P = (eye(3) * theta + (1 - cos(theta)) * skew(w) + (theta - sin(theta)) * skew(w) * skew(w)) * v;

    T = [R(1, 1), R(1, 2), R(1, 3), P(1);
         R(2, 1), R(2, 2), R(2, 3), P(2);
         R(3, 1), R(3, 2), R(3, 3), P(3);
         0, 0, 0, 1];
end
