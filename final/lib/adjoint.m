function AdT = adjoint(T)
    % ADJOINT Computes the adjoint representation of a transformation matrix.
    %
    %   AdT = adjoint(T) returns the adjoint representation of the given
    %   transformation matrix T. The adjoint representation is a 6x6 matrix
    %   that is used in the context of rigid body transformations.
    %
    %   Input:
    %       T - A 4x4 homogeneous transformation matrix.
    %
    %   Output:
    %       AdT - A 6x6 adjoint representation matrix.
    %
    %   The adjoint representation is defined as:
    %       AdT = [R, 0;
    %              skew(P) * R, R]
    %   where R is the 3x3 rotation matrix and P is the 3x1 position vector
    %   extracted from the transformation matrix T.
    %
    %   Example:
    %       T = [1, 0, 0, 1;
    %            0, 1, 0, 2;
    %            0, 0, 1, 3;
    %            0, 0, 0, 1];
    %       AdT = adjoint(T)
    %
    %   See also: SKEW

    R = T(1:3, 1:3);
    P = T(1:3, 4);
    AdT = [R, zeros(3);
           skew(P) * R, R];
end
