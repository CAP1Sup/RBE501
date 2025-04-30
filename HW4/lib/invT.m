function iT = invT(T)
    % invT - Computes the inverse of a homogeneous transformation matrix.
    %
    % Syntax:
    %   iT = invT(T)
    %
    % Inputs:
    %   T - A 4x4 homogeneous transformation matrix. The matrix represents
    %       a rigid body transformation, where the top-left 3x3 submatrix
    %       is the rotation matrix, and the top-right 3x1 column is the
    %       translation vector.
    %
    % Outputs:
    %   iT - A 4x4 homogeneous transformation matrix that is the inverse
    %        of the input matrix T.
    %
    % Example:
    %   T = [R, p; 0, 0, 0, 1]; % where R is a 3x3 rotation matrix and p is a 3x1 translation vector
    %   iT = invT(T);
    iR = inv(T(1:3, 1:3)); % Pseudo-inverse of the rotation matrix
    iT = [iR, -iR * T(1:3, 4);
          zeros(1, 3), 1];
end
