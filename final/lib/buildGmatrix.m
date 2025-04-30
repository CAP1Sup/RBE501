function G = buildGmatrix(Is, m)
    % BUILDGMATRIX Constructs the G matrix for a rigid body.
    %
    %   G = buildGmatrix(Is, m) computes the G matrix, which is a 6x6 matrix
    %   used in dynamics calculations for a rigid body. The matrix is composed
    %   of the inertia tensor and mass of the body.
    %
    %   Inputs:
    %       Is - A 3x3 matrix representing the inertia tensor of the rigid body
    %            about its center of mass.
    %       m  - A scalar representing the mass of the rigid body.
    %
    %   Outputs:
    %       G  - A 6x6 matrix where the top-left 3x3 block is the inertia tensor
    %            (Is), the bottom-right 3x3 block is the mass times the identity
    %            matrix (m * eye(3)), and the other blocks are zero matrices.
    %
    %   Example:
    %       Is = [1, 0, 0; 0, 2, 0; 0, 0, 3];
    %       m = 5;
    %       G = buildGmatrix(Is, m);
    %
    %   See also: DIAG, EYE, ZEROS

    G = [diag(Is), zeros(3);
         zeros(3), m * eye(3)];
end
