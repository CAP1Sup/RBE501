function AdT = invadjoint(T)
    % INVADJOINT - Computes the inverse adjoint transformation matrix.
    %
    % Syntax: AdT = invadjoint(T)
    %
    % Inputs:
    %    T - A 4x4 homogeneous transformation matrix.
    %
    % Outputs:
    %    AdT - A 6x6 inverse adjoint transformation matrix.
    %
    % Example:
    %    T = [1 0 0 1;
    %         0 1 0 2;
    %         0 0 1 3;
    %         0 0 0 1];
    %    AdT = invadjoint(T);
    %
    % See also: ADJOINT, SKEW

    tR = transpose(T(1:3, 1:3));
    P = T(1:3, 4);
    AdT = [tR, zeros(3);
           -tR * skew(P), tR];
end
