function adV = ad(V)
    % AD - Computes the adjoint representation of a twist vector.
    %
    % Syntax:
    %   adV = ad(V)
    %
    % Inputs:
    %   V - A 6x1 twist vector, where the first 3 elements represent angular
    %       velocity and the last 3 elements represent linear velocity.
    %
    % Outputs:
    %   adV - A 6x6 matrix representing the adjoint representation of the
    %         input twist vector. The matrix is structured as:
    %         [skew(angular_velocity), zeros(3);
    %          skew(linear_velocity),  skew(angular_velocity)]
    %

    adV = [skew(V(1:3)), zeros(3);
           skew(V(4:6)), skew(V(1:3))];
end
