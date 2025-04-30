function G = buildRectPrismGmatrix(l, w, h, m)
    % BUILDRECTPRISMGMATRIX - Computes the spatial inertia matrix (G) for a rectangular prism.
    %
    % This function calculates the spatial inertia matrix for a rectangular prism
    % given its dimensions (length, width, height) and mass. The inertia tensor
    % components are computed assuming the prism is a rigid body with uniform
    % density and its center of mass is located at the geometric center.
    %
    % Inputs:
    %     l - Length of the rectangular prism (scalar, in meters).
    %     w - Width of the rectangular prism (scalar, in meters).
    %     h - Height of the rectangular prism (scalar, in meters).
    %     m - Mass of the rectangular prism (scalar, in kilograms).
    %
    % Outputs:
    %     G - Spatial inertia matrix (6x6 matrix) for the rectangular prism.
    %
    % Dependencies:
    %     This function calls `buildGmatrix`, which constructs the spatial inertia
    %     matrix given the principal moments of inertia and mass.
    %
    % Example:
    %     % Define dimensions and mass
    %     length = 2.0; % meters
    %     width = 1.0;  % meters
    %     height = 0.5; % meters
    %     mass = 10.0;  % kilograms
    %
    %     % Compute the spatial inertia matrix
    %     G = buildRectPrismGmatrix(length, width, height, mass);

    Ixx = (m / 12) * (w ^ 2 + h ^ 2);
    Iyy = (m / 12) * (l ^ 2 + h ^ 2);
    Izz = (m / 12) * (l ^ 2 + w ^ 2);
    G = buildGmatrix([Ixx, Iyy, Izz], m);
end
