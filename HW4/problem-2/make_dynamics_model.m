function [Mlist, Glist] = make_dynamics_model(robot)
    % MAKE_KINEMATICS_MODEL Creates the dynamics model of the robot
    %
    % Inputs: None
    %
    % Output: Mlist - 4x4x7 matrix containing all the transformation matrices between consecutive link frames
    %         Glist - 6x6x6 matrix containing the spatial inertia matrices of each link

    %% Link poses when the robot is in the home configuration
    [M01, M12, M23, M34, M45, M56, M67] = calculatelinkframes(robot);
    Mlist = cat(3, M01, M12, M23, M34, M45, M56, M67);

    %% Spatial Inertia Matrices
    % *** Define the link inertial properties ***
    G1 = buildGmatrix([0.010267495893, 0.010267495893, 0.00666], 3.7);
    G2 = buildGmatrix([0.22689067591, 0.22689067591, 0.0151074], 8.393);
    G3 = buildGmatrix([0.049443313556, 0.049443313556, 0.004095], 2.275);
    G4 = buildGmatrix([0.111172755531, 0.111172755531, 0.21942], 1.219);
    G5 = G4;
    G6 = buildGmatrix([0.0171364731454, 0.0171364731454, 0.033822], 0.1879);
    Glist = cat(3, G1, G2, G3, G4, G5, G6);

end
