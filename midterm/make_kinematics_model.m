function [S, M] = make_kinematics_model()
    % MAKE_KINEMATICS_MODEL Calculates the Screw Axes and Home Configuration of
    % the EPSON VT6L robot.
    %
    % Inputs: None
    %
    % Output: S - 6xn matrix whose columns are the screw axes of the robot
    %         M - homogeneous transformation representing the home configuration

    % *** YOUR CODE HERE ***
    S = transpose([0, 0, 1, 0, 0, 0;
                   0, 1, 0, -0.412, 0, 0.1;
                   0, 1, 0, -0.832, 0, 0.1;
                   1, 0, 0, 0, 0.832, 0;
                   0, 1, 0, -0.832, 0, 0.5;
                   1, 0, 0, 0, 0.832, 0]);

    M = [0, 0, 1, 0.58;
         0, -1, 0, 0;
         1, 0, 0, 0.832;
         0, 0, 0, 1];

end
