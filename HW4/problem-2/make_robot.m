function robot = make_robot()
    %MAKE_ROBOT Creates the kinematic structure of the robot used in homework 4, problem 2.
    %
    %   This is a factory function that creates the robot used in the homework.
    %
    %   Inputs: none
    %
    %   Output: robot - the robot structure, created using Peter Corke's
    %   robotics toolbox
    %
    %   Author: L. Fichera <lfichera@wpi.edu>
    %   Last modified: 04/07/2025

    %% Create the manipulator
    mdl_ur5;
    robot = ur5;

end
