function [robot, jointLimits] = make_robot()
%MAKE_ROBOT Creates the kinematic structure of the robot used in the exam
%   This is a factory function that creates the robot needed for the exam.
%
%   Inputs: None
%   Output: robot - the robot structure, created using Peter Corke's
%   robotics toolbox
%
%   Author: L. Fichera <lfichera@wpi.edu>
%   Last modified: 4/29/2025

L1 = 300e-3;
L2 = 250e-3;

robot = SerialLink([Revolute('d', 0, 'a', L1, 'alpha', 0), ...
                    Revolute('d', 0, 'a', L2, 'alpha', pi), ...
                    Prismatic('theta', 0, 'a', 0, 'alpha', 0, 'qlim', [0 180e-3])], ...
                    'name', 'ABB IRB 910C/0.55');

jointLimits = [[-140   140] * pi/180;  % q(1)
               [-150   150] * pi/180;  % q(2)
               [  0   180]  * 1e-3]; % q(3)   

end

