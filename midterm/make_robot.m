function robot = make_robot()
%MAKE_ROBOT Creates the kinematic structure of the EPSON VT6L robot.
%
%   Inputs: None
%
%   Output: robot - the robot structure, created using Peter Corke's
%   robotics toolbox
%
%   Author: L. Fichera <lfichera@wpi.edu>
%   Last modified: 2/27/2025

d1 = 412e-3;
a1 = 100e-3;
a2 = 420e-3;
d4 = 400e-3;
d6 = 80e-3;

robot = SerialLink([Revolute('d', d1, 'a', a1, 'alpha', -pi/2), ...
                    Revolute('d', 0,  'a', a2, 'alpha', 0, 'offset', -pi/2), ...
                    Revolute('d', 0,  'a', 0, 'alpha', -pi/2), ...
                    Revolute('d', d4, 'a', 0,  'alpha', pi/2), ...
                    Revolute('d', 0,  'a', 0, 'alpha', -pi/2), ...
                    Revolute('d', d6,  'a', 0, 'alpha', 0)], ...
                    'name', 'Epson VT6L');
end

