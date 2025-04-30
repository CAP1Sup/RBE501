addpath('lib');

% KINEMATICS
n = 3; % degrees of freedom
L1 = 0.3; % Length of Link 1 [m]
L2 = 0.3; % Length of Link 2 [m]
L3 = 0.15; % Length of Link 3 [m]

% Screw Axes
S = [0 0 1 0 0 0;
     1 0 0 -cross([1 0 0], [0 0 L1]);
     1 0 0 -cross([1 0 0], [0 L2 L1])]';

% Home configuration matrix
R_home = [0 0 -1; 1 0 0; 0 -1 0]';
t_home = [0 L2 L1 - L3]';
M = [R_home t_home; 0 0 0 1];

% Link frames in the home configuration
M01 = [eye(3) [0 0 L1 / 2]'; 0 0 0 1];
M12 = [[1 0 0; 0 0 1; 0 -1 0], [0 L2 / 2 L1 / 2]'; 0 0 0 1];
M23 = [[1 0 0; 0 0 1; 0 -1 0], [0 L3 / 2 L2 / 2]'; 0 0 0 1];
M34 = [[0 1 0; 0 0 1; 1 0 0], [0 0 L3 / 2]'; 0 0 0 1];

M1 = M01;
M2 = M1 * M12;
M3 = M2 * M23;
M4 = M3 * M34;

M = cat(3, M01, M12, M23, M34);

% INERTIAL PROPERTIES
m1 = 5; % Mass of Link 1 [kg]
m2 = 1; % Mass of Link 2 [kg]
m3 = 1; % Mass of Link 3 [kg]
w = 0.04; % Link Width [m]
l = 0.04; % Link Depth [m]

% Spatial Inertia Matrices
G1 = zeros(6, 6);
Ixx1 = m1 * (w ^ 2 + L1 ^ 2) / 12;
Iyy1 = m1 * (l ^ 2 + L1 ^ 2) / 12;
Izz1 = m1 * (l ^ 2 + w ^ 2) / 12;
G1(1:3, 1:3) = diag([Ixx1 Iyy1 Izz1]);
G1(4:6, 4:6) = m1 * eye(3);

G2 = zeros(6, 6);
Ixx2 = m2 * (w ^ 2 + L2 ^ 2) / 12;
Iyy2 = m2 * (l ^ 2 + L2 ^ 2) / 12;
Izz2 = m2 * (l ^ 2 + w ^ 2) / 12;
G2(1:3, 1:3) = diag([Ixx2 Iyy2 Izz2]);
G2(4:6, 4:6) = m2 * eye(3);

G3 = zeros(6, 6);
Ixx3 = m2 * (w ^ 2 + L3 ^ 2) / 12;
Iyy3 = m2 * (l ^ 2 + L3 ^ 2) / 12;
Izz3 = m2 * (l ^ 2 + w ^ 2) / 12;
G3(1:3, 1:3) = diag([Ixx3 Iyy3 Izz3]);
G3(4:6, 4:6) = m3 * eye(3);

G = cat(3, G1, G2, G3);

%% *** FINALLY, LET US ASSEMBLE THE STRUCTURE WE NEED TO PASS TO THE RNE FUNCTION ***
params.g = [0 0 -9.81]; % Gravity Vector [m/s^2]
params.S = S;
params.M = M;
params.G = G;
params.jointPos = [0 0 pi / 4]'; % Current Joint Variables
params.jointVel = [1 2 3]'; % Current Joint Velocities
params.jointAcc = [1 2 3]'; % Current Joint Accelerations
params.Ftip = [0 0 0 0 0 1]'; % Wrench applied at the tip

[tau, V, Vdot] = rne(params);
transpose(tau)
transpose(V)
transpose(Vdot)
