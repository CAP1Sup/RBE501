addpath('lib');

params.t = [0 5];
params.dt = 0.01;
params.q = [0 pi / 4];
params.v = [0 0];
params.a = [0 0];

traj = make_trajectory('quintic', params);
traj.q
