function traj = make_trajectory (type, params)
    % MAKE_TRAJECTORY Generates a trajectory based on the specified type and parameters.
    %
    %   traj = MAKE_TRAJECTORY(type, params) computes a trajectory of the specified
    %   type ('cubic' or 'quintic') using the given parameters. The function returns
    %   a structure containing the time vector, position, velocity, and acceleration
    %   of the trajectory.
    %
    %   Input Arguments:
    %       type    - A string specifying the type of trajectory. Supported values:
    %                 'cubic'  - Generates a cubic trajectory.
    %                 'quintic' - Generates a quintic trajectory.
    %       params  - A structure containing the following fields:
    %                 t  - A 2-element vector specifying the start and end times [t1, t2].
    %                 q  - A 2-element vector specifying the start and end positions [q1, q2].
    %                 v  - A 2-element vector specifying the start and end velocities [v1, v2].
    %                 a  - (Required for 'quintic') A 2-element vector specifying the
    %                      start and end accelerations [a1, a2].
    %                 dt - A scalar specifying the time step for the trajectory.
    %
    %   Output:
    %       traj - A structure containing the following fields:
    %              t  - A vector of time values from t1 to t2 with step size dt.
    %              q  - A vector of position values corresponding to the time vector.
    %              v  - A vector of velocity values corresponding to the time vector.
    %              a  - A vector of acceleration values corresponding to the time vector.
    %
    %   Example:
    %       params.t = [0, 2];
    %       params.q = [0, 10];
    %       params.v = [0, 0];
    %       params.a = [0, 0];
    %       params.dt = 0.01;
    %       traj = make_trajectory('quintic', params);
    %
    %   See also POLYVAL, POLYDER, MLDIVIDE.

    if strcmp(type, 'cubic')
        % Build the matrices, then calculate the coefficients
        A = [1, params.t(1), params.t(1) ^ 2, params.t(1) ^ 3;
             0, 1, 2 * params.t(1), 3 * params.t(1) ^ 2;
             1, params.t(2), params.t(2) ^ 2, params.t(2) ^ 3;
             0, 1, 2 * params.t(2), 3 * params.t(2) ^ 2];
        B = [params.q(1); params.v(1); params.q(2); params.v(2)];
        coeffs = flip(mldivide(A, B));

    elseif strcmp(type, 'quintic')
        % Build the matrices, then calculate the coefficients
        A = [1, params.t(1), params.t(1) ^ 2, params.t(1) ^ 3, params.t(1) ^ 4, params.t(1) ^ 5;
             0, 1, 2 * params.t(1), 3 * params.t(1) ^ 2, 4 * params.t(1) ^ 3, 5 * params.t(1) ^ 4;
             0, 0, 2, 6 * params.t(1), 12 * params.t(1) ^ 2, 20 * params.t(1) ^ 3;
             1, params.t(2), params.t(2) ^ 2, params.t(2) ^ 3, params.t(2) ^ 4, params.t(2) ^ 5;
             0, 1, 2 * params.t(2), 3 * params.t(2) ^ 2, 4 * params.t(2) ^ 3, 5 * params.t(2) ^ 4;
             0, 0, 2, 6 * params.t(2), 12 * params.t(2) ^ 2, 20 * params.t(2) ^ 3];
        B = [params.q(1); params.v(1); params.a(1); params.q(2); params.v(2); params.a(2)];
        coeffs = flip(mldivide(A, B));

    else
        error("Unknown trajectory type");
    end

    traj.t = params.t(1):params.dt:params.t(2);
    traj.q = polyval(coeffs, traj.t);
    traj.v = polyval(polyder(coeffs), traj.t);
    traj.a = polyval(polyder(polyder(coeffs)), traj.t);
end
