function [tau, V, Vdot] = rne(params)
    %% RNE Implements the Recursive Newton-Euler Inverse Dynamics Algorithm
    %
    % Inputs: params - a structure containing the following fields:
    %           params.g - 3-dimensional column vector describing the acceleration of gravity
    %           params.S - 6xn matrix of screw axes (each column is an axis)
    %           params.M - 4x4xn home configuration matrix for each link
    %           params.G - 6x6xn spatial inertia matrix for each link
    %           params.jointPos - n-dimensional column vector of joint coordinates
    %           params.jointVel - n-dimensional column vector of joint velocities
    %           params.jointAcc - n-dimensional column vector of joint accelerations
    %           params.Ftip - 6-dimensional column vector representing the
    %           wrench applied at the tip
    %
    % Output: tau  - n-dimensional column vector of generalized joint forces
    %         V    - 6x(n+1) matrix - each column represents the twist of one of the robot's links
    %         Vdot - 6x(n+1) matrix - each column represents the acceleration of one of the robot's links
    %

    % Variable initialization
    n = size(params.S, 2);
    tau = zeros(n, 1);
    V = zeros(6, n + 1);
    Vdot = zeros(6, n + 1);
    A = [twistsspace2local(params.S, params.M)];
    W = zeros(6, n + 1);

    % Seed Vdot and W terms
    Vdot(4:6, 1) = -params.g;
    W(:, n + 1) = params.Ftip;

    % Forward iterations
    for i = 1:n
        adjT = adjoint(invT(params.M(:, :, i) * twist2ht(A(:, i), params.jointPos(i))));
        V(:, i + 1) = adjT * V(:, i) + A(:, i) .* params.jointVel(i);
        Vdot(:, i + 1) = adjT * Vdot(:, i) + ad(V(:, i + 1)) * A(:, i) .* params.jointVel(i) + A(:, i) .* params.jointAcc(i);
    end

    % Pad the screw axes and joint positions to allow for the backward iterations
    A = [A, zeros(6, 1)];
    params.jointPos = [params.jointPos; 0];

    % Backward iterations
    for i = n:-1:1
        adjT = adjoint(invT(params.M(:, :, i + 1) * twist2ht(A(:, i + 1), params.jointPos(i + 1))));
        W(:, i) = params.G(:, :, i) * Vdot(:, i + 1) - transpose(ad(V(:, i + 1))) * params.G(:, :, i) * V(:, i + 1) + transpose(adjT) * W(:, i + 1);
        tau(i) = transpose(W(:, i)) * A(:, i);
    end

end
