function A = twistsspace2local(V_s, Ts)
    % TWISTSSPACE2LOCAL - Converts twists from the space frame to the local frame.
    %
    % Syntax:
    %   A = twistsspace2local(V_s, Ts)
    %
    % Inputs:
    %   V_s - A 6xN matrix where each column represents a twist in the space frame.
    %         N is the number of joints.
    %   Ts  - A 4x4xN array of transformation matrices, where Ts(:, :, i) is the
    %         transformation matrix from the space frame to the local frame of the
    %         i-th joint.
    %
    % Outputs:
    %   A   - A 6xN matrix where each column represents the twist in the local frame
    %         corresponding to the input twists in the space frame.

    joints = size(V_s, 2);
    A = zeros(6, joints);
    totalT = eye(4);

    for i = 1:joints
        totalT = totalT * Ts(:, :, i);
        A(:, i) = adjoint(invT(totalT)) * V_s(:, i);
    end

end
