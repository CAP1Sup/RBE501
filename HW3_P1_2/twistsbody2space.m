function V_s = twistsbody2space(V_b, T)
    % TWISTSBODY2SPACE - Converts twists from body frame to space frame
    %
    % Syntax: V_s = twistsbody2space(V_b, T)
    %
    % Inputs:
    %    V_b - 6xn matrix of twists in the body frame, where n is the number of joints
    %    T - 4x4 homogeneous transformation matrix representing the pose of the body frame relative to the space frame
    %
    % Outputs:
    %    V_s - 6xn matrix of twists in the space frame
    %
    % Example:
    %    V_b = [0; 0; 1; 0; 0; 0];
    %    T = eye(4);
    %    V_s = twistsbody2space(V_b, T);
    %
    % See also: ADJOINT

    joints = size(V_b, 2);
    V_s = zeros(6, joints);

    for i = 1:joints
        V_s(:, i) = adjoint(T) * V_b(:, i);
    end

end
