function out = skew(v)
    % SKEW  Generates the skew-symmetric matrix of a 3-element vector.
    %
    %   out = skew(v) returns the 3x3 skew-symmetric matrix of the input vector v.
    %
    %   Input:
    %       v - A 3-element vector [v1; v2; v3]
    %
    %   Output:
    %       out - A 3x3 skew-symmetric matrix corresponding to the input vector v:
    %             [  0   -v3   v2;
    %               v3    0   -v1;
    %              -v2   v1    0 ]
    %
    %   Example:
    %       v = [1; 2; 3];
    %       out = skew(v);
    %       % out will be:
    %       % [  0  -3   2;
    %       %    3   0  -1;
    %       %   -2   1   0 ]

    out = [0, -v(3), v(2);
           v(3), 0, -v(1);
           -v(2), v(1), 0];
end
