function pose = ht2pose(T)
    pose = MatrixLog6(T);
    pose = [pose(3, 2) pose(1, 3) pose(2, 1) pose(1:3, 4)']';
end
