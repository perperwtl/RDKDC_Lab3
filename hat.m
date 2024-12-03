function S = hat(w)
    % Converts a 3D vector to a skew-symmetric matrix
    S = [0, -w(3), w(2);
         w(3), 0, -w(1);
        -w(2), w(1), 0];
end