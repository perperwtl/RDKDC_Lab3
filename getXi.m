function xi = getXi(g)
    % getXi: Extracts the unscaled twist from a homogeneous transformation matrix
    % Input: g - Homogeneous transformation matrix (4x4)
    % Output: xi - The unnormalized twist in 6x1 vector or twist coordinate form

    if size(g, 1) ~= 4 || size(g, 2) ~= 4
        error('Invalid input.');
    end

    R = g(1:3, 1:3); % rotation part
    p = g(1:3, 4);  % translation part

    omega_hat = logm(R); 

    omega = [omega_hat(3, 2); omega_hat(1, 3); omega_hat(2, 1)];
    omega_norm = norm(omega);
    if omega_norm > 1e-6
        M = (eye(3) - R) * omega_hat / omega_norm^2 + omega * omega.' / omega_norm^2;
        v = M \ p; 
    else
        v = p;
    end

    xi = [v; omega];
end

