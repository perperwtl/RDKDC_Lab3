function xi = getXi(g)
    R = g(1:3, 1:3); % rotation part
    p = g(1:3, 4);   % translation part
    
    theta = acos((trace(R) - 1) / 2);
    
    if abs(theta) < 1e-6
        omega = [0; 0; 0];        
        v = p / norm(p);        
        theta = norm(p);         
    else
        omega_hat = (R - R') / (2 * sin(theta));
        omega = [omega_hat(3, 2); omega_hat(1, 3); omega_hat(2, 1)]; 
        
        A = eye(3) * theta + (1 - cos(theta)) * omega_hat + (theta - sin(theta)) * omega_hat ^ 2;
        v = A \ p; 
    end
    
    xi = [v * theta;  omega * theta];
end

