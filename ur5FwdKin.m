function gst = ur5FwdKin(q)
    % Input: 
    %   q: 6x1 joint space variable vector = [θ1, θ2, θ3, θ4, θ5, θ6].T, where θn is the angle of joint n for n = 1, · · · , 6.
    % Output: 
    %   gst: end effector pose, gst (4 × 4 matrix)

    th1 = q(1);
    th2 = q(2);
    th3 = q(3);
    th4 = q(4);
    th5 = q(5);
    th6 = q(6);
    
    l1 = 0; 
    l2 = 0; 
    l3 = 0; 
    l4 = 0; 
    l5 = 0; 
    l6 = 0;

    % Compute the forward kinematics matrix
    gst = [
        % TODO
    ];
end
