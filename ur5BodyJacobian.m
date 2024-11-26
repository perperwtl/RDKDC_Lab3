function Jb = ur5BodyJacobian(q)
    % ur5BodyJacobian - Compute the Jacobian matrix for the UR5 robot
    % Input: 
    %   q: 6x1 joint space variables vector = [θ1, θ2, θ3, θ4, θ5, θ6].T
    % Output: 
    %   Jb: 6x6 Body Jacobian matrix
    
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

    % Define the Body Jacobian 
    Jb = [

    ];
end
