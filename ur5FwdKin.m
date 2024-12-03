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
    
    l0 = 0.0892;
    l1 = 0.425; 
    l2 = 0.392; 
    l3 = 0.1093; 
    l4 = 0.09475; 
    l5 = 0.0825; 

    % Define unit vectors
    e1 = [1; 0; 0];
    e2 = [0; 1; 0];
    e3 = [0; 0; 1];
    
    % Define axes of rotation for each joint
    w1 = e3;  % Axis 1: Rotation about z-axis
    w2 = e2;  % Axis 2: Rotation about y-axis
    w3 = e2;  % Axis 3: Rotation about y-axis
    w4 = e2;  % Axis 4: Rotation about y-axis
    w5 = e3;  % Axis 5: Rotation about z-axis
    w6 = e2;  % Axis 6: Rotation about y-axis
    
    % Define points on the joint axes
    q1 = [0; 0; 0];
    q2 = [0; 0; l0];
    q3 = [0; 0; l0+l1];
    q4 = [0; 0; l0+l1+l2];
    q5 = [0; l3; l0+l1+l2];
    q6 = [0; 0; l0+l1+l2+l4];
    
    % Define twists
    xi1 = RevoluteTwist(q1, w1);
    xi2 = RevoluteTwist(q2, w2);
    xi3 = RevoluteTwist(q3, w3);
    xi4 = RevoluteTwist(q4, w4);
    xi5 = RevoluteTwist(q5, w5);
    xi6 = RevoluteTwist(q6, w6);
    
    % Home configuration of the end-effector
    gst0 = [[e1,-e3,e2], [0; l3 + l5; l0+l1+l2+l4]; 0,0,0,1];
    
    gst = expm(TwistExp(xi1)* th1) * expm(TwistExp(xi2)* th2) ...
    * expm(TwistExp(xi3)* th3) * expm(TwistExp(xi4)* th4) ...
    * expm(TwistExp(xi5)* th5) *expm(TwistExp(xi6)* th6) * gst0; 

end
