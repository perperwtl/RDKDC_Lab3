clc;
clear all;

% function gst = ur5FwdKin(joints)
syms th1 th2 th3 th4 th5 th6
syms L0 L1 L2 L3 L4 L5
syms l0 l1 l2 l3 l4 l5

% Define unit vectors
e1 = [1; 0; 0];
e2 = [0; 1; 0];
e3 = [0; 0; 1];

% Define axes of rotation for each joint
w1 = e3;  % Axis 1: Rotation about z-axis
w2 = e2;  % Axis 2: Rotation about y-axis
w3 = e2;  % Axis 3: Rotation about y-axis
w4 = e3;  % Axis 4: Rotation about y-axis
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
gst0 = RPToHomogeneous([e1,-e3,e2], [0; l3 + l5; l0+l1+l2+l4]);%reverse x and y axis

% Compute forward kinematics with joint offsets
joint_offsets = [0; -pi/2; 0; -pi/2; 0; 0]; % New initial configuration offsets
% joint_offsets = [0;0;0;0;0;0];
gst = ForwardKinematics({xi1, th1 + joint_offsets(1); 
                         xi2, th2 + joint_offsets(2); 
                         xi3, th3 + joint_offsets(3); 
                         xi4, th4 + joint_offsets(4); 
                         xi5, th5 + joint_offsets(5); 
                         xi6, th6 + joint_offsets(6)}, gst0);

disp('Forward Kinematics Map (Simplified):');
disp(simplify(gst));

% Compute the body Jacobian
Jb = BodyJacobian({xi1, th1; xi2, th2; ...
                   xi3, th3; xi4, th4; ...
                   xi5, th5; xi6, th6}, gst0);

disp('Body Jacobian (Simplified):');
disp(simplify(Jb));

%% Helper Functions

% ForwardKinematics Function
function g = ForwardKinematics(twists, gst0)
    % Input: twists - cell array of {xi, th}, gst0 - home configuration
    % Output: g - 4x4 transformation matrix of end-effector

    g = eye(4);  % Start with identity matrix

    for i = 1:size(twists, 1)
        xi = twists{i, 1};
        th = twists{i, 2};
        g = g * TwistExp(xi, th);
    end

    g = g * gst0;  % Multiply by home configuration
end

% BodyJacobian Function
function Jb = BodyJacobian(twists, gst0)
    % Computes the body Jacobian
    % Input: q: 6 × 1 joint space variables vector
    % Output: J: Body Jacobian, Jbst (6 × 6 matrix)

    g = gst0;  % Start with home configuration
    Jb = [];

    for i = size(twists, 1):-1:1
        xi = twists{i, 1};
        th = twists{i, 2};
        Jb = [RigidAdjoint(inv(g)) * xi, Jb];  % Compute adjoint and prepend
        Jb = simplify(Jb);
        g = TwistExp(xi, th) * g;  % Update transformation
    end
end

% RevoluteTwist Function
function xi = RevoluteTwist(q, w)
    v = -cross(q, w);
    xi = [v; w];
end


% TwistExp Function
function g = TwistExp(xi, th)
    % Computes the matrix exponential of a twist
    % Input: xi - 6x1 twist vector, th - joint angle/displacement
    % Output: g - 4x4 homogeneous transformation matrix

    w = xi(4:6);  % Angular velocity
    v = xi(1:3);  % Linear velocity
    w_skew = hat(w);

    if norm(w) == 0
        % Pure translation
        R = eye(3);
        p = v * th;
    else
        % Rotation with translation
        R = eye(3) + sin(th) * w_skew + (1 - cos(th)) * w_skew^2;
        p = (eye(3) - R) * cross(w, v) + (w * (w' * v)) * th;
    end

    g = [R, p; 0, 0, 0, 1];
end

% RPToHomogeneous Function
function g = RPToHomogeneous(R, p)
    % Converts rotation matrix and position vector into homogeneous form
    % Input: R - 3x3 rotation matrix, p - 3x1 position vector
    % Output: g - 4x4 homogeneous transformation matrix

    g = [R, p; 0, 0, 0, 1];
end

% AxisToSkew Function
function S = hat(w)
    % Converts a 3D vector to a skew-symmetric matrix
    S = [0, -w(3), w(2);
         w(3), 0, -w(1);
        -w(2), w(1), 0];
end

% RigidAdjoint Function
function Ad = RigidAdjoint(g)

    R = g(1:3, 1:3);  % Rotation part
    p = g(1:3, 4);    % Translation part
    Ad = [R, hat(p) * R;
          zeros(3, 3), R];
end
