clc;
clear all;
close all;

ur5 = ur5_interface();
tf_frame.get_tf_tree();

is_test_ur5FwdKin = 1;
is_test_ur5BodyJacobian = 0;
is_test_manipulability = 0;
is_test_getXi = 0;
is_test_ur5RRcontrol = 0;


%% Test ur5FwdKin.m
if is_test_ur5FwdKin
    disp("Start Test ur5FwdKin...")
    % Random pose for the other joint angles (avoid singularities)

    % home pose
    q_home = [0; -pi/2; 0; -pi/2; 0; 0];

    angle = 15 * pi/180;

    q = q_home + angle;
    
    q_offset = [0; -pi/2; 0; -pi/2; 0; 0];
    g = ur5FwdKin(q-q_offset);

    ur5.move_joints(q, 5);
    pause(5);
    fwd_real = ur5.get_current_transformation('base_link', 'tool0');
    
    
    fwdKinToolFrame = tf_frame('base_link','fwdKinToolFrame',eye(4));
    fwdKinToolFrame.move_frame('base_link', g);
    pause(3);
    fwd_calculated = fwdKinToolFrame.read_frame('base_link');


    disp(fwd_calculated);
    disp(fwd_real);
    disp(fwd_calculated - fwd_real);
end


%% Test ur5BodyJacobian.m
if is_test_ur5BodyJacobian
    % Parameters
    epsilon = 1e-6;           % Small perturbation value
    q = rand(6, 1) * 2 * pi;  % Random joint configuration in [0, 2π]
    
    % Compute the exact Jacobian using ur5BodyJacobian
    gst = ur5FwdKin(q);       % Forward kinematics to get gst
    J = ur5BodyJacobian(q);  % Exact Jacobian
    
    % Initialize approximate Jacobian
    J_approx = zeros(6, 6);
    
    % Compute approximate Jacobian column-by-column
    for i = 1:6
        % Perturb the i-th joint angle by +epsilon and -epsilon
        q_plus = q;
        q_minus = q;
        q_plus(i) = q_plus(i) + epsilon;
        q_minus(i) = q_minus(i) - epsilon;
    
        % Forward kinematics for perturbed configurations
        gst_plus = ur5FwdKin(q_plus);
        gst_minus = ur5FwdKin(q_minus);
    
        % Central difference approximation for ∂g/∂q_i
        dg_dqi = (gst_plus - gst_minus) / (2 * epsilon);
    
        % Compute g⁻¹ ∂g/∂q_i
        twist_matrix = inv(gst) * dg_dqi;
    
        % Twistify: Convert the 4x4 matrix to a 6D twist vector
        xi = Twistify(twist_matrix);
    
        % Store the twist as the i-th column of J_approx
        J_approx(:, i) = xi;
    end
    
    % Compute the error norm ||J_approx - J_actual||
    errorNorm = norm(J_approx - J);
    
    % Display results
    disp('Actual Jacobian (J):');
    disp(J);
    
    disp('Approximate Jacobian (J_approx):');
    disp(J_approx);
    
    disp('Norm of the error ||J_approx - J_actual||:');
    disp(errorNorm);
    
end

%% Test manipulability.m
if is_test_manipulability
    disp("Start Test manipulability...")
    theta3_values = linspace(-pi/4, pi/4, 100); % Increment θ3 from -π/4 to π/4
    num_points = length(theta3_values);
    
    % Random pose for the other joint angles (avoid singularities)
    theta1 = 0.1 * pi; % Random angle in [0, 2π]
    theta2 = 0.1 * pi;
    theta4 = 0.1 * pi;
    theta5 = 0.1 * pi;
    theta6 = 0.1 * pi;
    
    % Initialize arrays to store manipulability measures
    sigmamin_values = zeros(1, num_points);
    detjac_values = zeros(1, num_points);
    invcond_values = zeros(1, num_points);
    
    % Loop over θ3 values
    for i = 1:num_points
        % Joint configuration
        q = [theta1; theta2; theta3_values(i); theta4; theta5; theta6];
        
        % Compute the body Jacobian
        Jb = ur5BodyJacobian(q);
        
        % Compute each manipulability measure
        sigmamin_values(i) = manipulability(Jb, 'sigmamin');
        detjac_values(i) = manipulability(Jb, 'detjac');
        invcond_values(i) = manipulability(Jb, 'invcond');
    end
    
    % Plot the manipulability measures
    figure;
    
    % Plot sigmamin
    subplot(3, 1, 1);
    plot(theta3_values, sigmamin_values, 'LineWidth', 1.5);
    xlabel('\theta_3 (radians)');
    ylabel('sigmamin');
    title('Manipulability Measure: sigmamin');
    grid on;
    
    % Plot detjac
    subplot(3, 1, 2);
    plot(theta3_values, detjac_values, 'LineWidth', 1.5);
    xlabel('\theta_3 (radians)');
    ylabel('detjac');
    title('Manipulability Measure: detjac');
    grid on;
    
    % Plot invcond
    subplot(3, 1, 3);
    plot(theta3_values, invcond_values, 'LineWidth', 1.5);
    xlabel('\theta_3 (radians)');
    ylabel('invcond');
    title('Manipulability Measure: invcond');
    grid on;
    
    % Ensure the plots are visually separated
    sgtitle('Manipulability Measures Near a Singularity');

end


%% Test getXi.m
if is_test_getXi
    disp("Start Test getXi...")
    % Test1
    fprintf('Test 1: \n');
    Original_g = homogeneous_matrix([pi/2, 0, 0],[0; 0; 1]);
    testgetXi(Original_g);
    
    % Test2
    fprintf('Test 2: \n');
    Original_g = homogeneous_matrix([pi/2, pi/3, -pi/4],[1; 3; 2]);
    testgetXi(Original_g);
    
    % Test3
    fprintf('Test 3: \n');
    Original_g = homogeneous_matrix([pi/5, -pi/4, 0],[1; 1; 1]);
    testgetXi(Original_g);
    
    % Test4
    fprintf('Test 4: \n');
    Original_g = homogeneous_matrix([pi/4, pi/4, pi/4],[2; 3; 4]);
    testgetXi(Original_g);

end


% Test ur5RRcontrol.m
if is_test_ur5RRcontrol
    %% Test ur5RRcontrol.m
    disp("Start Test ur5RRcontrol...")
    % Case 1
    q=[pi/4; pi/6; -pi/6; -pi/4; pi/4; -pi/3];
    ur5.move_joints(q, 10);
    pause(10);
    disp("Moved to the initial joint position");
    %%
    q = [-pi/4; -pi/6; pi/6; -pi/2; pi/6; 0];
    gdesired=ur5FwdKin(q);
    ur5RRcontrol(gdesired, 10.0, ur5);
    pause(20);
    %% Case 2
    q=ur5.home;
    ur5.move_joints(q, 10);
    pause(10)
    q=[-pi/3; pi/2; -pi/3; pi/2; pi/3; -pi/4];
    gdesired=ur5FwdKin(q);
    ur5RRcontrol(gdesired, 10.0, ur5);
    pause(10);

end


%% Helper Functions

function xi = Twistify(g)
    % Convert a 4x4 matrix to a 6x1 twist vector
    %
    % Input:
    %   g - 4x4 homogeneous transformation matrix
    % Output:
    %   xi - 6x1 twist vector [v; omega]

    % Extract rotation (upper-left 3x3 block)
    R = g(1:3, 1:3);

    % Force skew-symmetric adjustment
    R_skew = (R - R') / 2;

    omega = SkewToAxis(R_skew);
    v = g(1:3, 4);
    xi = [v; omega];
end

function w = SkewToAxis(S)
    w = [S(3, 2); S(1, 3); S(2, 1)];
end

%% Test the getXi function by reconstructing and comparing matrices
function testgetXi(g)    
    xi = getXi(g);
    omega_hat = skewSymmetric(xi(4:6)); 
    v = xi(1:3);
    reconstructed_g = expm([omega_hat, v; 0 0 0 0]);
    
    diff = norm(g - reconstructed_g, 'fro');
    fprintf('Difference: %e\n', diff);
end

function S = skewSymmetric(v)
    S = [ 0    -v(3)  v(2);
          v(3)  0    -v(1);
         -v(2)  v(1)  0];
end

% Construct a 4x4 homogeneous transformation matrix.
function M_h = homogeneous_matrix(angles_XYZ,translation)
    roll=angles_XYZ(1);
    pitch=angles_XYZ(2);
    yaw=angles_XYZ(3);

    Rx = [1               0          0;
         0            cos(roll)      -sin(roll);
         0            sin(roll)      cos(roll);];
    Ry = [cos(pitch)      0          sin(pitch);
         0                1          0         ;
         -sin(pitch)      0          cos(pitch);];
    Rz = [cos(yaw)    -sin(yaw)      0;
         sin(yaw)     cos(yaw)       0;
            0            0           1;];

    R = Rx*Ry*Rz;
    M_h = [ R   ,   translation(:);
            0,  0,  0,  1];
end


