%% Test manipulability.m
theta3_values = linspace(-pi/4, pi/4, 100); % Increment θ3 from -π/4 to π/4
num_points = length(theta3_values);

% Random pose for the other joint angles (avoid singularities)
theta1 = rand() * 2 * pi; % Random angle in [0, 2π]
theta2 = rand() * 2 * pi;
theta4 = rand() * 2 * pi;
theta5 = rand() * 2 * pi;
theta6 = rand() * 2 * pi;

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
