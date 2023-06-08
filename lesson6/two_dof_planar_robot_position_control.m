clear all;
close all;

% Length
l1 = 1;
l2 = 1;

% Sampling time
tau = 0.001; 

% Control threshold 
control_threshold = 0.01;

% Control gain
eta = 200;

% Create robot
two_dof_planar_robot = TwoDofPlanarRobot(l1, l2);

% Initial joint value 
theta1 = 0;
theta2 = pi/2;

% Desired task-space position
% tx 
tx =0;
% vy 
ty =2;

% Compose the end effector position into a pure quaternion
td = DQ([tx ty 0]);

% Position controller. The simulation ends when the 
% derivative of the error is below a threshold
time = 0;
t_error_dot = DQ(1); % Give it an initial high value
t_error = DQ(1); % Give it an initial high value
while norm(vec4(t_error_dot)) > control_threshold
    % Get the current translation
    [a, t] = two_dof_planar_robot.fkm1(theta1, theta2);
    
    % Calculate the error and old error
    t_error_old = t_error;
    t_error = (t-td);
    t_error_dot = (t_error-t_error_old)/tau;
    
    % Get the translation jacobian, based on theta
    Jt = two_dof_planar_robot.translation_jacobian(theta1, theta2);
    
    % Calculate the IDKM
    theta_dot = -eta*pinv(Jt)*vec3(t_error);
    
    % Move the robot
    theta1 = theta1 + theta_dot(1)*tau;
    theta2 = theta2 + theta_dot(2)*tau;
    
    % Plot the robot
    two_dof_planar_robot.plot(theta1, theta2)
    
    % Plot the desired position
    hold on
    plot(tx,ty,'bx')
    hold off
    
    % [For animations only]
    drawnow; % [For animations only] Ask MATLAB to draw the plot now
    pause(0.001) % [For animations only] Pause so that MATLAB has enough time to draw the plot
    
end