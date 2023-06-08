clear all;
close all;
include_namespace_dq

% Sampling time 
tau = 0.01; 
% Simulation time 
final_time = 1;

% Initial joint values 
theta1 = pi/8;
theta2 = pi/8;
theta3 = pi/8;
theta4 = pi/8;
theta5 = pi/8;
theta6 = pi/8;
theta7 = pi/8;

% Arrange the joint values in a column vector
q_init = [theta1 theta2 theta3, theta4, theta5, theta6, theta7]';

% Desired translation components
tx = 2;
ty = -4;
% Desired translation
td = tx*i_ + ty*j_;

% Desired rotation component 
gamma = -pi/8;
% Desired rotation
rd = cos(gamma/2.0) + k_*sin(gamma/2.0);

% Desired pose
xd = rd + 0.5*E_*td*rd;

% Create robot
seven_dof_planar_robot = NDofPlanarRobotDH.kinematics(7);
% Instanteate the controller 
translation_controller = DQ_PseudoinverseController(seven_dof_planar_robot);
translation_controller.set_control_objective(ControlObjective.Pose);
translation_controller.set_gain(5.0);
translation_controller.set_damping(0); % Damping was not explained yet, set it as 0 to use pinv()

% Translation controller loop.
q = q_init;
for time=0:tau:final_time
    % Get the next control signal 
    u = translation_controller.compute_setpoint_control_signal(q,vec8(xd));
    
    % Move the robot
    q = q + u*tau
    
    
    % Plot
    % Plot the robot
    seven_dof_planar_robot.plot(q);
    title(['Pose control' ' time=' num2str(time) 's out of ' num2str(final_time) 's'])
    % Plot the desired pose
    hold on
    plot(xd);
    hold off
    % [For animations only]
    drawnow; % [For animations only] Ask MATLAB to draw the plot now
end