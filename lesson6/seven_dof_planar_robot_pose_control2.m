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
tx = 5;
ty = 2;
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