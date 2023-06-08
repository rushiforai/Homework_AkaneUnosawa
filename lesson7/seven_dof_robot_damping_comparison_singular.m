%% Question1 
clear all;
close all;
include_namespace_dq;

% Define the robot
seven_dof_planar_robot = SevenDofPlanarRobotDH.kinematics();
lambda = [0.001, 0.01, 0.1, 0.2, 0.3, 0.4, 0.5, 1];

% Define the controller
translation_controller = DQ_PseudoinverseController(seven_dof_planar_robot);
translation_controller.set_control_objective(ControlObjective.Translation)
translation_controller.set_gain(10)

translation_controller.set_damping(0)

% Desired translation (pure quaternion)
td = 7*j_;
% Sampling time
tau = 0.01; 
% Simulation time 
final_time = 1;
% Initial joint values
q = zeros(7,1);
% Store the control signals
stored_u_1 = [];
stored_t_1 = [];
stored_u_2 = [];
stored_t_2 = [];
t_error = DQ(1);

% Translation controller loop.
% for time=0:tau:final_time
%     % Get the next control signal [rad/s]
%     u = translation_controller.compute_setpoint_control_signal(q,vec4(td));
%     xd = seven_dof_planar_robot.fkm(q);
%     t_error_old = t_error;
%     t_error = vec8(xd-td);
%     t_error_dot = (t_error-t_error_old)/tau;
%    
%     
%     % Move the robot
%     q = q + u*tau;
%     
%     % Store data
%     stored_u_1 = [stored_u_1 u];
%     stored_t_1 = [stored_t_1 t_error];
%     
%     % Plot
% %     % Plot the robot
% %     plot(seven_dof_planar_robot,q);
% %     title(['Pose control' ' time=' num2str(time) 's out of ' num2str(final_time) 's'])
% %     % Plot the desired pose
% %     hold on
% %     plot3(td.q(2),td.q(3),td.q(4));
% %     hold off
% %     % [For animations only]
% %     drawnow; % [For animations only] Ask MATLAB to draw the plot now
% %     pause(0.0001)
% end
k = 1;
i = 1;
while i < 8
    damping = lambda(i);
    translation_controller.set_damping(damping);
    
    theta_1 = 0;
    td = 7*j_;
    tau = 0.01;
    eta = 10;
    % Simulation time
    final_time = 1;
    % Initial joint values 
    q = zeros(7,1);
    t_error = DQ(1);
    
    % Translation controller loop.
    for time=0:tau:final_time
        % Get the next control signal
        u = translation_controller.compute_setpoint_control_signal(q,vec4(td));
        xd = seven_dof_planar_robot.fkm(q);
        t_error_old = t_error;
        t_error = vec8(xd-td);
       
        
        % Move the robot
        q = q + u*tau;
        
        % Store data
        stored_u_2(i,k) = norm(u);
        stored_t_2(i,k) = norm(t_error);
        % Plot
        % Plot the robot
%         plot(seven_dof_planar_robot,q);
%         title(['Pose control' ' time=' num2str(time) 's out of ' num2str(final_time) 's'])
%         % Plot the desired pose
%         hold on
%         plot3(td.q(2),td.q(3),td.q(4));
%         hold off
%         % [For animations only]
%         drawnow; % [For animations only] Ask MATLAB to draw the plot now
%         pause(0.0001)
        k = k+1;
    end
    k = 1;
    i = i + 1;
end
% f1 = figure;
% subplot(1,2,1);
% plot(stored_u_1,0:tau:final_time);
% subplot(1,2,2);
% plot(stored_t_1,0:tau:final_time,10);

f2 = figure;
subplot(1,2,1);
plot_u(stored_u_2,0:tau:final_time,100);
subplot(1,2,2);
plot_u(stored_t_2,0:tau:final_time,100);