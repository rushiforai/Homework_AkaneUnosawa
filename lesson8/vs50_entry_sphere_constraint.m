% Let us isolate the code of this section from the other sections
clear all;
close all;
include_namespace_dq;

p = 0.45*i_ + 0.2*k_;
d_safe = 0.005;

% Define the initial joint configurations
q = [0, pi/3, pi/3, 0, pi/3, 0];

% Define the robot
vs050_robot = VS050RobotDH.kinematics();

% Add the shaft
vs050_robot.set_effector(1 + 0.5*E_*(0.2*k_));

% Desired translation
td_list = {0.45*i_ + 0.03*k_, 0.48*i_ + 0.03*k_, 0.41*i_ +0.03*k_, 0.4*i_ + 0.01*k_};


% Solver definition
qp_solver = DQ_QuadprogSolver();

% Controller definition
translation_controller = DQ_ClassicQPController(vs050_robot, qp_solver);
translation_controller.set_control_objective(ControlObjective.Translation);
translation_controller.set_gain(10);
translation_controller.set_damping(1);

% VFI gain
eta_d = 1;

% Sampling time 
tau = 0.01;
% Simulation time 
final_time = 4;
store_dis = [];

for td_counter = 1:length(td_list)
    td = td_list{td_counter};
    % Create a new figure
    f = figure(1);
    % Translation controller loop.
    
    p = 0.45*i_ + 0.2*k_;
    d_safe = 0.005;
    i = 0;
    
    % Define the initial joint configurations 
    q = [0, pi/3, pi/3, 0, pi/3, 0];
    for time=0:tau:final_time
        i = i + 1;
        % Get the pose Jacobian and the pose
        Jx = vs050_robot.pose_jacobian(q);
        x = vs050_robot.fkm(q);
        
        % Get the line Jacobian for the x-axis
        Jl = DQ_Kinematics.line_jacobian(Jx, x, i_);
        
        % Get the line with respect to the base
        t = translation(x);
        r = rotation(x);
        l = Ad(r, i_);
        l_dq = l + E_*cross(t, l);
        
        % Get the line-to-point distance Jacobian
        Jl_p = DQ_Kinematics.line_to_point_distance_jacobian(Jl, l_dq, p);
        
        % Get the line-to-point square distance
        Dl_p = DQ_Geometry.point_to_line_squared_distance(p, l_dq);
        store_dis=[store_dis,Dl_p];
        
        % Get the distance error
        D_safe = d_safe^2;
        D_tilde = D_safe - Dl_p;
        
        % The inequality matrix and vector
        W = Jl_p;
        w = eta_d*D_tilde;
        
        % Update the linear inequalities in the controller
        translation_controller.set_inequality_constraint(W, w);
        
        % Get the next control signal 
        u = translation_controller.compute_setpoint_control_signal(q,vec4(td));
        
        % Move the robot
        q = q + u*tau;
        
        % Clear plot
    %     plot(vs050_robot, q)
    %     clf(f)
    %     % Plot the robot
    %     plot(vs050_robot, q)
    %     title(['Translation control' ' time = ' num2str(time) 's out of ' num2str(final_time) 's'])
    %     hold on
        % Plot the desired pose
%         hold on
%         plot3(td.q(2),td.q(3),td.q(4), 'ko');
%         % Plot the shaft in blue
%         t_1 = translation(vs050_robot.raw_fkm(q));
%         t_2 = translation(vs050_robot.fkm(q));
%         plot3([t_1.q(2) t_2.q(2)],[t_1.q(3) t_2.q(3)],[t_1.q(4) t_2.q(4)],'b','LineWidth',2)
%         % Plot the entry sphere
        scatter(i, Dl_p);hold on
%         plot3(p.q(2), p.q(3), p.q(4), 'ro', 'MarkerSize', 20)
        % [For animations only]
        drawnow limitrate % [For animations only] Ask MATLAB to draw the plot now
    end
end