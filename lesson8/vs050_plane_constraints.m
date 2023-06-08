clear all;
close all;
include_namespace_dq;
% Build a 5cmx5cmx5cm cubic region delineated by 6 planes.
n_pi1 = i_;
n_pi2 = -1 * i_;
n_pi3 = j_;
n_pi4 = -1 * j_;
n_pi5 = k_;
n_pi6 = -1 * k_;
% Build the plane
p_pi1 = (0.45 + 2.5 * 10^(-2)) * i_ + 0.08 * k_;
p_pi2 = (0.45 -2.5* 10^(-2)) * i_+ 0.08 * k_;
p_pi3 = 0.45*i_ +2.5* 10^(-2) * j_+ 0.08 * k_;
p_pi4 = 0.45*i_ -2.5* 10^(-2) * j_+ 0.08 * k_;
p_pi5 = 0.45*i_ + (0.08 +2.5* 10^(-2)) * k_;
p_pi6 = 0.45*i_ + (0.08 -2.5* 10^(-2)) * k_;
d_pi1 = dot(p_pi1,n_pi2);
d_pi2 = dot(p_pi2,n_pi1);
d_pi3 = dot(p_pi3,n_pi4);
d_pi4 = dot(p_pi4,n_pi3);
d_pi5 = dot(p_pi5,n_pi6);
d_pi6 = dot(p_pi6,n_pi5);

pi1 = n_pi2 + E_*d_pi1;
pi2 = n_pi1 + E_*d_pi2;
pi3 = n_pi4 + E_*d_pi3;
pi4 = n_pi3 + E_*d_pi4;
pi5 = n_pi6 + E_*d_pi5;
pi6 = n_pi5 + E_*d_pi6;


% Define a list of planes
plane_list = {pi1, pi2, pi3, pi4, pi5, pi6};

% Define the robot
vs050_robot = VS050RobotDH.kinematics();

% Solver definition
qp_solver = DQ_QuadprogSolver();

% Define the controller
translation_controller = DQ_ClassicQPController(vs050_robot, qp_solver);
translation_controller.set_control_objective(ControlObjective.Translation)
translation_controller.set_gain(10)
translation_controller.set_damping(1)

% VFI gain
eta_d = 1;

% Desired translation (pure quaternion)
td_list = {0.60*i_ + 0.08*k_, 0.20*i_+0.15*k_, 0.45*i_+0.08*k_, 0.45*i_};
% Sampling time 
tau = 0.01; 
% Simulation time 
final_time = 4;
% Initial joint values 
q = [0 pi/3 pi/3 0 pi/3 0];

% % Store the control signals
% stored_u_1 = [];
% stored_t_1 = [];
% stored_u_2 = [];
% stored_t_2 = [];
% t_error = DQ(1);

for td_counter = 1:length(td_list)    
    td = td_list{td_counter};
    % Initial joint values 
    q = [0 pi/3 pi/3 0 pi/3 0];
    f = figure(1);
    % Translation controller loop.
    for time=0:tau:final_time
        
        % The inequality matrix and vector
        W = [];
        w = [];
    %     n = vs050_robot.get_dim_configuration_space;
    %     for joint_counter=1:n
    %         
        % Get the pose Jacobian and pose of the current joint
        Jx = vs050_robot.pose_jacobian(q);
        x = vs050_robot.fkm(q);
        
        % Get the translation Jacobian and the translation of the current
        % joint
        Jt = DQ_Kinematics.translation_jacobian(Jx, x);
        t = translation(x);
            
        for plane_counter=1:length(plane_list)
            
            % Get the current plane
            workspace_plane = plane_list{plane_counter};
            
            % Calculate the point to plane distance Jacobian
            % We have to augment the Jp_pi with zeros
            Jp_pi = DQ_Kinematics.point_to_plane_distance_jacobian(Jt, t, workspace_plane);
            
            % Calculate the point to plane distance
            dp_pi = DQ_Geometry.point_to_plane_distance(t, workspace_plane);
            
            % Wq <= w
            W = [W; -Jp_pi];
            w = [w; eta_d*dp_pi];
        end
        
    
        % Update the linear inequalities in the controller
        translation_controller.set_inequality_constraint(W, w);
        
        % Get the next control signal [rad/s]
        u = translation_controller.compute_setpoint_control_signal(q,vec4(td));
        
        % Move the robot
        q = q + u*tau;
        
       
        % Plot the robot
%         plot(vs050_robot,q)
%         title(['Translation control' ' time = ' num2str(time) 's out of ' num2str(final_time) 's'])
%         hold on
        % Plot the desired pose
        hold on
        plot3(td.q(2),td.q(3),td.q(4), 'ko');
        % Plot the shaft in blue
        t_1 = translation(vs050_robot.raw_fkm(q));
        t_2 = translation(vs050_robot.fkm(q));
        plot3([t_1.q(2) t_2.q(2)],[t_1.q(3) t_2.q(3)],[t_1.q(4) t_2.q(4)],'b','LineWidth',2)
         % Plot the walls
        plot(pi1, 'plane', 0.05, 'location', p_pi1);
        plot(pi2, 'plane', 0.05, 'location', p_pi2);
        plot(pi3, 'plane', 0.05, 'location', p_pi3);
        plot(pi4, 'plane', 0.05, 'location', p_pi4);
        plot(pi5, 'plane', 0.05, 'location', p_pi5);
        plot(pi6, 'plane', 0.05, 'location', p_pi6);
        hold off
        % [For animations only]
        drawnow limitrate % [For animations only] Ask MATLAB to draw the plot now
    end
    clf(f)
end