classdef TwoDofPlanarRobot
    %TwoDofPlanarRobot regarding all methods related to the 2-DoF planar robot
    
    properties (Access=private)
        % The length of the robot, in meters
        l1
        l2
    end
    
    methods
        function obj = TwoDofPlanarRobot(l1,l2)
            %OneDofPlanarRobot creates a 2-DoF planar robot of length l
            obj.l1 = l1;
            obj.l2 = l2;
        end
        
        function [t_w_r1, t_w_r] = fkm1(obj,theta1, theta2)
            %fkm calculates the FKM for the 2-DoF planar robot.
            
            % Include the namespace inside the function
            include_namespace_dq
            
            % The rotation about the joint
            x_w_11 = cos(theta1/2.0) + k_*sin(theta1/2.0);
            % The translation about the length
            x_1_r1 = 1 + 0.5*E_*i_*obj.l1;
            % Pose transformation
            x_w_r1 = x_w_11*x_1_r1;
            t_w_r1 = translation(x_w_r1);

            % The rotation about the joint
            x_w_12 = cos(theta2/2.0) + k_*sin(theta2/2.0);
            % The translation about the length
            x_1_r2 = 1 + 0.5*E_*i_*(obj.l2);
            % Pose transformation
            x_w_r2 = x_w_r1 * x_w_12*x_1_r2;
            
            % Get the translation
            t_w_r = translation(x_w_r2);
        end

        
        
        function theta1 = ikm_tx1(obj,tx1)
            %fkm calculates the IKM for the 1-DoF planar robot using the 
            % desired x-axis translation.
            
            % Return the angle to reach the desired tx
            theta1 = acos(tx1/obj.l1);
        end
        
        function theta1 = ikm_ty1(obj,ty1)
            %fkm calculates the IKM for the 1-DoF planar robot using the 
            % desired y-axis translation.
            
            % Return the angle to reach the desired ty
            theta1 = asin(ty1/obj.l1);
        end

        function theta2 = ikm_tx2(obj,tx2)
            %fkm calculates the IKM for the 1-DoF planar robot using the 
            % desired x-axis translation.
            
            % Return the angle to reach the desired tx
            theta2 = acos(tx2/obj.l2);
        end
        
        function theta2 = ikm_ty2(obj,ty2)
            %fkm calculates the IKM for the 1-DoF planar robot using the 
            % desired y-axis translation.
            
            % Return the angle to reach the desired ty
            theta2 = asin(ty2/obj.l2);
        end
        
        function Jt = translation_jacobian(obj,theta1, theta2)
            % Calculates the translation Jacobian of the 1-DoF planar
            % robot.
            
            % Include the namespace inside the function
            include_namespace_dq
            
            j1 = i_ *(-obj.l1*sin(theta1)-obj.l2*sin(theta1+theta2)) + j_*(-obj.l2*sin(theta1 + theta2));
            j2 = i_ *(obj.l1*cos(theta1)+obj.l2*cos(theta1+theta2)) + j_*(obj.l2*cos(theta1+theta2));
            Jt = [vec3(j1),vec3(j2)];

%             J1 = -1 * obj.l1 * sin(theta1) - obj.l2 * sin(theta1 + theta2);
%             J2 = -1 * obj.l2 * sin(theta1 + theta2);
%             J3 = obj.l1 * cos(theta1) + obj.l2 * cos(theta1 + theta2);
%             J4 = obj.l2 * cos(theta1 + theta2);
%             
%             Jt = [J1, J2;
%                   J3, J4];
        end
        
        function plot(obj,theta1,theta2)
            % Plot the 2-DoF planar robot in the xy-plane
            
            % Get the fkm
            [t_w_r1, t_w_r] = obj.fkm1(theta1, theta2);
            
            % Plot
            plot([0 t_w_r1.q(2)],[0 t_w_r1.q(3)],'g')
            hold on
            plot([t_w_r1.q(2) t_w_r.q(2)], [t_w_r1.q(3) t_w_r.q(3)],'r')
   
            hold on
            % Mark the base with an o
            plot(0,0,'o')
            % Mark the end effector with an x
            hold on
            plot(t_w_r1.q(2),t_w_r1.q(3),'gx')
            hold on
            plot(t_w_r.q(2),t_w_r.q(3),'rx')
            hold off
            title('The Two DoF planar robot in the xy-plane')
            xlim([-2 2])
            xlabel('x [m]')
            ylim([-2 2])
            ylabel('y [m]')
        end
    end
end