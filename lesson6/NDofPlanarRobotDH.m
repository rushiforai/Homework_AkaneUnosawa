classdef NDofPlanarRobotDH
    %NDofPlanarRobot regarding all methods related to the N-DoF planar robot
%     properties (Access=private)
%         % N value
%         N
%     end
    methods (Static)
%         function obj = NDofPlanarRobot(N)
%             %OneDofPlanarRobot creates a 2-DoF planar robot of length l
%             obj.N = N;
%         end
        function ret = kinematics(N)
            %kinematics returns the kinematics of the NDoFPlanarRobot as DQ_SerialManipulatorDH
            DH_theta=  zeros(1,N);
            DH_d =     zeros(1,N);
            DH_a =     ones(1,N);
            DH_alpha = zeros(1,N);
           
            DH_type = repmat(DQ_SerialManipulatorDH.JOINT_ROTATIONAL,1,N);
            DH_matrix = [DH_theta;
                DH_d;
                DH_a;
                DH_alpha;
                DH_type];
            
            ret = DQ_SerialManipulatorDH(DH_matrix,'standard');
            ret.name = "N DoF Planar Robot";
        end
    end
 end