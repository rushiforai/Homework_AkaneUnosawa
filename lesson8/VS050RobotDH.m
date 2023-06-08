classdef VS050RobotDH
    %VS050Robot regarding all methods related to the 3-DoF planar robot
    methods (Static)
        function ret = kinematics()
            %kinematics returns the kinematics of the VS050 Robot as DQ_SerialManipulatorDH
            DH_theta=  [- pi, pi/2, - pi/2, 0, pi, -pi/2];
            DH_d =     [0.345, 0, 0, 0.255, 0, 0.07];
            DH_a =     [0, 0.255, 0.01, 0, 0, 0];
            DH_alpha = [pi/2, 0, -pi/2, pi/2, pi/2, 0];
            DH_type = repmat(DQ_SerialManipulatorDH.JOINT_ROTATIONAL,1,6);
            DH_matrix = [DH_theta;
                DH_d;
                DH_a;
                DH_alpha;
                DH_type];
            
            ret = DQ_SerialManipulatorDH(DH_matrix,'standard');
            ret.name = "VS050 Robot";
        end
    end
end