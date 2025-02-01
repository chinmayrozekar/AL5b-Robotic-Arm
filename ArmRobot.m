classdef ArmRobot < handle
    % ArmRobot is a class that contains all the functions that need to
    % control the Arm Robot.
    %
    %Authors:  Vu Nguyen        (vkn3505@rit.edu)
    %          Ryan M. Bowen    (rmb3518@rit.edu)
    %          Matt Haywood     (mxh7801@rit.edu)
    %Modified: 10/5/2011 (rmb3518)
    %Modified: 10/9/2016 (rmb3518)
    
    properties(Constant)
        %----- Arm Joints--------
        base            =   0;              % Base joint number
        shoulder        =   1;              % Shoulder joint number
        elbow           =   2;              % Elbow joint number
%        elbowTwist      =   3;              % Elbow twist joint number
        wrist           =   3;              % Wrist joint number
        wristTwist      =   4;              % Wrist twist joint number
        gripper         =   5;              % Gripper joint number
        
        allJoints       = [0 1 2 3 4 5];  % Array containing all joint numbers
        
        defaultCenter   = 1500;             % Default servo center value
        
    end
    
    properties(SetAccess = private, GetAccess = private)
        
        controller;                     % Communicator for SSC-32
        frame                           % Tool Frame w.r.t Universe
        
        linkLengths     = ones(1,7);    % Lengths of the links of the robot
        
        servoCenters    = ones(1,6);    % Center values for the servos
        servoMin        = ones(1,6);    % Min values for each servo
        servoMax        = ones(1,6);    % Max values for each servo
        
        servoValues     = ones(1,6);    % Current values of the servos
        
        connected       = 0;            % Connectivity flag

    end
    
    properties(SetAccess=public, GetAccess=public)
            
        % Universal frames
        UtA     = eye(4,4);
        UtB     = eye(4,4);
        UtC     = eye(4,4);
        UtD     = eye(4,4);
        UtE     = eye(4,4);
%         UtF     = eye(4,4);
        UtT     = eye(4,4);
        
        % Intermediate frames
        AtB     = eye(4,4);
        BtC     = eye(4,4);
        CtD     = eye(4,4);
        DtE     = eye(4,4);
%         EtF     = eye(4,4);
        EtT     = eye(4,4);
        
    end
    
    methods(Access=public)
        
        function obj = ArmRobot(portname)
            % ArmRobot Contructor
            %
            % @params  portName - port to communicate with robot ie 'COM1'
            
            % Create new SSC-32 object
            obj.controller  = SSC32(portname);
            
            % Set the cneters
            obj.setServoCenters(ArmRobot.defaultCenter*ones(1,length(ArmRobot.allJoints)));
            
            % Set some general bounds for the robot
            obj.setServoBounds([500 760 900 550 500 1100],...
                [2400 2100 1800 2300 2300 2300]);
            
            obj.setIntermediateFrame();
            obj.updateToolFrame();
            
        end
        
        function delete(obj)
            % ArmRobot destrctor, disconnect from the robot and destroy
            % objects.
            
            obj.disconnect();
            obj.controller.delete();
            clear obj;
            display('Arm Robot destroyed!');
        end
        
        function obj = setServoCenters(obj, centers)
            % Sets the servo center values
            %
            % @params centers - the center values for ALL servos.
            
            if (isequal(length(ArmRobot.allJoints),length(centers)))
                obj.servoCenters = centers;
            else
                error('Center Values must match the sizeof(Joints.allJoints)');
            end
        end
        
        function obj = setServoBounds(obj,min,max)
            % Sets the bounds for ALL servos.
            %
            % @params min - the min servo values.
            %         max - the max servo values.
            
            if( isequal(length(min),length(max)) && ...
                    (isequal(length(min),length(ArmRobot.allJoints))) )
                obj.servoMin = min;
                obj.servoMax = max;
            else
                error('Min and Max values must match the sizeof(Joints.allJoints)');
            end
        end
        
        function obj = setLinkLengths(obj,linkLengths)
            % Sets the lengths of ALL the links for the robot.
            %
            % @params linkLengths - link lengths for ALL the links.
            
            if (isequal(length(obj.linkLengths),length(linkLengths)))
                obj.linkLengths = linkLengths;
            else
                error('Link Lengths must match.');
            end
            
            obj.setIntermediateFrame();
            obj.updateToolFrame();
        end
        
        function obj = moveJoints(obj,values,joints)
            % Move servos based on the values and joint numbers specified,
            % throws an error is length(values) ~= length(joints) or if any
            % of the values are out of bounds.
            %
            % @params  value  - servo values (array or single value)
            %          joints - respective joint numbers for values.
            
            % Check lengths error if not the same
            if (isequal(length(values),length(joints)) )
                obj.servoValues = values;
            else
                error('"values" must match the sizeof("joints")');
            end
            
            % Out of bounds error flag
            outOfBounds = 0;
            
            % Store temp values of the servos
            tempServoValues = zeros(size(obj.servoValues));
            
            % Iterate through all the values
            for i = 1:length(values)
                
                % Check bounds break out if violation.
                if( (values(i) > obj.servoMax(joints(i)+1)) || ...
                        (values(i) < obj.servoMin(joints(i)+1) ))
                    outOfBounds = 1;
                    break;
                else
                    % In bound update new servo values.
                    tempServoValues(joints(i)+1) = values(i);
                end
            end
            
            % All values are in bounds
            if( ~outOfBounds )
                % Send values to SSC-32 object
                obj.controller.setServoValues(values,joints);
                % Save servo values state
                obj.servoValues = tempServoValues;
            else
                error('Servo value outside of bounds');
            end
            
        end
        
        function obj = centerJoints(obj)
            % Sets all the servos to their center positions.
            obj.moveJoints(obj.servoCenters,ArmRobot.allJoints);
        end
        
        function obj = connect(obj)
            % Connects to the robot by connecting to the SSC-32 controller
            display('Attemping to Connect to Robot...');
            obj.controller.connect();
            obj.connected = 1;
            display('Connected to Robot!');
        end
        
        function obj = disconnect(obj)
            % Disconnects from the robot by disconnecting from the SSC-32
            if( obj.connected )
                obj.controller.disconnect();
                obj.connected = 0;
                display('Robot disconnected!');
            end
        end
        
        function servoValues = getServoValues(obj)
            % Returns the current servos values
            servoValues = obj.servoValues
        end
        
        function obj = moveAbsolute(obj,degrees,joints)
            m = obj.deg2val(degrees,joints);
            obj.moveJoints(m,joints)
   
            
            
            % Moves the specified servo(s) to the desired degree(s) with
            % respect the servo(s) home position. 
            %
            % @params degrees - the desired degree(s) to set.
            %         joints  - respective joint number(s) for degree(s).
                        
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % FINISH IMPLEMENTING THIS FUNCTION            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        
        function obj = moveRelative(obj,degrees,joints)
            
             v = 0.*degrees;      %getServoValues(obj);
           % display(v);
            m = 0.*degrees;
             k = length(joints);
             a = obj.getServoValues();
            for i=1:k
                j = joints(i);
             switch j
                 case {0,'base'}
                     % base conversion
                     display('j+1 =')
                     display(j+1)
                     v(i) = a(j+1)-1300; %make j = j+1 if matlab throws error%
                     
                     display (v(i))
                 case  {1,'shoulder'}
                     %shoulder conversion
                     display(i);
                     display(j+1);
                     v(i) = a(j+1)-1500;
                     
                 case  {2,'elbow'}
                     %elbow conversion
                     v(i) = a(j+1)-1700;
                     
                 case  {3,'wrist'}
                     %wrist conversion
                     display(i);
                     display(j+1);
                     v(i) = a(j+1)-1600;
                     
                 case  {4,'wrist twist'}
                     %wrist twist conversion
                     v(i) = a(j+1)-1600;
                     
                 case  {5,'gripper'}
                     %gripper conversion
                     v(i) = a(j+1)-1400;
             end
            end
           disp('m')
             m = obj.deg2val(degrees,joints)
             disp('m + v')
             m = m + v
             
            obj.moveJoints(m,joints);
            obj.servoValues = a;
            
            for x = 1:length(m)
                j = joints(x);
                obj.servoValues(j+1) = m(x)
                display(obj.servoValues)
                %might need to change it to j+1
            end
            
            
            % Moves the specified servo(s) to the desired degree(s) with 
            % respect to the current position(s) of the servos.
            %
            % @params degrees - the desired relative degree(s) to set.
            %         joints  - respective joint number(s for degree(s).
            
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % FINISH IMPLEMENTING THIS FUNCTION            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        end
        
        function obj = moveRelativeLinear(obj,degrees,joints,nStep)
            % Moves the specified servo(s) to the desired degree(s) with
            % respect to the current positions(s) of the servos. This is
            % done in a linear fashion with number of steps specified.
            %
            % @params degrees - the desired relative degree(s) to set.
            %         joints  - respective joint number(s for degree(s).
            %         nStep   - the number of steps in linear motion. 
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % FINISH IMPLEMENTING THIS FUNCTION            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        
        function obj = moveAbsoluteLinear(obj,degrees,joints,nStep)
            % Moves the specified servo(s) to the desired degree(s) with
            % respect to the home positions(s) of the servos. This is
            % done in a linear fashion with number of steps specified.
            %
            % @params degrees - the desired relative degree(s) to set.
            %         joints  - respective joint number(s for degree(s).
            %         nStep   - the number of steps in linear motion.  
            
                 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % FINISH IMPLEMENTING THIS FUNCTION            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        end
        
        function setIntermediateFrame(obj) 
            % Sets the initial value of the intermediate frames.
   
            obj.UtA = FrameTransformation.trans('xz',[-obj.linkLengths(1) obj.linkLengths(2)]);
            obj.AtB = FrameTransformation.trans('z',obj.linkLengths(3))*FrameTransformation.rot('x',90);            
            obj.BtC = FrameTransformation.trans('y',obj.linkLengths(4));                        
%             obj.CtD = FrameTransformation.trans('x',obj.linkLengths(5))*FrameTransformation.rot('y',90);
            obj.CtD = FrameTransformation.trans('x',obj.linkLengths(5));  
            obj.DtE = FrameTransformation.trans('z',obj.linkLengths(6))*FrameTransformation.rot('y',90);            
%             obj.EtF = FrameTransformation.trans('x',obj.linkLengths(7))*FrameTransformation.rot('y',90);            
            obj.EtT = FrameTransformation.trans('z',obj.linkLengths(7));        
            
        end
        
        function updateToolFrame(obj)
            % Update the tool frame given that the intermediate ones have
            % already been updated. 
           
            obj.UtB = obj.UtA*obj.AtB;
            obj.UtC = obj.UtB*obj.BtC;
            obj.UtD = obj.UtC*obj.CtD;
            obj.UtE = obj.UtD*obj.DtE;            
%             obj.UtF = obj.UtE*obj.EtF;
%             obj.UtT = obj.UtF*obj.FtT;  
            obj.UtT = obj.UtE*obj.EtT;
        end
        
        function displayFrames(obj)
            % Display all frames
            display('UtA');obj.UtA
            display('AtB');obj.AtB
            display('BtC');obj.BtC
            display('CtD');obj.CtD
            display('DtE');obj.DtE
%             display('EtF');obj.EtF
%             display('FtT');obj.FtT
            display('EtT');obj.EtT
            
            display('UtA');obj.UtA
            display('UtB');obj.UtB
            display('UtC');obj.UtC
            display('UtD');obj.UtD
            display('UtE');obj.UtE            
            display('UtT');obj.UtT            
        end       
        
    end
    
    methods(Static)
        
        function value = deg2val(degree,joints)
            
            value = 0.*degree;
            
             k = length(joints);
            for i=1:k
                j = joints(i);
             switch j
                 case {0,'base'}
                     % base conversion
                     value(i) = 9.4*degree(i) + 1300;
                     
                 case  {1,'shoulder'}
                     %shoulder conversion
                     value(i) = 3.8*degree(i) + 1501;
                     
                 case  {2,'elbow'}
                     %elbow conversion
                     value(i) = -8.8*degree(i) + 1700;
                     
                 case  {3,'wrist'}
                     %wrist conversion
                     value(i) = 8.3*degree(i) + 1600;
                     
                 case  {4,'wrist twist'}
                     %wrist twist conversion
                     value(i) = 8.9*degree(i) + 1600;
                     
                 case  {5,'gripper'}
                     %gripper conversion
                     value(i) = -14.44*degree(i) + 1400;
             end
            end
           end
            
              
            % Converts degree(s) values into servo value(s) for the specified
            % joints.
            %
            % @params degree - degree(s) to be converted
            %         joints - respectful joint numbers
            
            
         
            
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % FINISH IMPLEMENTING THIS FUNCTION            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            
        end
        methods(Static)
        function degree = val2deg(val,joints)
            
            % Converts servo values(s) values into degree(s)for the specified
            % joints.
            %
            % @params val - servo value(s) to be converted
            %         joints - respectful joint numbers
                           
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % FINISH IMPLEMENTING THIS FUNCTION      
            for k=1:5
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            k = length(joints);
       
            
            for i = 1:k
                switch joints(i);
                    case {0, 'base'}
                        %base conversion
                        degree(i) = (1/9.44)*value(i) - 1300;
                        
                    case {1,'shoulder'}
                        %shoulder conversion
                        degree(i) = (1/3.88)*value(i) - 1500;
                        
                    case {2,'elbow'} 
                        %elblow conversion
                        degree(i) = (1/-8.8)*value(i) - 1700;
                        
                    case {3, 'wrist'}
                        %wrist conversion
                        degree(i) = (1/8.33)*value(i) - 1600;
                        
                    case {4, 'wrist twist'}
                         %wrist twist conversion
                         degree(i) = (1/8.8)*value(i) - 1600;
                         
                    case {5, 'gripper'}
                        %gripper conversion
                        degree(i) = (1/-14.4)*value(i) - 1400;
                end
            end
            end
       end
        end
end

