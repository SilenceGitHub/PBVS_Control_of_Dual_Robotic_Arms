%% Jaco2 class
classdef Jaco2
    %% Variables
    properties(SetAccess = public)
        theta = [];
        dotTheta = [];
        JHat = [];
        dotJHat = [];
        actualPath = [];
        actualVelocity = [];
        desiredPath = [];
        desiredVelocity = [];
        positionFilter  = [];
        errors = [];
    end
    properties(SetAccess = private)
        joint_handle = [];
        jointNumber = 6;
        taskSpaceDimension = 7;
    end
    
    %% Constructor 

    methods
        function obj = Jaco2(numberOfSampling, vrepInstance)
            obj.theta = zeros(obj.jointNumber,numberOfSampling);  
            obj.dotTheta =  zeros(obj.jointNumber,numberOfSampling);  
            obj.JHat = zeros(obj.taskSpaceDimension,obj.jointNumber,numberOfSampling);
            obj.dotJHat = zeros(obj.taskSpaceDimension,obj.jointNumber,numberOfSampling);
            obj.desiredPath = zeros(obj.taskSpaceDimension,numberOfSampling);
            obj.desiredVelocity = zeros(obj.taskSpaceDimension,numberOfSampling);
            obj.actualPath = zeros(obj.taskSpaceDimension,numberOfSampling);
            obj.actualVelocity = zeros(obj.taskSpaceDimension,numberOfSampling);
            obj.positionFilter = zeros(obj.taskSpaceDimension,numberOfSampling);
            obj.errors = zeros(obj.taskSpaceDimension,numberOfSampling);
            
            for i = 1:obj.jointNumber
                [~, obj.joint_handle(i)] = vrepInstance.vrep.simxGetObjectHandle(vrepInstance.clientID, ['Jaco_joint',num2str(i)], vrepInstance.vrep.simx_opmode_blocking);
            end
       
        end
    end
    
   %% functions
    methods (Static)
        % set joint angle
        ret = SetJointAnglePosition(obj, vrepInstance, input, opmode);
        
        % set joint velocity
        ret = SetJointVel(obj, vrepInstance, input, opmode);
        
        % get joint angle
        ret = GetJointAnglePosition(obj, vrepInstance, opmode);
        
        % initialize the Jacobian matrix
        ret = InitializeJacobian(obj, initialJointAngle, vrepInstance);
       
    end
end
        