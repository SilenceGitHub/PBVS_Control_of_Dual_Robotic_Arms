%% VREP class
classdef VREP
    %% Variables
    properties(SetAccess = private)
         vrep=0;
         clientID=0;
         cone_handle = [];
         target_handle = [];   
         vision_sensor_handle = [];
    end
    
    %% Costructor (open the comunication with VREP )
    % Start the simulation in vrep before calling the costructor!!!!!!!
    methods
        %It opens communication to a specific port
        function obj = VREP(port)
            obj.vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
            obj.vrep.simxFinish(-1); % just in case, close all opened connections
            obj.clientID=obj.vrep.simxStart('127.0.0.1',port ,true ,true,5000,5); %Open the comunication

            [~, obj.cone_handle] = obj.vrep.simxGetObjectHandle(obj.clientID, 'Jaco_connection' ,obj.vrep.simx_opmode_blocking);
            [~, obj.target_handle] = obj.vrep.simxGetObjectHandle(obj.clientID, 'Cylinder' ,obj.vrep.simx_opmode_blocking);
            [~, obj.vision_sensor_handle] = obj.vrep.simxGetObjectHandle(obj.clientID, 'Vision_sensor' ,obj.vrep.simx_opmode_blocking);
        end
    end
     
    %% Object functions
    methods(Static)
        
         %Function for control a VREP simulator for ball and plate task
         %obj is the class object
         %object_name is the name of the object in the VREP scene
         ret =  Get_object_pose(obj, object_hendle);
         ret =  Get_object_pose_reference(obj, object_handle, reference_handle, opmode);
    end
    
    %% Useful functions
    methods(Static)
         Disconnect(obj);
    end
end
        