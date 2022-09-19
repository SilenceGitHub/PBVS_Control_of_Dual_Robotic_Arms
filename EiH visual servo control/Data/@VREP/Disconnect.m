function [] = Disconnect(obj)
    % Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    obj.vrep.simxGetPingTime(obj.clientID);

    obj.vrep.simxStopSimulation(obj.clientID,obj.vrep.simx_opmode_oneshot_wait);
    % Now close the connection to V-REP:
    obj.vrep.simxFinish(obj.clientID);
end