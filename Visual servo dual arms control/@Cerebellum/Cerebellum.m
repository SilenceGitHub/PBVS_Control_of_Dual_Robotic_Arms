%% Cerebellum class
classdef Cerebellum
    %% Variables
    properties(SetAccess = private)
        reservoirSize = 0;
        learningRate = 0;
        leakyRate = 0;
        randSeed = 0;
        LINParameter = 0;
        Win = [];
        cell = [];
        W = [];
        Wout = [];
    end
    
    %% Constructor 

    methods
        % Initialize a cerebellum model
        function obj = Cerebellum(reservoirSize, learningRate, LINParameter,... 
            leakyRate,randSeed,inputDimension,outputDimension)
            obj.reservoirSize = reservoirSize;
            obj.learningRate = learningRate;
            obj.leakyRate = leakyRate;
            obj.randSeed = randSeed;
            obj.LINParameter = LINParameter;
            rand('seed',obj.randSeed);
            obj.cell = zeros(obj.reservoirSize,1);
            obj.Win = (rand(obj.reservoirSize,inputDimension)-0.5)*2;
            obj.W = (rand(obj.reservoirSize)-0.5)*2;
            opt.disp = 0;
            rhoW = abs(eigs(obj.W,1,'largestabs',opt));
            alpha = 0.5 / rhoW;
            % alpha = 0.8 / max(svd(W));
            obj.W =  alpha * obj.W;% Scale W
            
            opt.disp = 0;
            rhoW = abs(eigs(obj.W,1,'largestabs',opt));

            obj.Wout = zeros(outputDimension,reservoirSize);
            

        end
    end
    
   %% functions
    methods (Static)
        % get the output
        [ret, output] = Forward(obj,input);
        % inner activation function of reservoir
        output = sigmoid(input);
        % update the output weight
        obj = Backward(obj,error);
    end

    
end
        