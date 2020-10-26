classdef RobotQTraj
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
            Qi  
            Qf    
            Qdi   
            Qdf  
            Qddi 
            Qddf  
            Ti    
            Cs
    end
    
    methods
        function obj = RobotQTraj(initialQ)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.Qi    = initialQ; 
            obj.Qf    = initialQ; 
            obj.Qdi   = zeros(7,1); 
            obj.Qdf   = zeros(7,1); 
            obj.Qddi  = zeros(7,1); 
            obj.Qddf  = zeros(7,1); 
            obj.Ti    = 0;
            obj.Cs    = zeros(7,6);
        end
        
    end
end

