function [Rx] = Rx(phi)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
Rx = [[1;0;0],[0;cos(phi);sin(phi)],[0;-sin(phi);cos(phi)]];
end
