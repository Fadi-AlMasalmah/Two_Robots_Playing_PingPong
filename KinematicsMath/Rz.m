function [Rz] = Rz(phi)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
Rz = [[cos(phi);sin(phi);0],[-sin(phi);cos(phi);0],[0;0;1]];
end

