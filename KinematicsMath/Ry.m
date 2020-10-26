function [Ry] = Ry(phi)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
Ry = [[cos(phi);0;-sin(phi)],[0;1;0],[sin(phi);0;cos(phi)]];
end
