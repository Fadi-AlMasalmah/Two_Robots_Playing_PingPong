function [Trans] = Translation(s)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
Trans=[[eye(3);[0,0,0]],[s(1);s(2);s(3);1]];
end

