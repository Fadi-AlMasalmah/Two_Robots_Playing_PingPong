function [T07] = FK(varargin)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
if nargin == 2
%     A = varargin{1};
    Q = varargin{2};
else
    Q = varargin{1};
end
T07 = calc_T07(Q);

% q1 = Q(1);
% q2 = Q(2);
% q3 = Q(3);
% q4 = Q(4);
% q5 = Q(5);
% q6 = Q(6);
% q7 = Q(7);

% T07 = subs(A(:,:,7));
% w=T07(1:3,4);
end

