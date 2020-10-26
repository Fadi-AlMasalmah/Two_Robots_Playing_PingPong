function [Rot] = Rot(R)
%Inverse Kinematics
%   Detailed explanation goes here
Rot = zeros(4,4);
Rot(4,4)=1;
Rot(1:3,1:3) = R;

end
