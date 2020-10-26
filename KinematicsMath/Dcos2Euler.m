function [Euler] = Dcos2Euler(R)
%UNTITLED3 Summary of this function goes here
%   Euler angles are about the relative frames (Z'Y'X')

Beta = atan2(-R(3,1),sqrt(R(1,1)^2 + R(2,1)^2));
CB = cos(Beta);
if( CB ~= 0 )
    Alpha = atan2(R(2,1)/CB ,R(1,1)/CB);
    Gamma = atan2(R(3,2)/CB ,R(3,3)/CB);
else
    if(Beta > 0)
        Alpha = 0;
        Gamma = atan2(R(1,2),R(2,2));
    else
        Alpha = 0;
        Gamma = -atan2(R(1,2),R(2,2));
    end
end

Euler = [Alpha,Beta,Gamma];  %(Z',Y',X')
end

