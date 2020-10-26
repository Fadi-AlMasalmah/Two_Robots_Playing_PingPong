function [lambda] = Fk_lambda(q,T07)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
if nargin == 1
    T07=calc_T07(q);
end
q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);
q5 = q(5);
q6 = q(6);
q7 = q(7);
d1=0.340;
% T67 = diag(ones(4,1));
% d7r = 0.12+0.126;
% T67(2,4) = -d7r;

T67 = calc_T67(1);
T06 = T07/T67;


w = double(T06(1:3,4));

phi = atan2(w(2),w(1));

s=[0;0;d1];
sw=w-s;

Rsw=Rz(phi)*Ry(-atan2(sw(3),sw(1)));

ys = Rsw(1:3,2);

n1 = [ sin((q4))*(cos((q3))*sin((q1)) + cos((q1))*cos((q2))*sin((q3)));
       -sin((q4))*(cos((q1))*cos((q3)) - cos((q2))*sin((q1))*sin((q3)));
                                                     -sin((q2))*sin((q3))*sin((q4))];
                                                 
n1 = n1/(double(norm(n1)));
%%
prod = ys'*(n1);
if (prod > 1) & (prod < 1.001)
    prod = 1;
end
if (prod < -1) & (prod > -1.001)
    prod = -1;
end
lambda = -sign(q3)*acos(prod);



end

