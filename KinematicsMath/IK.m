function [q,solvable] = IK(T,A,Td07,lambda,config)
%Inverse Kinematics
%   Detailed explanation goes here
% solvable = true;
d1=0.340;
d3 = 0.4;
d5 = 0.4;
d7r = 0.12+0.12;
arm = config(1);
elbow = config(2);
wrist = config(3);

 T06 = Td07/T(:,:,7);
 w = T06(1:3,4)
% double(w)%
%w = Td04(1:3,4);
phi = atan2(w(2),w(1));
w_= Rz(-phi)*w;
% double(w_) %
s=[0;0;d1];
sw_=w_-s;
% double(sw_)
dsw_=double(norm(sw_));

% calculating Rzw_= Rz(psi)*Rx(theta)
%syms psi
%syms theta
%Rzw_=Rz(psi)*Rx(theta);
%psi1 = atan2(w_(2),w_(1));
% ex = sw_/dsw_;
Rsw_=Ry(-atan2(sw_(3),sw_(1)));
S = [[Rsw_;[0,0,0]],[s;1]];
alpha = elbow*acos((d5^2 - d3^2 - dsw_^2)/(-2*d3*dsw_)); %acos? how many solutions  %% I added Elbow%%%%

if(~isreal(alpha))
    solvable = false;
end

E0_=S * Translation([d3*cos(alpha);0;d3*sin(alpha)]);
e0_= E0_(1:3,4);
% double(e0_)

E_lambda = S*Rot(Rx(lambda))/Rot(Rsw_)*Translation(e0_-s); %you might have to write Rx(lambda) instead
e_lambda = E_lambda(1:3,4);
% double(e_lambda)

E_lambda_phi = Rot(Rz(phi))*E_lambda;
e_lambda_phi = E_lambda_phi(1:3,4);
% double(e_lambda_phi)

q1 = atan2(arm*e_lambda_phi(2),arm*e_lambda_phi(1)); %%% q1
% double(rad2deg(q1)) %

q2 = atan2( arm  *  norm([e_lambda(1) , e_lambda(2)])  ,  (e_lambda(3)-s(3))  ); %%q2 I added elbow (I think)

% if(elbow==-1)  
%     q2 = q2 + 2*arm*alpha;
% end
q4 = arm*elbow*(  pi - acos(  (dsw_^2 - d5^2 - d3^2) / (-2*d3*d5) )); %%q4

if(~isreal(q4))
    solvable = false;
end
% double(rad2deg(q4)) %
%%
% T01 = subs(T(:,:,1))
% T12 = subs(T(:,:,2))
% T34 = subs(T(:,:,4))
% M14 = T01*T12*T(:,:,3)*T34
% T16 = T01\A(:,:,6)

%% calculating q3

 w_temp = Rz(-q1)*w;
 q3 = 0;
if (sin(q4)==0)
    q3 = 0; %% q3 affects only the rotation of w_ , and it is parallel to q5 
else
    s3 = -w_temp(2)/(sin(q4)*d5);  % what if s4 = 0 
    if (cos(q2)~=0)
        c3= ( -w_temp(1) + 0.4000*sin(q2) + 0.4000*cos(q4)*sin(q2))/(0.4000*cos(q2)*sin(q4));
    else
        c3 = (w_temp(3) - (0.4000*cos(q2) + 0.4000*cos(q2)*cos(q4)))/(0.4000*sin(q2)*sin(q4));
    end
    %c3 = sqrt(1 - s3^2);
    q3 = atan2(s3,c3);
    
       if(abs(s3)>1 | abs(c3)>1)
       solvable = false;
       end
end

% 
% E_pi2 = S*Rot(Rx(pi/2))/Rot(Rsw_)*Translation(e0_-s); %you might have to write Rx(lambda) instead
% e_pi2 = E_pi2(1:3,4);
% a = e_pi2(2);
% double(a)
% 
% b = e0_(1)-e_pi2(1);
% double(b)
% 
% ty = -a*sqrt(-b^2 + (0.5*w_(1))^2 ) / (-0.5*w_(1));
% 
% s3 = -w_temp(2)/(sin(q4)*d5);
% double(s3)
% c3 = sqrt(1 - s3^2);
% 
% 
% if(b==0)
%     q3 = atan2(s3 , elbow*sign(e_lambda(3)-s(3))*c3);
% elseif ((ty ~= real(ty))| (isnan(ty))| (ty==Inf) | abs(e_lambda(2)) > abs(ty) )| ( abs(e_lambda(1)) >  abs(-0.5*w_(1)))
%     q3 = atan2(s3 , elbow* sign(b)*c3);
% else
%      q3 = atan2(s3 , -elbow*c3);
% end
%      double(rad2deg(q3))
%% calculating q5, q6 , q7 
% I have to discuss the wrist
%  T04 = subs(A(:,:,4))
 %    T47 = inv(T04)*Td07


%gg = subs(A(:,:,4))
%      fdf = det(gg)
    
%    q5 = atan2(T47(1,3),-T47(3,3));
%    
%    if(cos(q5)~=0)
%        s6 = T47(1,3)/cos(q5);
%    else
%         s6 = -T47(3,3)/sin(q5);
%    end
%    q6 = atan2(s6,T47(2,3));
%    q7 = atan2(T47(2,2),-T47(2,1));
     q = [q1,q2,q3,q4,0,0,0];
end


