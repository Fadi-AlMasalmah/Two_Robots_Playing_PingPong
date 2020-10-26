%% Calculating Jacobian
%T07 = simplify(A(:,:,7),'Steps',25);
%% calulating lambda
y2 = A(1:3,2,2)
y4 = A(1:3,2,4)
assume(q,'real')
n1 = cross(y4,y2)
n1 = simplify(n1)
%%
T07 = A(:,:,7);
T67 = calc_T67(1);
T06 = T07/T67;
%  T06 = Td07/T(:,:,7);
 w = (T06(1:3,4));
% double(w)%
 %w = Td07(1:3,4)
 phi = atan2(w(2),w(1));
% w_= Rz(-phi)*w;
% double(w_) %
s=[0;0;d1];
sw=w-s;
% double(sw_)
dsw=double(norm(sw));

% calculating Rzw_= Rz(psi)*Rx(theta)
%syms psi
%syms theta
%Rzw_=Rz(psi)*Rx(theta);
%psi1 = atan2(w_(2),w_(1));
% ex = sw_/dsw_;
Rsw=Rz(phi)*Ry(-atan2(sw(3),sw(1)));
% S = [[Rsw;[0,0,0]],[s;1]];
ys = Rsw(1:3,2)
%%
lambda = acos(ys'*(n1)) 
J_lambda = jacobian(lambda,q(1:7));
J_lambda = sign(q3)*J_lambda;
%%
T07 = A(:,:,7);
Jv = jacobian((T07(1:3,4)),q(1:7));
syms Jw [3,7];
%Jw = zeros(3,7);
for i=1:7
    Jw(:,i) = A(1:3,3,i);
end

% Jacobian
J = [Jv;Jw];
J = simplify(J);
% J_ = diff()

% Jt = (J*J')\J;
% Jt1 = simplify((J'*J)\J');
%%
J77 = [J;J_lambda]
%%
diff(J_lambda,t)
%% Trying to avoid singularities
% deleting the 7th joint
% d_J = det(J(:,1:6));
% d_J = simplify(d_J)
d_J7 = -(8*cos(q5)*sin(q2)*(cos(q4)^2 - 1))/125;
%delet 3rd joint
% d_J3 = det(J(:,[1,2,4:7]));
% d_J3 = simplify(d_J3)
d_J3 = -(8*sin(q6)*(cos(q4) + 1)*(sin(q2)*sin(q4) - cos(q2)*cos(q3) + cos(q2)*cos(q3)*cos(q4)))/125;
%%
syms c7;
syms c3;
% c7 = 1; c3 = 1;
PHI = c3*d_J3 + c7*d_J7;

J_PHI = jacobian(PHI,q(1:7));
J_PHI = simplify(J_PHI)

PHI_abs = abs(PHI);
J_PHI_abs = jacobian(PHI_abs,q(1:7));
%% Adj(Jm)
% djm = det(J(:,1:6));
% djm = simplify(dj);
det_j_6 = -(8*cos(q5)*sin(q2)*(cos(q4)^2 - 1))/125;
adj_J_6 = adjoint(J(:,1:6))

Z = [J(:,7)'*-adj_J_6,det_j_6];
for i=1:7
   i=6
    tic
    Z(i) = simplify(Z(i))
   toc
end