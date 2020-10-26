function [q__,q_,q]= Q_3rd_deg(t,T,q0,qf,q0_,qf_)

% if t<1
%     theta(2)=t;
% else
%     theta(2)=1;
% end
% if (t>1)
% theta(1) = t-1;
% end
% if t>2
%     theta(1) = 1;
% end
tt = t(1,1)*ones(7,1);

% q = zeros(7,1);
% q_ = zeros(7,1);
% q__ = zeros(7,1);

a0 = q0;
a1 = q0_;
a2 = 3*(qf-q0)./T.^2 - 2*q0_./T - qf_./T;
a3 = -2*(qf-q0)./T.^3 + (qf_ + q0_)./T.^2;
q = a0 + a1.*tt + a2.*tt.^2 + a3.*tt.^3;
q_ = a1 +  2*a2.*tt + 3*a3.*tt.^2;
q__ =       2*a2 + 6*a3.*tt;
end

