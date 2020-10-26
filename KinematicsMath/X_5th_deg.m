function [X_,X] = X_5th_deg(t,Tf,X0,Xf,V0,Vf,Acc0,Accf)
%UNTITLED Summary of this function goes here
%   this Function calculate X as a 5th degree polynomial of time
tt = ones(7,1);
tt(:,:) = tt(:,:)*t(1,1);
a0 = X0;
a1 = V0;
a2 = Acc0/2;
a3 = (20*Xf-20*X0-(8*Vf+12*V0).*Tf - (3*Acc0 - Accf).*Tf.^2)./(2*Tf.^3);
a4 = (30*X0 - 30*Xf + (14*Vf + 16*V0).*Tf + (3*Acc0 - 2*Accf).*Tf.^2)./(2*Tf.^4);
a5 = (12*Xf-12*X0- (6*Vf+6*V0).*Tf - (Acc0 - Accf).*Tf.^2)./(2*Tf.^5)

X = a0 + a1.*tt + a2.*tt.^2 + a3.*tt.^3 + a4.*tt.^4 + a5.*tt.^5;
X_ = a1 +  2*a2.*tt + 3*a3.*tt.^2 + 4*a4.*tt.^3 + 5*a5.*tt.^4;

end

