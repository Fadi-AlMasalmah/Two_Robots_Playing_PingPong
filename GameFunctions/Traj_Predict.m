function [fitz,P] = Traj_Predict(t,Positions,N)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
%P0 is the starting point
% [fitx,gof_x,fitinfo_x] = fit(t,x,ftx)
% dbstop if error
x = Positions(:,1);
y = Positions(:,2);
z = Positions(:,3);
% x0 = sum(x)/N;
% x0 = sum(x)/N;
% x0 = sum(x)/N;
% x0 = sum(x)/N;
% x0 = sum(x)/N;
% x0 = sum(x)/N;
% P = sum(x)/N;

ftx = fittype('a*x+b','coefficients', {'a','b'});
[fitx] = fit(t,x,ftx,'StartPoint',[1 1]);

fty = fittype('a*x+b','coefficients', {'a','b'});
[fity] = fit(t,y,fty,'StartPoint',[t(1) Positions(1,2)]);

ftz = fittype('a*x^2+b*x+c','coefficients', {'a','b','c'});
[fitz] = fit(t,z,ftz,'StartPoint',[1 1 1]);
% residuals = fitinfo.residuals;
% figure 
% plot(fit1,x,y) 
% hold on
t_now = t(N);
t = (2 - t_now:0.01:2);

P(1,:) = fitx.a*t + fitx.b;

P(2,:) = fity.a*t + fity.b;

P(3,:) = fitz.a*t.^2 + fitz.b*t + fitz.c;
end
