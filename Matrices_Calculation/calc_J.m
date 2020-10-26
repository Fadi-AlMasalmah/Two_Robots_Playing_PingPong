function J = calc_J(q1,q2,q3,q4,q5,q6)
%CALC_J
%    J = CALC_J(q1(t),q2(t),q3(t),q4(t),q5(t),q6(t))

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    21-Mar-2020 00:05:59

J = reshape([sin(q1(t)(t)).*sin(q2(t)).*(-2.0./5.0)+cos(q6(t)).*(sin(q4(t)).*(cos(q1(t)).*sin(q3(t))+cos(q2(t)).*cos(q3(t)).*sin(q1(t)))-cos(q4(t)).*sin(q1(t)).*sin(q2(t))).*(1.23e+2./5.0e+2)-sin(q6(t)).*(cos(q5(t)).*(cos(q4(t)).*(cos(q1(t)).*sin(q3(t))+cos(q2(t)).*cos(q3(t)).*sin(q1(t)))+sin(q1(t)).*sin(q2(t)).*sin(q4(t)))+sin(q5(t)).*(cos(q1(t)).*cos(q3(t))-cos(q2(t)).*sin(q1(t)).*sin(q3(t)))).*(1.23e+2./5.0e+2)+sin(q4(t)).*(cos(q1(t)).*sin(q3(t))+cos(q2(t)).*cos(q3(t)).*sin(q1(t))).*(2.0./5.0)-cos(q4(t)).*sin(q1(t)).*sin(q2(t)).*(2.0./5.0),cos(q1(t)).*sin(q2(t)).*(2.0./5.0)+cos(q6(t)).*(sin(q4(t)).*(sin(q1(t)).*sin(q3(t))-cos(q1(t)).*cos(q2(t)).*cos(q3(t)))+cos(q1(t)).*cos(q4(t)).*sin(q2(t))).*(1.23e+2./5.0e+2)-sin(q6(t)).*(cos(q5(t)).*(cos(q4(t)).*(sin(q1(t)).*sin(q3(t))-cos(q1(t)).*cos(q2(t)).*cos(q3(t)))-cos(q1(t)).*sin(q2(t)).*sin(q4(t)))+sin(q5(t)).*(cos(q3(t)).*sin(q1(t))+cos(q1(t)).*cos(q2(t)).*sin(q3(t)))).*(1.23e+2./5.0e+2)+sin(q4(t)).*(sin(q1(t)).*sin(q3(t))-cos(q1(t)).*cos(q2(t)).*cos(q3(t))).*(2.0./5.0)+cos(q1(t)).*cos(q4(t)).*sin(q2(t)).*(2.0./5.0),0.0,0.0,0.0,1.0,(cos(q1(t)).*(cos(q2(t)).*2.0e+2+cos(q2(t)).*cos(q4(t)).*2.0e+2+cos(q2(t)).*cos(q4(t)).*cos(q6(t)).*1.23e+2+cos(q3(t)).*sin(q2(t)).*sin(q4(t)).*2.0e+2+cos(q3(t)).*cos(q6(t)).*sin(q2(t)).*sin(q4(t)).*1.23e+2+cos(q2(t)).*cos(q5(t)).*sin(q4(t)).*sin(q6(t)).*1.23e+2+sin(q2(t)).*sin(q3(t)).*sin(q5(t)).*sin(q6(t)).*1.23e+2-cos(q3(t)).*cos(q4(t)).*cos(q5(t)).*sin(q2(t)).*sin(q6(t)).*1.23e+2))./5.0e+2,(sin(q1(t)).*(cos(q2(t)).*2.0e+2+cos(q2(t)).*cos(q4(t)).*2.0e+2+cos(q2(t)).*cos(q4(t)).*cos(q6(t)).*1.23e+2+cos(q3(t)).*sin(q2(t)).*sin(q4(t)).*2.0e+2+cos(q3(t)).*cos(q6(t)).*sin(q2(t)).*sin(q4(t)).*1.23e+2+cos(q2(t)).*cos(q5(t)).*sin(q4(t)).*sin(q6(t)).*1.23e+2+sin(q2(t)).*sin(q3(t)).*sin(q5(t)).*sin(q6(t)).*1.23e+2-cos(q3(t)).*cos(q4(t)).*cos(q5(t)).*sin(q2(t)).*sin(q6(t)).*1.23e+2))./5.0e+2,sin(q2(t)).*(-2.0./5.0)-cos(q4(t)).*sin(q2(t)).*(2.0./5.0)-sin(q6(t)).*(cos(q5(t)).*(sin(q2(t)).*sin(q4(t))+cos(q2(t)).*cos(q3(t)).*cos(q4(t)))-cos(q2(t)).*sin(q3(t)).*sin(q5(t))).*(1.23e+2./5.0e+2)-cos(q6(t)).*(cos(q4(t)).*sin(q2(t))-cos(q2(t)).*cos(q3(t)).*sin(q4(t))).*(1.23e+2./5.0e+2)+cos(q2(t)).*cos(q3(t)).*sin(q4(t)).*(2.0./5.0),-sin(q1(t)),cos(q1(t)),0.0,sin(q4(t)).*(cos(q3(t)).*sin(q1(t))+cos(q1(t)).*cos(q2(t)).*sin(q3(t))).*(2.0./5.0)+sin(q6(t)).*(sin(q5(t)).*(sin(q1(t)).*sin(q3(t))-cos(q1(t)).*cos(q2(t)).*cos(q3(t)))-cos(q4(t)).*cos(q5(t)).*(cos(q3(t)).*sin(q1(t))+cos(q1(t)).*cos(q2(t)).*sin(q3(t)))).*(1.23e+2./5.0e+2)+cos(q6(t)).*sin(q4(t)).*(cos(q3(t)).*sin(q1(t))+cos(q1(t)).*cos(q2(t)).*sin(q3(t))).*(1.23e+2./5.0e+2),sin(q4(t)).*(cos(q1(t)).*cos(q3(t))-cos(q2(t)).*sin(q1(t)).*sin(q3(t))).*(-2.0./5.0)-sin(q6(t)).*(sin(q5(t)).*(cos(q1(t)).*sin(q3(t))+cos(q2(t)).*cos(q3(t)).*sin(q1(t)))-cos(q4(t)).*cos(q5(t)).*(cos(q1(t)).*cos(q3(t))-cos(q2(t)).*sin(q1(t)).*sin(q3(t)))).*(1.23e+2./5.0e+2)-cos(q6(t)).*sin(q4(t)).*(cos(q1(t)).*cos(q3(t))-cos(q2(t)).*sin(q1(t)).*sin(q3(t))).*(1.23e+2./5.0e+2),sin(q2(t)).*(sin(q3(t)).*sin(q4(t)).*2.0e+2+cos(q6(t)).*sin(q3(t)).*sin(q4(t)).*1.23e+2-cos(q3(t)).*sin(q5(t)).*sin(q6(t)).*1.23e+2-cos(q4(t)).*cos(q5(t)).*sin(q3(t)).*sin(q6(t)).*1.23e+2).*(-1.0./5.0e+2),cos(q1(t)).*sin(q2(t)),sin(q1(t)).*sin(q2(t)),cos(q2(t)),cos(q6(t)).*(cos(q4(t)).*(sin(q1(t)).*sin(q3(t))-cos(q1(t)).*cos(q2(t)).*cos(q3(t)))-cos(q1(t)).*sin(q2(t)).*sin(q4(t))).*(1.23e+2./5.0e+2)+cos(q4(t)).*(sin(q1(t)).*sin(q3(t))-cos(q1(t)).*cos(q2(t)).*cos(q3(t))).*(2.0./5.0)+cos(q5(t)).*sin(q6(t)).*(sin(q4(t)).*(sin(q1(t)).*sin(q3(t))-cos(q1(t)).*cos(q2(t)).*cos(q3(t)))+cos(q1(t)).*cos(q4(t)).*sin(q2(t))).*(1.23e+2./5.0e+2)-cos(q1(t)).*sin(q2(t)).*sin(q4(t)).*(2.0./5.0),cos(q6(t)).*(cos(q4(t)).*(cos(q1(t)).*sin(q3(t))+cos(q2(t)).*cos(q3(t)).*sin(q1(t)))+sin(q1(t)).*sin(q2(t)).*sin(q4(t))).*(-1.23e+2./5.0e+2)-cos(q4(t)).*(cos(q1(t)).*sin(q3(t))+cos(q2(t)).*cos(q3(t)).*sin(q1(t))).*(2.0./5.0)-sin(q1(t)).*sin(q2(t)).*sin(q4(t)).*(2.0./5.0)-cos(q5(t)).*sin(q6(t)).*(sin(q4(t)).*(cos(q1(t)).*sin(q3(t))+cos(q2(t)).*cos(q3(t)).*sin(q1(t)))-cos(q4(t)).*sin(q1(t)).*sin(q2(t))).*(1.23e+2./5.0e+2),cos(q2(t)).*sin(q4(t)).*(-2.0./5.0)-cos(q6(t)).*(cos(q2(t)).*sin(q4(t))-cos(q3(t)).*cos(q4(t)).*sin(q2(t))).*(1.23e+2./5.0e+2)+cos(q5(t)).*sin(q6(t)).*(cos(q2(t)).*cos(q4(t))+cos(q3(t)).*sin(q2(t)).*sin(q4(t))).*(1.23e+2./5.0e+2)+cos(q3(t)).*cos(q4(t)).*sin(q2(t)).*(2.0./5.0),cos(q3(t)).*sin(q1(t))+cos(q1(t)).*cos(q2(t)).*sin(q3(t)),-cos(q1(t)).*cos(q3(t))+cos(q2(t)).*sin(q1(t)).*sin(q3(t)),-sin(q2(t)).*sin(q3(t)),sin(q6(t)).*(sin(q5(t)).*(cos(q4(t)).*(sin(q1(t)).*sin(q3(t))-cos(q1(t)).*cos(q2(t)).*cos(q3(t)))-cos(q1(t)).*sin(q2(t)).*sin(q4(t)))-cos(q5(t)).*(cos(q3(t)).*sin(q1(t))+cos(q1(t)).*cos(q2(t)).*sin(q3(t)))).*(1.23e+2./5.0e+2),sin(q6(t)).*(sin(q5(t)).*(cos(q4(t)).*(cos(q1(t)).*sin(q3(t))+cos(q2(t)).*cos(q3(t)).*sin(q1(t)))+sin(q1(t)).*sin(q2(t)).*sin(q4(t)))-cos(q5(t)).*(cos(q1(t)).*cos(q3(t))-cos(q2(t)).*sin(q1(t)).*sin(q3(t)))).*(-1.23e+2./5.0e+2),sin(q6(t)).*(sin(q5(t)).*(cos(q2(t)).*sin(q4(t))-cos(q3(t)).*cos(q4(t)).*sin(q2(t)))-cos(q5(t)).*sin(q2(t)).*sin(q3(t))).*(-1.23e+2./5.0e+2),sin(q4(t)).*(sin(q1(t)).*sin(q3(t))-cos(q1(t)).*cos(q2(t)).*cos(q3(t)))+cos(q1(t)).*cos(q4(t)).*sin(q2(t)),-sin(q4(t)).*(cos(q1(t)).*sin(q3(t))+cos(q2(t)).*cos(q3(t)).*sin(q1(t)))+cos(q4(t)).*sin(q1(t)).*sin(q2(t)),cos(q2(t)).*cos(q4(t))+cos(q3(t)).*sin(q2(t)).*sin(q4(t)),sin(q6(t)).*(sin(q4(t)).*(sin(q1(t)).*sin(q3(t))-cos(q1(t)).*cos(q2(t)).*cos(q3(t)))+cos(q1(t)).*cos(q4(t)).*sin(q2(t))).*(-1.23e+2./5.0e+2)-cos(q6(t)).*(cos(q5(t)).*(cos(q4(t)).*(sin(q1(t)).*sin(q3(t))-cos(q1(t)).*cos(q2(t)).*cos(q3(t)))-cos(q1(t)).*sin(q2(t)).*sin(q4(t)))+sin(q5(t)).*(cos(q3(t)).*sin(q1(t))+cos(q1(t)).*cos(q2(t)).*sin(q3(t)))).*(1.23e+2./5.0e+2),sin(q6(t)).*(sin(q4(t)).*(cos(q1(t)).*sin(q3(t))+cos(q2(t)).*cos(q3(t)).*sin(q1(t)))-cos(q4(t)).*sin(q1(t)).*sin(q2(t))).*(1.23e+2./5.0e+2)+cos(q6(t)).*(cos(q5(t)).*(cos(q4(t)).*(cos(q1(t)).*sin(q3(t))+cos(q2(t)).*cos(q3(t)).*sin(q1(t)))+sin(q1(t)).*sin(q2(t)).*sin(q4(t)))+sin(q5(t)).*(cos(q1(t)).*cos(q3(t))-cos(q2(t)).*sin(q1(t)).*sin(q3(t)))).*(1.23e+2./5.0e+2),cos(q6(t)).*(cos(q5(t)).*(cos(q2(t)).*sin(q4(t))-cos(q3(t)).*cos(q4(t)).*sin(q2(t)))+sin(q2(t)).*sin(q3(t)).*sin(q5(t))).*(1.23e+2./5.0e+2)-sin(q6(t)).*(cos(q2(t)).*cos(q4(t))+cos(q3(t)).*sin(q2(t)).*sin(q4(t))).*(1.23e+2./5.0e+2),sin(q5(t)).*(cos(q4(t)).*(sin(q1(t)).*sin(q3(t))-cos(q1(t)).*cos(q2(t)).*cos(q3(t)))-cos(q1(t)).*sin(q2(t)).*sin(q4(t)))-cos(q5(t)).*(cos(q3(t)).*sin(q1(t))+cos(q1(t)).*cos(q2(t)).*sin(q3(t))),-sin(q5(t)).*(cos(q4(t)).*(cos(q1(t)).*sin(q3(t))+cos(q2(t)).*cos(q3(t)).*sin(q1(t)))+sin(q1(t)).*sin(q2(t)).*sin(q4(t)))+cos(q5(t)).*(cos(q1(t)).*cos(q3(t))-cos(q2(t)).*sin(q1(t)).*sin(q3(t))),-sin(q5(t)).*(cos(q2(t)).*sin(q4(t))-cos(q3(t)).*cos(q4(t)).*sin(q2(t)))+cos(q5(t)).*sin(q2(t)).*sin(q3(t)),0.0,0.0,0.0,cos(q6(t)).*(sin(q4(t)).*(sin(q1(t)).*sin(q3(t))-cos(q1(t)).*cos(q2(t)).*cos(q3(t)))+cos(q1(t)).*cos(q4(t)).*sin(q2(t)))-sin(q6(t)).*(cos(q5(t)).*(cos(q4(t)).*(sin(q1(t)).*sin(q3(t))-cos(q1(t)).*cos(q2(t)).*cos(q3(t)))-cos(q1(t)).*sin(q2(t)).*sin(q4(t)))+sin(q5(t)).*(cos(q3(t)).*sin(q1(t))+cos(q1(t)).*cos(q2(t)).*sin(q3(t)))),-cos(q6(t)).*(sin(q4(t)).*(cos(q1(t)).*sin(q3(t))+cos(q2(t)).*cos(q3(t)).*sin(q1(t)))-cos(q4(t)).*sin(q1(t)).*sin(q2(t)))+sin(q6(t)).*(cos(q5(t)).*(cos(q4(t)).*(cos(q1(t)).*sin(q3(t))+cos(q2(t)).*cos(q3(t)).*sin(q1(t)))+sin(q1(t)).*sin(q2(t)).*sin(q4(t)))+sin(q5(t)).*(cos(q1(t)).*cos(q3(t))-cos(q2(t)).*sin(q1(t)).*sin(q3(t)))),sin(q6(t)).*(cos(q5(t)).*(cos(q2(t)).*sin(q4(t))-cos(q3(t)).*cos(q4(t)).*sin(q2(t)))+sin(q2(t)).*sin(q3(t)).*sin(q5(t)))+cos(q6(t)).*(cos(q2(t)).*cos(q4(t))+cos(q3(t)).*sin(q2(t)).*sin(q4(t)))],[6,7]);
