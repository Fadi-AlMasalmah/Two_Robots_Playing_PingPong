function J_diff = calc_J_diff(q,q_)
%CALC_J_DIFF
%    J_DIFF = CALC_J_DIFF(T)
q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);
q5 = q(5);
q6 = q(6);
q7 = q(7);
q1_ = q_(1);
q2_ = q_(2);
q3_ = q_(3);
q4_ = q_(4);
q5_ = q_(5);
q6_ = q_(6);
q7_ = q_(7);

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    21-Mar-2020 00:18:52

J_diff = reshape([sin(q6).*(cos(q5).*(-cos(q4).*(-cos(q1).*cos(q3).*q3_+sin(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*sin(q2).*q2_+cos(q2).*sin(q1).*sin(q3).*q3_-cos(q1).*cos(q2).*cos(q3).*q1_)-sin(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1)).*q4_+cos(q1).*sin(q2).*sin(q4).*q1_+cos(q2).*sin(q1).*sin(q4).*q2_+cos(q4).*sin(q1).*sin(q2).*q4_)-sin(q5).*(cos(q3).*sin(q1).*q1_+cos(q1).*sin(q3).*q3_+cos(q1).*cos(q2).*sin(q3).*q1_+cos(q2).*cos(q3).*sin(q1).*q3_-sin(q1).*sin(q2).*sin(q3).*q2_)+cos(q5).*(cos(q1).*cos(q3)-cos(q2).*sin(q1).*sin(q3)).*q5_-sin(q5).*(cos(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1))+sin(q1).*sin(q2).*sin(q4)).*q5_).*(-1.23e+2./5.0e+2)-sin(q4).*(-cos(q1).*cos(q3).*q3_+sin(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*sin(q2).*q2_+cos(q2).*sin(q1).*sin(q3).*q3_-cos(q1).*cos(q2).*cos(q3).*q1_).*(2.0./5.0)-cos(q6).*(sin(q4).*(-cos(q1).*cos(q3).*q3_+sin(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*sin(q2).*q2_+cos(q2).*sin(q1).*sin(q3).*q3_-cos(q1).*cos(q2).*cos(q3).*q1_)-cos(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1)).*q4_+cos(q1).*cos(q4).*sin(q2).*q1_+cos(q2).*cos(q4).*sin(q1).*q2_-sin(q1).*sin(q2).*sin(q4).*q4_).*(1.23e+2./5.0e+2)+cos(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1)).*q4_.*(2.0./5.0)-sin(q6).*(sin(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1))-cos(q4).*sin(q1).*sin(q2)).*q6_.*(1.23e+2./5.0e+2)-cos(q1).*sin(q2).*q1_.*(2.0./5.0)-cos(q2).*sin(q1).*q2_.*(2.0./5.0)-cos(q6).*(sin(q5).*(cos(q1).*cos(q3)-cos(q2).*sin(q1).*sin(q3))+cos(q5).*(cos(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1))+sin(q1).*sin(q2).*sin(q4))).*q6_.*(1.23e+2./5.0e+2)-cos(q1).*cos(q4).*sin(q2).*q1_.*(2.0./5.0)-cos(q2).*cos(q4).*sin(q1).*q2_.*(2.0./5.0)+sin(q1).*sin(q2).*sin(q4).*q4_.*(2.0./5.0),sin(q6).*(sin(q5).*(-cos(q1).*cos(q3).*q1_+sin(q1).*sin(q3).*q3_+cos(q2).*sin(q1).*sin(q3).*q1_+cos(q1).*sin(q2).*sin(q3).*q2_-cos(q1).*cos(q2).*cos(q3).*q3_)+cos(q5).*(-cos(q4).*(cos(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*q3_+cos(q2).*cos(q3).*sin(q1).*q1_+cos(q1).*cos(q3).*sin(q2).*q2_+cos(q1).*cos(q2).*sin(q3).*q3_)+sin(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3)).*q4_+cos(q1).*cos(q2).*sin(q4).*q2_+cos(q1).*cos(q4).*sin(q2).*q4_-sin(q1).*sin(q2).*sin(q4).*q1_)-cos(q5).*(cos(q3).*sin(q1)+cos(q1).*cos(q2).*sin(q3)).*q5_+sin(q5).*(cos(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3))-cos(q1).*sin(q2).*sin(q4)).*q5_).*(1.23e+2./5.0e+2)+sin(q4).*(cos(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*q3_+cos(q2).*cos(q3).*sin(q1).*q1_+cos(q1).*cos(q3).*sin(q2).*q2_+cos(q1).*cos(q2).*sin(q3).*q3_).*(2.0./5.0)+cos(q6).*(sin(q4).*(cos(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*q3_+cos(q2).*cos(q3).*sin(q1).*q1_+cos(q1).*cos(q3).*sin(q2).*q2_+cos(q1).*cos(q2).*sin(q3).*q3_)+cos(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3)).*q4_-cos(q4).*sin(q1).*sin(q2).*q1_-cos(q1).*sin(q2).*sin(q4).*q4_+cos(q1).*cos(q2).*cos(q4).*q2_).*(1.23e+2./5.0e+2)+cos(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3)).*q4_.*(2.0./5.0)-sin(q6).*(sin(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3))+cos(q1).*cos(q4).*sin(q2)).*q6_.*(1.23e+2./5.0e+2)+cos(q1).*cos(q2).*q2_.*(2.0./5.0)-sin(q1).*sin(q2).*q1_.*(2.0./5.0)-cos(q6).*(sin(q5).*(cos(q3).*sin(q1)+cos(q1).*cos(q2).*sin(q3))+cos(q5).*(cos(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3))-cos(q1).*sin(q2).*sin(q4))).*q6_.*(1.23e+2./5.0e+2)-cos(q4).*sin(q1).*sin(q2).*q1_.*(2.0./5.0)-cos(q1).*sin(q2).*sin(q4).*q4_.*(2.0./5.0)+cos(q1).*cos(q2).*cos(q4).*q2_.*(2.0./5.0),0.0,0.0,0.0,0.0,cos(q1).*(sin(q2).*q2_.*2.0e+2+cos(q4).*sin(q2).*q2_.*2.0e+2+cos(q2).*sin(q4).*q4_.*2.0e+2-cos(q2).*cos(q3).*sin(q4).*q2_.*2.0e+2-cos(q3).*cos(q4).*sin(q2).*q4_.*2.0e+2+cos(q4).*cos(q6).*sin(q2).*q2_.*1.23e+2+cos(q2).*cos(q6).*sin(q4).*q4_.*1.23e+2+cos(q2).*cos(q4).*sin(q6).*q6_.*1.23e+2+sin(q2).*sin(q3).*sin(q4).*q3_.*2.0e+2-cos(q2).*cos(q3).*cos(q6).*sin(q4).*q2_.*1.23e+2-cos(q3).*cos(q4).*cos(q6).*sin(q2).*q4_.*1.23e+2-cos(q2).*cos(q4).*cos(q5).*sin(q6).*q4_.*1.23e+2-cos(q2).*cos(q5).*cos(q6).*sin(q4).*q6_.*1.23e+2-cos(q2).*sin(q3).*sin(q5).*sin(q6).*q2_.*1.23e+2+cos(q6).*sin(q2).*sin(q3).*sin(q4).*q3_.*1.23e+2-cos(q3).*sin(q2).*sin(q5).*sin(q6).*q3_.*1.23e+2+cos(q5).*sin(q2).*sin(q4).*sin(q6).*q2_.*1.23e+2+cos(q3).*sin(q2).*sin(q4).*sin(q6).*q6_.*1.23e+2-cos(q5).*sin(q2).*sin(q3).*sin(q6).*q5_.*1.23e+2+cos(q2).*sin(q4).*sin(q5).*sin(q6).*q5_.*1.23e+2-cos(q6).*sin(q2).*sin(q3).*sin(q5).*q6_.*1.23e+2+cos(q2).*cos(q3).*cos(q4).*cos(q5).*sin(q6).*q2_.*1.23e+2+cos(q3).*cos(q4).*cos(q5).*cos(q6).*sin(q2).*q6_.*1.23e+2-cos(q4).*cos(q5).*sin(q2).*sin(q3).*sin(q6).*q3_.*1.23e+2-cos(q3).*cos(q5).*sin(q2).*sin(q4).*sin(q6).*q4_.*1.23e+2-cos(q3).*cos(q4).*sin(q2).*sin(q5).*sin(q6).*q5_.*1.23e+2).*(-1.0./5.0e+2)-(sin(q1).*q1_.*(cos(q2).*2.0e+2+cos(q2).*cos(q4).*2.0e+2+cos(q2).*cos(q4).*cos(q6).*1.23e+2+cos(q3).*sin(q2).*sin(q4).*2.0e+2+cos(q3).*cos(q6).*sin(q2).*sin(q4).*1.23e+2+cos(q2).*cos(q5).*sin(q4).*sin(q6).*1.23e+2+sin(q2).*sin(q3).*sin(q5).*sin(q6).*1.23e+2-cos(q3).*cos(q4).*cos(q5).*sin(q2).*sin(q6).*1.23e+2))./5.0e+2,sin(q1).*(sin(q2).*q2_.*2.0e+2+cos(q4).*sin(q2).*q2_.*2.0e+2+cos(q2).*sin(q4).*q4_.*2.0e+2-cos(q2).*cos(q3).*sin(q4).*q2_.*2.0e+2-cos(q3).*cos(q4).*sin(q2).*q4_.*2.0e+2+cos(q4).*cos(q6).*sin(q2).*q2_.*1.23e+2+cos(q2).*cos(q6).*sin(q4).*q4_.*1.23e+2+cos(q2).*cos(q4).*sin(q6).*q6_.*1.23e+2+sin(q2).*sin(q3).*sin(q4).*q3_.*2.0e+2-cos(q2).*cos(q3).*cos(q6).*sin(q4).*q2_.*1.23e+2-cos(q3).*cos(q4).*cos(q6).*sin(q2).*q4_.*1.23e+2-cos(q2).*cos(q4).*cos(q5).*sin(q6).*q4_.*1.23e+2-cos(q2).*cos(q5).*cos(q6).*sin(q4).*q6_.*1.23e+2-cos(q2).*sin(q3).*sin(q5).*sin(q6).*q2_.*1.23e+2+cos(q6).*sin(q2).*sin(q3).*sin(q4).*q3_.*1.23e+2-cos(q3).*sin(q2).*sin(q5).*sin(q6).*q3_.*1.23e+2+cos(q5).*sin(q2).*sin(q4).*sin(q6).*q2_.*1.23e+2+cos(q3).*sin(q2).*sin(q4).*sin(q6).*q6_.*1.23e+2-cos(q5).*sin(q2).*sin(q3).*sin(q6).*q5_.*1.23e+2+cos(q2).*sin(q4).*sin(q5).*sin(q6).*q5_.*1.23e+2-cos(q6).*sin(q2).*sin(q3).*sin(q5).*q6_.*1.23e+2+cos(q2).*cos(q3).*cos(q4).*cos(q5).*sin(q6).*q2_.*1.23e+2+cos(q3).*cos(q4).*cos(q5).*cos(q6).*sin(q2).*q6_.*1.23e+2-cos(q4).*cos(q5).*sin(q2).*sin(q3).*sin(q6).*q3_.*1.23e+2-cos(q3).*cos(q5).*sin(q2).*sin(q4).*sin(q6).*q4_.*1.23e+2-cos(q3).*cos(q4).*sin(q2).*sin(q5).*sin(q6).*q5_.*1.23e+2).*(-1.0./5.0e+2)+(cos(q1).*q1_.*(cos(q2).*2.0e+2+cos(q2).*cos(q4).*2.0e+2+cos(q2).*cos(q4).*cos(q6).*1.23e+2+cos(q3).*sin(q2).*sin(q4).*2.0e+2+cos(q3).*cos(q6).*sin(q2).*sin(q4).*1.23e+2+cos(q2).*cos(q5).*sin(q4).*sin(q6).*1.23e+2+sin(q2).*sin(q3).*sin(q5).*sin(q6).*1.23e+2-cos(q3).*cos(q4).*cos(q5).*sin(q2).*sin(q6).*1.23e+2))./5.0e+2,cos(q2).*q2_.*(-2.0./5.0)-cos(q6).*(cos(q2).*cos(q4).*q2_-sin(q2).*sin(q4).*q4_+cos(q3).*sin(q2).*sin(q4).*q2_+cos(q2).*sin(q3).*sin(q4).*q3_-cos(q2).*cos(q3).*cos(q4).*q4_).*(1.23e+2./5.0e+2)+sin(q6).*(cos(q5).*(-cos(q2).*sin(q4).*q2_-cos(q4).*sin(q2).*q4_+cos(q3).*cos(q4).*sin(q2).*q2_+cos(q2).*cos(q4).*sin(q3).*q3_+cos(q2).*cos(q3).*sin(q4).*q4_)+sin(q5).*(sin(q2).*sin(q4)+cos(q2).*cos(q3).*cos(q4)).*q5_+cos(q2).*cos(q3).*sin(q5).*q3_+cos(q2).*cos(q5).*sin(q3).*q5_-sin(q2).*sin(q3).*sin(q5).*q2_).*(1.23e+2./5.0e+2)+sin(q6).*(cos(q4).*sin(q2)-cos(q2).*cos(q3).*sin(q4)).*q6_.*(1.23e+2./5.0e+2)-cos(q6).*(cos(q5).*(sin(q2).*sin(q4)+cos(q2).*cos(q3).*cos(q4))-cos(q2).*sin(q3).*sin(q5)).*q6_.*(1.23e+2./5.0e+2)-cos(q2).*cos(q4).*q2_.*(2.0./5.0)+sin(q2).*sin(q4).*q4_.*(2.0./5.0)-cos(q3).*sin(q2).*sin(q4).*q2_.*(2.0./5.0)-cos(q2).*sin(q3).*sin(q4).*q3_.*(2.0./5.0)+cos(q2).*cos(q3).*cos(q4).*q4_.*(2.0./5.0),-cos(q1).*q1_,-sin(q1).*q1_,0.0,sin(q6).*(sin(q5).*(cos(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*q3_+cos(q2).*cos(q3).*sin(q1).*q1_+cos(q1).*cos(q3).*sin(q2).*q2_+cos(q1).*cos(q2).*sin(q3).*q3_)+cos(q5).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3)).*q5_+cos(q4).*cos(q5).*(-cos(q1).*cos(q3).*q1_+sin(q1).*sin(q3).*q3_+cos(q2).*sin(q1).*sin(q3).*q1_+cos(q1).*sin(q2).*sin(q3).*q2_-cos(q1).*cos(q2).*cos(q3).*q3_)+cos(q5).*sin(q4).*(cos(q3).*sin(q1)+cos(q1).*cos(q2).*sin(q3)).*q4_+cos(q4).*sin(q5).*(cos(q3).*sin(q1)+cos(q1).*cos(q2).*sin(q3)).*q5_).*(1.23e+2./5.0e+2)-sin(q4).*(-cos(q1).*cos(q3).*q1_+sin(q1).*sin(q3).*q3_+cos(q2).*sin(q1).*sin(q3).*q1_+cos(q1).*sin(q2).*sin(q3).*q2_-cos(q1).*cos(q2).*cos(q3).*q3_).*(2.0./5.0)+cos(q4).*(cos(q3).*sin(q1)+cos(q1).*cos(q2).*sin(q3)).*q4_.*(2.0./5.0)+cos(q6).*(sin(q5).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3))-cos(q4).*cos(q5).*(cos(q3).*sin(q1)+cos(q1).*cos(q2).*sin(q3))).*q6_.*(1.23e+2./5.0e+2)-cos(q6).*sin(q4).*(-cos(q1).*cos(q3).*q1_+sin(q1).*sin(q3).*q3_+cos(q2).*sin(q1).*sin(q3).*q1_+cos(q1).*sin(q2).*sin(q3).*q2_-cos(q1).*cos(q2).*cos(q3).*q3_).*(1.23e+2./5.0e+2)+cos(q4).*cos(q6).*(cos(q3).*sin(q1)+cos(q1).*cos(q2).*sin(q3)).*q4_.*(1.23e+2./5.0e+2)-sin(q4).*sin(q6).*(cos(q3).*sin(q1)+cos(q1).*cos(q2).*sin(q3)).*q6_.*(1.23e+2./5.0e+2),sin(q6).*(-sin(q5).*(-cos(q1).*cos(q3).*q3_+sin(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*sin(q2).*q2_+cos(q2).*sin(q1).*sin(q3).*q3_-cos(q1).*cos(q2).*cos(q3).*q1_)+cos(q5).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1)).*q5_+cos(q4).*cos(q5).*(cos(q3).*sin(q1).*q1_+cos(q1).*sin(q3).*q3_+cos(q1).*cos(q2).*sin(q3).*q1_+cos(q2).*cos(q3).*sin(q1).*q3_-sin(q1).*sin(q2).*sin(q3).*q2_)+cos(q5).*sin(q4).*(cos(q1).*cos(q3)-cos(q2).*sin(q1).*sin(q3)).*q4_+cos(q4).*sin(q5).*(cos(q1).*cos(q3)-cos(q2).*sin(q1).*sin(q3)).*q5_).*(-1.23e+2./5.0e+2)+sin(q4).*(cos(q3).*sin(q1).*q1_+cos(q1).*sin(q3).*q3_+cos(q1).*cos(q2).*sin(q3).*q1_+cos(q2).*cos(q3).*sin(q1).*q3_-sin(q1).*sin(q2).*sin(q3).*q2_).*(2.0./5.0)+cos(q6).*sin(q4).*(cos(q3).*sin(q1).*q1_+cos(q1).*sin(q3).*q3_+cos(q1).*cos(q2).*sin(q3).*q1_+cos(q2).*cos(q3).*sin(q1).*q3_-sin(q1).*sin(q2).*sin(q3).*q2_).*(1.23e+2./5.0e+2)-cos(q4).*(cos(q1).*cos(q3)-cos(q2).*sin(q1).*sin(q3)).*q4_.*(2.0./5.0)-cos(q6).*(sin(q5).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1))-cos(q4).*cos(q5).*(cos(q1).*cos(q3)-cos(q2).*sin(q1).*sin(q3))).*q6_.*(1.23e+2./5.0e+2)-cos(q4).*cos(q6).*(cos(q1).*cos(q3)-cos(q2).*sin(q1).*sin(q3)).*q4_.*(1.23e+2./5.0e+2)+sin(q4).*sin(q6).*(cos(q1).*cos(q3)-cos(q2).*sin(q1).*sin(q3)).*q6_.*(1.23e+2./5.0e+2),sin(q2).*(cos(q3).*sin(q4).*q3_.*2.0e+2+cos(q4).*sin(q3).*q4_.*2.0e+2+cos(q3).*cos(q6).*sin(q4).*q3_.*1.23e+2+cos(q4).*cos(q6).*sin(q3).*q4_.*1.23e+2-cos(q3).*cos(q5).*sin(q6).*q5_.*1.23e+2-cos(q3).*cos(q6).*sin(q5).*q6_.*1.23e+2+sin(q3).*sin(q5).*sin(q6).*q3_.*1.23e+2-sin(q3).*sin(q4).*sin(q6).*q6_.*1.23e+2-cos(q3).*cos(q4).*cos(q5).*sin(q6).*q3_.*1.23e+2-cos(q4).*cos(q5).*cos(q6).*sin(q3).*q6_.*1.23e+2+cos(q5).*sin(q3).*sin(q4).*sin(q6).*q4_.*1.23e+2+cos(q4).*sin(q3).*sin(q5).*sin(q6).*q5_.*1.23e+2).*(-1.0./5.0e+2)-(cos(q2).*q2_.*(sin(q3).*sin(q4).*2.0e+2+cos(q6).*sin(q3).*sin(q4).*1.23e+2-cos(q3).*sin(q5).*sin(q6).*1.23e+2-cos(q4).*cos(q5).*sin(q3).*sin(q6).*1.23e+2))./5.0e+2,cos(q1).*cos(q2).*q2_-sin(q1).*sin(q2).*q1_,cos(q1).*sin(q2).*q1_+cos(q2).*sin(q1).*q2_,-sin(q2).*q2_,cos(q4).*(cos(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*q3_+cos(q2).*cos(q3).*sin(q1).*q1_+cos(q1).*cos(q3).*sin(q2).*q2_+cos(q1).*cos(q2).*sin(q3).*q3_).*(2.0./5.0)-cos(q6).*(-cos(q4).*(cos(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*q3_+cos(q2).*cos(q3).*sin(q1).*q1_+cos(q1).*cos(q3).*sin(q2).*q2_+cos(q1).*cos(q2).*sin(q3).*q3_)+sin(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3)).*q4_+cos(q1).*cos(q2).*sin(q4).*q2_+cos(q1).*cos(q4).*sin(q2).*q4_-sin(q1).*sin(q2).*sin(q4).*q1_).*(1.23e+2./5.0e+2)-sin(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3)).*q4_.*(2.0./5.0)+cos(q5).*sin(q6).*(sin(q4).*(cos(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*q3_+cos(q2).*cos(q3).*sin(q1).*q1_+cos(q1).*cos(q3).*sin(q2).*q2_+cos(q1).*cos(q2).*sin(q3).*q3_)+cos(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3)).*q4_-cos(q4).*sin(q1).*sin(q2).*q1_-cos(q1).*sin(q2).*sin(q4).*q4_+cos(q1).*cos(q2).*cos(q4).*q2_).*(1.23e+2./5.0e+2)-sin(q6).*(cos(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3))-cos(q1).*sin(q2).*sin(q4)).*q6_.*(1.23e+2./5.0e+2)-cos(q1).*cos(q2).*sin(q4).*q2_.*(2.0./5.0)-cos(q1).*cos(q4).*sin(q2).*q4_.*(2.0./5.0)+sin(q1).*sin(q2).*sin(q4).*q1_.*(2.0./5.0)+cos(q5).*cos(q6).*(sin(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3))+cos(q1).*cos(q4).*sin(q2)).*q6_.*(1.23e+2./5.0e+2)-sin(q5).*sin(q6).*(sin(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3))+cos(q1).*cos(q4).*sin(q2)).*q5_.*(1.23e+2./5.0e+2),cos(q6).*(-cos(q4).*(-cos(q1).*cos(q3).*q3_+sin(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*sin(q2).*q2_+cos(q2).*sin(q1).*sin(q3).*q3_-cos(q1).*cos(q2).*cos(q3).*q1_)-sin(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1)).*q4_+cos(q1).*sin(q2).*sin(q4).*q1_+cos(q2).*sin(q1).*sin(q4).*q2_+cos(q4).*sin(q1).*sin(q2).*q4_).*(-1.23e+2./5.0e+2)+cos(q4).*(-cos(q1).*cos(q3).*q3_+sin(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*sin(q2).*q2_+cos(q2).*sin(q1).*sin(q3).*q3_-cos(q1).*cos(q2).*cos(q3).*q1_).*(2.0./5.0)+sin(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1)).*q4_.*(2.0./5.0)+cos(q5).*sin(q6).*(sin(q4).*(-cos(q1).*cos(q3).*q3_+sin(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*sin(q2).*q2_+cos(q2).*sin(q1).*sin(q3).*q3_-cos(q1).*cos(q2).*cos(q3).*q1_)-cos(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1)).*q4_+cos(q1).*cos(q4).*sin(q2).*q1_+cos(q2).*cos(q4).*sin(q1).*q2_-sin(q1).*sin(q2).*sin(q4).*q4_).*(1.23e+2./5.0e+2)+sin(q6).*(cos(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1))+sin(q1).*sin(q2).*sin(q4)).*q6_.*(1.23e+2./5.0e+2)-cos(q1).*sin(q2).*sin(q4).*q1_.*(2.0./5.0)-cos(q2).*sin(q1).*sin(q4).*q2_.*(2.0./5.0)-cos(q4).*sin(q1).*sin(q2).*q4_.*(2.0./5.0)-cos(q5).*cos(q6).*(sin(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1))-cos(q4).*sin(q1).*sin(q2)).*q6_.*(1.23e+2./5.0e+2)+sin(q5).*sin(q6).*(sin(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1))-cos(q4).*sin(q1).*sin(q2)).*q5_.*(1.23e+2./5.0e+2),cos(q6).*(cos(q2).*cos(q4).*q4_-sin(q2).*sin(q4).*q2_+cos(q4).*sin(q2).*sin(q3).*q3_+cos(q3).*sin(q2).*sin(q4).*q4_-cos(q2).*cos(q3).*cos(q4).*q2_).*(-1.23e+2./5.0e+2)-cos(q5).*sin(q6).*(cos(q4).*sin(q2).*q2_+cos(q2).*sin(q4).*q4_-cos(q2).*cos(q3).*sin(q4).*q2_-cos(q3).*cos(q4).*sin(q2).*q4_+sin(q2).*sin(q3).*sin(q4).*q3_).*(1.23e+2./5.0e+2)+sin(q6).*(cos(q2).*sin(q4)-cos(q3).*cos(q4).*sin(q2)).*q6_.*(1.23e+2./5.0e+2)-cos(q2).*cos(q4).*q4_.*(2.0./5.0)+sin(q2).*sin(q4).*q2_.*(2.0./5.0)-cos(q4).*sin(q2).*sin(q3).*q3_.*(2.0./5.0)-cos(q3).*sin(q2).*sin(q4).*q4_.*(2.0./5.0)+cos(q5).*cos(q6).*(cos(q2).*cos(q4)+cos(q3).*sin(q2).*sin(q4)).*q6_.*(1.23e+2./5.0e+2)-sin(q5).*sin(q6).*(cos(q2).*cos(q4)+cos(q3).*sin(q2).*sin(q4)).*q5_.*(1.23e+2./5.0e+2)+cos(q2).*cos(q3).*cos(q4).*q2_.*(2.0./5.0),cos(q1).*cos(q3).*q1_-sin(q1).*sin(q3).*q3_-cos(q2).*sin(q1).*sin(q3).*q1_-cos(q1).*sin(q2).*sin(q3).*q2_+cos(q1).*cos(q2).*cos(q3).*q3_,cos(q3).*sin(q1).*q1_+cos(q1).*sin(q3).*q3_+cos(q1).*cos(q2).*sin(q3).*q1_+cos(q2).*cos(q3).*sin(q1).*q3_-sin(q1).*sin(q2).*sin(q3).*q2_,-cos(q2).*sin(q3).*q2_-cos(q3).*sin(q2).*q3_,sin(q6).*(cos(q5).*(-cos(q1).*cos(q3).*q1_+sin(q1).*sin(q3).*q3_+cos(q2).*sin(q1).*sin(q3).*q1_+cos(q1).*sin(q2).*sin(q3).*q2_-cos(q1).*cos(q2).*cos(q3).*q3_)-sin(q5).*(-cos(q4).*(cos(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*q3_+cos(q2).*cos(q3).*sin(q1).*q1_+cos(q1).*cos(q3).*sin(q2).*q2_+cos(q1).*cos(q2).*sin(q3).*q3_)+sin(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3)).*q4_+cos(q1).*cos(q2).*sin(q4).*q2_+cos(q1).*cos(q4).*sin(q2).*q4_-sin(q1).*sin(q2).*sin(q4).*q1_)+sin(q5).*(cos(q3).*sin(q1)+cos(q1).*cos(q2).*sin(q3)).*q5_+cos(q5).*(cos(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3))-cos(q1).*sin(q2).*sin(q4)).*q5_).*(1.23e+2./5.0e+2)-cos(q6).*(cos(q5).*(cos(q3).*sin(q1)+cos(q1).*cos(q2).*sin(q3))-sin(q5).*(cos(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3))-cos(q1).*sin(q2).*sin(q4))).*q6_.*(1.23e+2./5.0e+2),sin(q6).*(sin(q5).*(-cos(q4).*(-cos(q1).*cos(q3).*q3_+sin(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*sin(q2).*q2_+cos(q2).*sin(q1).*sin(q3).*q3_-cos(q1).*cos(q2).*cos(q3).*q1_)-sin(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1)).*q4_+cos(q1).*sin(q2).*sin(q4).*q1_+cos(q2).*sin(q1).*sin(q4).*q2_+cos(q4).*sin(q1).*sin(q2).*q4_)+cos(q5).*(cos(q3).*sin(q1).*q1_+cos(q1).*sin(q3).*q3_+cos(q1).*cos(q2).*sin(q3).*q1_+cos(q2).*cos(q3).*sin(q1).*q3_-sin(q1).*sin(q2).*sin(q3).*q2_)+sin(q5).*(cos(q1).*cos(q3)-cos(q2).*sin(q1).*sin(q3)).*q5_+cos(q5).*(cos(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1))+sin(q1).*sin(q2).*sin(q4)).*q5_).*(-1.23e+2./5.0e+2)+cos(q6).*(cos(q5).*(cos(q1).*cos(q3)-cos(q2).*sin(q1).*sin(q3))-sin(q5).*(cos(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1))+sin(q1).*sin(q2).*sin(q4))).*q6_.*(1.23e+2./5.0e+2),sin(q6).*(sin(q5).*(cos(q2).*cos(q4).*q4_-sin(q2).*sin(q4).*q2_+cos(q4).*sin(q2).*sin(q3).*q3_+cos(q3).*sin(q2).*sin(q4).*q4_-cos(q2).*cos(q3).*cos(q4).*q2_)+cos(q5).*(cos(q2).*sin(q4)-cos(q3).*cos(q4).*sin(q2)).*q5_-cos(q2).*cos(q5).*sin(q3).*q2_-cos(q3).*cos(q5).*sin(q2).*q3_+sin(q2).*sin(q3).*sin(q5).*q5_).*(-1.23e+2./5.0e+2)-cos(q6).*(sin(q5).*(cos(q2).*sin(q4)-cos(q3).*cos(q4).*sin(q2))-cos(q5).*sin(q2).*sin(q3)).*q6_.*(1.23e+2./5.0e+2),sin(q4).*(cos(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*q3_+cos(q2).*cos(q3).*sin(q1).*q1_+cos(q1).*cos(q3).*sin(q2).*q2_+cos(q1).*cos(q2).*sin(q3).*q3_)+cos(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3)).*q4_-cos(q4).*sin(q1).*sin(q2).*q1_-cos(q1).*sin(q2).*sin(q4).*q4_+cos(q1).*cos(q2).*cos(q4).*q2_,sin(q4).*(-cos(q1).*cos(q3).*q3_+sin(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*sin(q2).*q2_+cos(q2).*sin(q1).*sin(q3).*q3_-cos(q1).*cos(q2).*cos(q3).*q1_)-cos(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1)).*q4_+cos(q1).*cos(q4).*sin(q2).*q1_+cos(q2).*cos(q4).*sin(q1).*q2_-sin(q1).*sin(q2).*sin(q4).*q4_,-cos(q4).*sin(q2).*q2_-cos(q2).*sin(q4).*q4_+cos(q2).*cos(q3).*sin(q4).*q2_+cos(q3).*cos(q4).*sin(q2).*q4_-sin(q2).*sin(q3).*sin(q4).*q3_,cos(q6).*(sin(q5).*(-cos(q1).*cos(q3).*q1_+sin(q1).*sin(q3).*q3_+cos(q2).*sin(q1).*sin(q3).*q1_+cos(q1).*sin(q2).*sin(q3).*q2_-cos(q1).*cos(q2).*cos(q3).*q3_)+cos(q5).*(-cos(q4).*(cos(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*q3_+cos(q2).*cos(q3).*sin(q1).*q1_+cos(q1).*cos(q3).*sin(q2).*q2_+cos(q1).*cos(q2).*sin(q3).*q3_)+sin(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3)).*q4_+cos(q1).*cos(q2).*sin(q4).*q2_+cos(q1).*cos(q4).*sin(q2).*q4_-sin(q1).*sin(q2).*sin(q4).*q1_)-cos(q5).*(cos(q3).*sin(q1)+cos(q1).*cos(q2).*sin(q3)).*q5_+sin(q5).*(cos(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3))-cos(q1).*sin(q2).*sin(q4)).*q5_).*(1.23e+2./5.0e+2)-sin(q6).*(sin(q4).*(cos(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*q3_+cos(q2).*cos(q3).*sin(q1).*q1_+cos(q1).*cos(q3).*sin(q2).*q2_+cos(q1).*cos(q2).*sin(q3).*q3_)+cos(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3)).*q4_-cos(q4).*sin(q1).*sin(q2).*q1_-cos(q1).*sin(q2).*sin(q4).*q4_+cos(q1).*cos(q2).*cos(q4).*q2_).*(1.23e+2./5.0e+2)-cos(q6).*(sin(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3))+cos(q1).*cos(q4).*sin(q2)).*q6_.*(1.23e+2./5.0e+2)+sin(q6).*(sin(q5).*(cos(q3).*sin(q1)+cos(q1).*cos(q2).*sin(q3))+cos(q5).*(cos(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3))-cos(q1).*sin(q2).*sin(q4))).*q6_.*(1.23e+2./5.0e+2),sin(q6).*(sin(q4).*(-cos(q1).*cos(q3).*q3_+sin(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*sin(q2).*q2_+cos(q2).*sin(q1).*sin(q3).*q3_-cos(q1).*cos(q2).*cos(q3).*q1_)-cos(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1)).*q4_+cos(q1).*cos(q4).*sin(q2).*q1_+cos(q2).*cos(q4).*sin(q1).*q2_-sin(q1).*sin(q2).*sin(q4).*q4_).*(-1.23e+2./5.0e+2)+cos(q6).*(cos(q5).*(-cos(q4).*(-cos(q1).*cos(q3).*q3_+sin(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*sin(q2).*q2_+cos(q2).*sin(q1).*sin(q3).*q3_-cos(q1).*cos(q2).*cos(q3).*q1_)-sin(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1)).*q4_+cos(q1).*sin(q2).*sin(q4).*q1_+cos(q2).*sin(q1).*sin(q4).*q2_+cos(q4).*sin(q1).*sin(q2).*q4_)-sin(q5).*(cos(q3).*sin(q1).*q1_+cos(q1).*sin(q3).*q3_+cos(q1).*cos(q2).*sin(q3).*q1_+cos(q2).*cos(q3).*sin(q1).*q3_-sin(q1).*sin(q2).*sin(q3).*q2_)+cos(q5).*(cos(q1).*cos(q3)-cos(q2).*sin(q1).*sin(q3)).*q5_-sin(q5).*(cos(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1))+sin(q1).*sin(q2).*sin(q4)).*q5_).*(1.23e+2./5.0e+2)+cos(q6).*(sin(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1))-cos(q4).*sin(q1).*sin(q2)).*q6_.*(1.23e+2./5.0e+2)-sin(q6).*(sin(q5).*(cos(q1).*cos(q3)-cos(q2).*sin(q1).*sin(q3))+cos(q5).*(cos(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1))+sin(q1).*sin(q2).*sin(q4))).*q6_.*(1.23e+2./5.0e+2),cos(q6).*(cos(q5).*(cos(q2).*cos(q4).*q4_-sin(q2).*sin(q4).*q2_+cos(q4).*sin(q2).*sin(q3).*q3_+cos(q3).*sin(q2).*sin(q4).*q4_-cos(q2).*cos(q3).*cos(q4).*q2_)-sin(q5).*(cos(q2).*sin(q4)-cos(q3).*cos(q4).*sin(q2)).*q5_+cos(q2).*sin(q3).*sin(q5).*q2_+cos(q3).*sin(q2).*sin(q5).*q3_+cos(q5).*sin(q2).*sin(q3).*q5_).*(1.23e+2./5.0e+2)+sin(q6).*(cos(q4).*sin(q2).*q2_+cos(q2).*sin(q4).*q4_-cos(q2).*cos(q3).*sin(q4).*q2_-cos(q3).*cos(q4).*sin(q2).*q4_+sin(q2).*sin(q3).*sin(q4).*q3_).*(1.23e+2./5.0e+2)-cos(q6).*(cos(q2).*cos(q4)+cos(q3).*sin(q2).*sin(q4)).*q6_.*(1.23e+2./5.0e+2)-sin(q6).*(cos(q5).*(cos(q2).*sin(q4)-cos(q3).*cos(q4).*sin(q2))+sin(q2).*sin(q3).*sin(q5)).*q6_.*(1.23e+2./5.0e+2),cos(q5).*(-cos(q1).*cos(q3).*q1_+sin(q1).*sin(q3).*q3_+cos(q2).*sin(q1).*sin(q3).*q1_+cos(q1).*sin(q2).*sin(q3).*q2_-cos(q1).*cos(q2).*cos(q3).*q3_)-sin(q5).*(-cos(q4).*(cos(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*q3_+cos(q2).*cos(q3).*sin(q1).*q1_+cos(q1).*cos(q3).*sin(q2).*q2_+cos(q1).*cos(q2).*sin(q3).*q3_)+sin(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3)).*q4_+cos(q1).*cos(q2).*sin(q4).*q2_+cos(q1).*cos(q4).*sin(q2).*q4_-sin(q1).*sin(q2).*sin(q4).*q1_)+sin(q5).*(cos(q3).*sin(q1)+cos(q1).*cos(q2).*sin(q3)).*q5_+cos(q5).*(cos(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3))-cos(q1).*sin(q2).*sin(q4)).*q5_,-sin(q5).*(-cos(q4).*(-cos(q1).*cos(q3).*q3_+sin(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*sin(q2).*q2_+cos(q2).*sin(q1).*sin(q3).*q3_-cos(q1).*cos(q2).*cos(q3).*q1_)-sin(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1)).*q4_+cos(q1).*sin(q2).*sin(q4).*q1_+cos(q2).*sin(q1).*sin(q4).*q2_+cos(q4).*sin(q1).*sin(q2).*q4_)-cos(q5).*(cos(q3).*sin(q1).*q1_+cos(q1).*sin(q3).*q3_+cos(q1).*cos(q2).*sin(q3).*q1_+cos(q2).*cos(q3).*sin(q1).*q3_-sin(q1).*sin(q2).*sin(q3).*q2_)-sin(q5).*(cos(q1).*cos(q3)-cos(q2).*sin(q1).*sin(q3)).*q5_-cos(q5).*(cos(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1))+sin(q1).*sin(q2).*sin(q4)).*q5_,-sin(q5).*(cos(q2).*cos(q4).*q4_-sin(q2).*sin(q4).*q2_+cos(q4).*sin(q2).*sin(q3).*q3_+cos(q3).*sin(q2).*sin(q4).*q4_-cos(q2).*cos(q3).*cos(q4).*q2_)-cos(q5).*(cos(q2).*sin(q4)-cos(q3).*cos(q4).*sin(q2)).*q5_+cos(q2).*cos(q5).*sin(q3).*q2_+cos(q3).*cos(q5).*sin(q2).*q3_-sin(q2).*sin(q3).*sin(q5).*q5_,0.0,0.0,0.0,sin(q6).*(sin(q5).*(-cos(q1).*cos(q3).*q1_+sin(q1).*sin(q3).*q3_+cos(q2).*sin(q1).*sin(q3).*q1_+cos(q1).*sin(q2).*sin(q3).*q2_-cos(q1).*cos(q2).*cos(q3).*q3_)+cos(q5).*(-cos(q4).*(cos(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*q3_+cos(q2).*cos(q3).*sin(q1).*q1_+cos(q1).*cos(q3).*sin(q2).*q2_+cos(q1).*cos(q2).*sin(q3).*q3_)+sin(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3)).*q4_+cos(q1).*cos(q2).*sin(q4).*q2_+cos(q1).*cos(q4).*sin(q2).*q4_-sin(q1).*sin(q2).*sin(q4).*q1_)-cos(q5).*(cos(q3).*sin(q1)+cos(q1).*cos(q2).*sin(q3)).*q5_+sin(q5).*(cos(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3))-cos(q1).*sin(q2).*sin(q4)).*q5_)+cos(q6).*(sin(q4).*(cos(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*q3_+cos(q2).*cos(q3).*sin(q1).*q1_+cos(q1).*cos(q3).*sin(q2).*q2_+cos(q1).*cos(q2).*sin(q3).*q3_)+cos(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3)).*q4_-cos(q4).*sin(q1).*sin(q2).*q1_-cos(q1).*sin(q2).*sin(q4).*q4_+cos(q1).*cos(q2).*cos(q4).*q2_)-sin(q6).*(sin(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3))+cos(q1).*cos(q4).*sin(q2)).*q6_-cos(q6).*(sin(q5).*(cos(q3).*sin(q1)+cos(q1).*cos(q2).*sin(q3))+cos(q5).*(cos(q4).*(sin(q1).*sin(q3)-cos(q1).*cos(q2).*cos(q3))-cos(q1).*sin(q2).*sin(q4))).*q6_,sin(q6).*(cos(q5).*(-cos(q4).*(-cos(q1).*cos(q3).*q3_+sin(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*sin(q2).*q2_+cos(q2).*sin(q1).*sin(q3).*q3_-cos(q1).*cos(q2).*cos(q3).*q1_)-sin(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1)).*q4_+cos(q1).*sin(q2).*sin(q4).*q1_+cos(q2).*sin(q1).*sin(q4).*q2_+cos(q4).*sin(q1).*sin(q2).*q4_)-sin(q5).*(cos(q3).*sin(q1).*q1_+cos(q1).*sin(q3).*q3_+cos(q1).*cos(q2).*sin(q3).*q1_+cos(q2).*cos(q3).*sin(q1).*q3_-sin(q1).*sin(q2).*sin(q3).*q2_)+cos(q5).*(cos(q1).*cos(q3)-cos(q2).*sin(q1).*sin(q3)).*q5_-sin(q5).*(cos(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1))+sin(q1).*sin(q2).*sin(q4)).*q5_)+cos(q6).*(sin(q4).*(-cos(q1).*cos(q3).*q3_+sin(q1).*sin(q3).*q1_+cos(q3).*sin(q1).*sin(q2).*q2_+cos(q2).*sin(q1).*sin(q3).*q3_-cos(q1).*cos(q2).*cos(q3).*q1_)-cos(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1)).*q4_+cos(q1).*cos(q4).*sin(q2).*q1_+cos(q2).*cos(q4).*sin(q1).*q2_-sin(q1).*sin(q2).*sin(q4).*q4_)+sin(q6).*(sin(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1))-cos(q4).*sin(q1).*sin(q2)).*q6_+cos(q6).*(sin(q5).*(cos(q1).*cos(q3)-cos(q2).*sin(q1).*sin(q3))+cos(q5).*(cos(q4).*(cos(q1).*sin(q3)+cos(q2).*cos(q3).*sin(q1))+sin(q1).*sin(q2).*sin(q4))).*q6_,sin(q6).*(cos(q5).*(cos(q2).*cos(q4).*q4_-sin(q2).*sin(q4).*q2_+cos(q4).*sin(q2).*sin(q3).*q3_+cos(q3).*sin(q2).*sin(q4).*q4_-cos(q2).*cos(q3).*cos(q4).*q2_)-sin(q5).*(cos(q2).*sin(q4)-cos(q3).*cos(q4).*sin(q2)).*q5_+cos(q2).*sin(q3).*sin(q5).*q2_+cos(q3).*sin(q2).*sin(q5).*q3_+cos(q5).*sin(q2).*sin(q3).*q5_)-cos(q6).*(cos(q4).*sin(q2).*q2_+cos(q2).*sin(q4).*q4_-cos(q2).*cos(q3).*sin(q4).*q2_-cos(q3).*cos(q4).*sin(q2).*q4_+sin(q2).*sin(q3).*sin(q4).*q3_)-sin(q6).*(cos(q2).*cos(q4)+cos(q3).*sin(q2).*sin(q4)).*q6_+cos(q6).*(cos(q5).*(cos(q2).*sin(q4)-cos(q3).*cos(q4).*sin(q2))+sin(q2).*sin(q3).*sin(q5)).*q6_],[6,7]);