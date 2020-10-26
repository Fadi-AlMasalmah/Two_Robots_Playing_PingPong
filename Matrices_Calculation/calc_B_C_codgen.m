function BC = calc_B_C(q,q_)
%CALC_G
%    G = CALC_G(Q2,Q3,Q4,Q5,Q6,Q7)
BC = zeros(7,1);
q_1 = q_(1);
q_2 = q_(2);
q_3 = q_(3);
q_4 = q_(4);
q_5 = q_(5);
q_6 = q_(6);
q_7 = q_(7);
qq_ = [ q_1*q_2;
 q_1*q_3;
 q_1*q_4;
 q_1*q_5;
 q_1*q_6;
 q_1*q_7;
 q_2*q_3;
 q_2*q_4;
 q_2*q_5;
 q_2*q_6;
 q_2*q_7;
 q_3*q_4;
 q_3*q_5;
 q_3*q_6;
 q_3*q_7;
 q_4*q_5;
 q_4*q_6;
 q_4*q_7;
 q_5*q_6;
 q_5*q_7;
 q_6*q_7];
BC = calc_C1_opt(q(1),q(2),q(3),q(4),q(5),q(6))*(q_.*q_);
BC = BC + calc_B_opt(q(1),q(2),q(3),q(4),q(5),q(6))*qq_;