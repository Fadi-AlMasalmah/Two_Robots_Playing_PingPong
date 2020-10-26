function G = calc_G(q2,q3,q4,q5,q6,q7)
%CALC_G
%    G = CALC_G(Q2,Q3,Q4,Q5,Q6,Q7)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    22-Feb-2020 01:11:25

t2 = cos(q2);
t3 = cos(q3);
t4 = cos(q4);
t5 = cos(q5);
t6 = cos(q6);
t7 = cos(q7);
t8 = sin(q2);
t9 = sin(q3);
t10 = sin(q4);
t11 = sin(q5);
t12 = sin(q6);
t13 = sin(q7);
t14 = t2.*t4;
t15 = t2.*t10;
t16 = t4.*t8;
t17 = t8.*t10;
t18 = t8.*t9.*t11;
t23 = t2.*t9.*t11;
t24 = t3.*t8.*t11;
t25 = t5.*t8.*t9;
t19 = t3.*t14;
t20 = t3.*t15;
t21 = t3.*t16;
t22 = t3.*t17;
t26 = t5.*t9.*t16;
t29 = -t23;
t30 = -t25;
t27 = -t20;
t28 = -t21;
t31 = t14+t22;
t32 = t17+t19;
t36 = t24+t26;
t33 = t15+t28;
t34 = t16+t27;
t35 = t5.*t32;
t37 = t5.*t33;
t38 = t11.*t33;
t40 = t29+t35;
t39 = t18+t37;
t41 = t30+t38;
G = [0.0;t2.*1.0237374e-2-t8.*6.062183744400001e+1-t16.*3.10156672e+1+t20.*3.10156672e+1-t23.*2.120034e-3+t35.*2.120034e-3-t7.*(t11.*t32+t2.*t5.*t9).*6.468e-2+t2.*t9.*3.226257999999986e-2-t6.*t34.*4.7594092-t11.*t32.*4.72803156e-1+t12.*(t23-t35).*4.7594092+t13.*(t12.*t34+t6.*(t23-t35)).*6.468e-2-t2.*t5.*t9.*4.72803156e-1;t24.*(-2.120034e-3)-t26.*2.120034e-3+t13.*(t6.*t36+t9.*t12.*t17).*6.468e-2+t3.*t8.*3.226257999999986e-2-t9.*t17.*3.10156672e+1+t12.*t36.*4.7594092-t7.*(t3.*t5.*t8-t9.*t11.*t16).*6.468e-2-t3.*t5.*t8.*4.72803156e-1-t6.*t9.*t17.*4.7594092+t9.*t11.*t16.*4.72803156e-1;t15.*(-3.10156672e+1)+t21.*3.10156672e+1+t13.*(t12.*t33+t5.*t6.*t31).*6.468e-2-t5.*t31.*2.120034e-3-t6.*t33.*4.7594092+t11.*t31.*4.72803156e-1+t5.*t12.*t31.*4.7594092+t7.*t11.*t31.*6.468e-2;t18.*4.72803156e-1-t25.*2.120034e-3+t37.*4.72803156e-1+t38.*2.120034e-3+t7.*t39.*6.468e-2+t12.*(t25-t38).*4.7594092+t6.*t13.*(t25-t38).*6.468e-2;t12.*t31.*(-4.7594092)+t6.*t39.*4.7594092-t13.*(t6.*t31+t12.*t39).*6.468e-2;t13.*(t25-t38).*6.468e-2-t7.*(t12.*t31-t6.*t39).*6.468e-2;0.0];