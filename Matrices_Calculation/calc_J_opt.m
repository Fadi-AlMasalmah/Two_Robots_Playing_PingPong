function J = calc_J_opt(q1,q2,q3,q4,q5,q6)
%CALC_J_OPT
%    J = CALC_J_OPT(Q1,Q2,Q3,Q4,Q5,Q6)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    18-Mar-2020 21:34:04

t2 = cos(q1);
t3 = cos(q2);
t4 = cos(q3);
t5 = cos(q4);
t6 = cos(q5);
t7 = cos(q6);
t8 = sin(q1);
t9 = sin(q2);
t10 = sin(q3);
t11 = sin(q4);
t12 = sin(q5);
t13 = sin(q6);
t14 = t2.*t4;
t15 = t3.*t5;
t16 = t2.*t10;
t17 = t4.*t8;
t18 = t3.*t11;
t19 = t8.*t10;
t20 = t3.*2.0e+2;
t21 = t8.*t9.*t11;
t22 = t9.*t10.*t12;
t26 = t2.*t5.*t9;
t27 = t4.*t5.*t9;
t29 = t2.*t9.*t11;
t30 = t5.*t8.*t9;
t31 = t4.*t9.*t11;
t32 = t6.*t9.*t10;
t23 = t3.*t14;
t24 = t3.*t16;
t25 = t3.*t17;
t28 = t3.*t19;
t33 = t15.*2.0e+2;
t35 = -t27;
t37 = -t29;
t38 = -t30;
t39 = t7.*t15.*1.23e+2;
t40 = t31.*2.0e+2;
t41 = t7.*t31.*1.23e+2;
t42 = t6.*t13.*t18.*1.23e+2;
t43 = t13.*t22.*1.23e+2;
t46 = t15+t31;
t47 = t6.*t13.*t27.*1.23e+2;
t34 = -t23;
t36 = -t28;
t44 = t16+t25;
t45 = t17+t24;
t50 = t18+t35;
t51 = -t47;
t48 = t14+t36;
t49 = t19+t34;
t52 = t5.*t44;
t53 = t6.*t45;
t54 = t11.*t44;
t55 = t12.*t45;
t59 = t6.*t50;
t62 = t12.*t50;
t75 = t20+t33+t39+t40+t41+t42+t43+t51;
t56 = t5.*t49;
t57 = t6.*t48;
t58 = -t53;
t60 = t11.*t49;
t61 = t12.*t48;
t63 = t21+t52;
t64 = t38+t54;
t65 = t22+t59;
t66 = t26+t60;
t67 = t6.*t63;
t68 = t12.*t63;
t69 = t37+t56;
t70 = -t6.*(t29-t56);
t71 = -t12.*(t29-t56);
t72 = t61+t67;
t73 = t55+t70;
t74 = t58+t71;
J = reshape([t30.*(-2.0./5.0)+t54.*(2.0./5.0)-t8.*t9.*(2.0./5.0)-t13.*t72.*(1.23e+2./5.0e+2)-t7.*(t30-t54).*(1.23e+2./5.0e+2),t26.*(2.0./5.0)+t60.*(2.0./5.0)+t2.*t9.*(2.0./5.0)+t7.*t66.*(1.23e+2./5.0e+2)-t13.*t73.*(1.23e+2./5.0e+2),0.0,0.0,0.0,1.0,(t2.*t75)./5.0e+2,(t8.*t75)./5.0e+2,t9.*(-2.0./5.0)-t5.*t9.*(2.0./5.0)+t4.*t18.*(2.0./5.0)-t13.*(t6.*(t4.*t15+t9.*t11)-t3.*t10.*t12).*(1.23e+2./5.0e+2)-t7.*(t5.*t9-t4.*t18).*(1.23e+2./5.0e+2),-t8,t2,0.0,t11.*t45.*(2.0./5.0)-t13.*(t5.*t53-t12.*t49).*(1.23e+2./5.0e+2)+t7.*t11.*t45.*(1.23e+2./5.0e+2),t11.*t48.*(-2.0./5.0)-t13.*(t12.*t44-t5.*t57).*(1.23e+2./5.0e+2)-t7.*t11.*t48.*(1.23e+2./5.0e+2),t9.*(t10.*t11.*2.0e+2+t7.*t10.*t11.*1.23e+2-t4.*t12.*t13.*1.23e+2-t5.*t6.*t10.*t13.*1.23e+2).*(-1.0./5.0e+2),t2.*t9,t8.*t9,t3,t29.*(-2.0./5.0)+t56.*(2.0./5.0)-t7.*(t29-t56).*(1.23e+2./5.0e+2)+t6.*t13.*t66.*(1.23e+2./5.0e+2),t21.*(-2.0./5.0)-t52.*(2.0./5.0)-t7.*t63.*(1.23e+2./5.0e+2)+t6.*t13.*(t30-t54).*(1.23e+2./5.0e+2),t18.*(-2.0./5.0)+t27.*(2.0./5.0)-t7.*t50.*(1.23e+2./5.0e+2)+t6.*t13.*t46.*(1.23e+2./5.0e+2),t45,-t14+t28,-t9.*t10,t13.*(t53+t12.*(t29-t56)).*(-1.23e+2./5.0e+2),t13.*(t57-t68).*(1.23e+2./5.0e+2),t13.*(t32-t62).*(1.23e+2./5.0e+2),t66,t30-t54,t46,t13.*t66.*(-1.23e+2./5.0e+2)-t7.*t73.*(1.23e+2./5.0e+2),t7.*t72.*(1.23e+2./5.0e+2)-t13.*(t30-t54).*(1.23e+2./5.0e+2),t13.*t46.*(-1.23e+2./5.0e+2)+t7.*t65.*(1.23e+2./5.0e+2),t74,t57-t68,t32-t62,0.0,0.0,0.0,t7.*t66-t13.*t73,t13.*t72+t7.*(t30-t54),t7.*t46+t13.*t65],[6,7]);