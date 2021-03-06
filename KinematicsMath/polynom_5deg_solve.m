%% 
syms t a b c d e f5
syms ti tf
T0= 1 ;
Tf =2;
% 
% q0 = 1;
% q0_=0.5;
% qf=2;
% qf_=1;
% d = q0;
% c = q0_;
% b = 3*(qf-q0)./T.^2 - 2*q0_./T - qf_./T;
% a = -2*(qf-q0)./T.^3 + (qf_ + q0_)./T.^2;

f5(t) = a*t^5+b*t^4+c*t^3+d*t^2+e*t+f5;
f5_d = diff(f5);
td=solve(f5_d);
theta_max1 = simplify(subs(f5,td(1)));
theta_max2 = simplify(subs(f5,td(2)));
theta_max3 = simplify(subs(f5,td(3)));
theta_max4 = simplify(subs(f5,td(4)));
theta_i = simplify(subs(f5,ti)) ;
theta_f = simplify(subs(f5,tf)) ;


f5_dd = diff(f5_d);
tdd=solve(f5_dd); %-b/(3*a)
theta_d_max = simplify(subs(f5_d,tdd));


theta_d_i = simplify(subs(f5_d,ti)) ;
theta_d_f = simplify(subs(f5_d,tf)) ;

theta_dd_i = subs(f5_dd,ti);
theta_dd_f = subs(f5_dd,tf);
%%  3th Deg
Ti = 1;
Tf = 1.8;
Qi =  -0.6;
Qf =  0;
Qdi = -1;
Qdf = 0;
Qddi= 12;
Qddf = -6;

%%  3th Deg
            T =[[1  Ti  Ti^2    Ti^3   ];
                [0  1   2*Ti    3*Ti^2 ];
                [1  Tf  Tf^2    Tf^3   ];
                [0  1   2*Tf    3*Tf^2 ]];
            
            %Qd 6x7
            Q = [Qi';
                Qdi';
                Qf';
                Qdf'];
            
            traj_3_Cs = (T\Q)';  %each row is for a different joint 
            traj_Cs = [traj_3_Cs 0 0];
%         end




%% 5th Deg

            T =[[1  Ti  Ti^2    Ti^3    Ti^4    Ti^5];
                [0  1   2*Ti    3*Ti^2  4*Ti^3  5*Ti^4  ];
                [0  0   2       6*Ti    12*Ti^2 20*Ti^3 ];
                [1  Tf  Tf^2    Tf^3    Tf^4    Tf^5    ];
                [0  1   2*Tf    3*Tf^2  4*Tf^3  5*Tf^4  ];
                [0  0   2       6*Tf    12*Tf^2 20*Tf^3 ]];

            Q = [Qi';
                Qdi';
                Qddi';
                Qf';
                Qdf';
                Qddf'];
            
            traj_Cs5 = (T\Q)';  %each row is for a different joint 
%%
%% find 3D
t = Ti:0.01:Tf;
[qdd3,qd3,q3] =  traj_5_order1(t',traj_Cs,length(t));
% plot(t,q)
hold all
plot(t,qd,'LineWidth',1.2,'DisplayName','theta-d_3')
% plot(t,qdd)
legend 
%%
%% find 5D
hold on
t = Ti:0.01:Tf;
[qdd5,qd5,q5] =  traj_5_order1(t',traj_Cs5,length(t));
% plot(t,q)
hold all
legend
plot(t,qd5,'LineWidth',1.2,'DisplayName','theta-d_5')
% plot(t,qdd)

%%
ax = tiledlayout(3,1);
ax1 = nexttile;
hold all
plot(t,q3,'--','LineWidth',1.2,'DisplayName','q_3')
plot(t,q5,'LineWidth',1.2,'DisplayName','q_5')
legend
ax2 = nexttile;
hold all
plot(t,qd3,'--','LineWidth',1.2,'DisplayName','qd_3')
plot(t,qd5,'LineWidth',1.2,'DisplayName','qd_5')
legend
ax3 = nexttile;
hold all
plot(t,qdd3,'--','LineWidth',1.2,'DisplayName','qdd_3')
plot(t,qdd5,'LineWidth',1.2,'DisplayName','qdd_5')
legend

% Add shared title and axis labels
% title(ax,'tt','FontSize',15)
xlabel(ax,'time')
ylabel(ax1,'q  rad')
ylabel(ax2,'qd  rad/sec')
ylabel(ax3,'qdd  rad/sec^2')
% Move plots closer together
xticklabels(ax1,{})
xticklabels(ax2,{})
ax.TileSpacing = 'compact';