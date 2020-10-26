%% Environment
g=9.81;

%% Traj
Tr_field1 ='Qi';                 Tr_value1=[0 0 0 0 0 0 0];
Tr_field2 ='Qf';                 Tr_value2=[0 0 0 0 0 0 0];
tTraj = struct(Tr_field1,Tr_value1,Tr_field2,Tr_value2);
% qTraj.Qi    = obj.initialQ; 
%             .Qf    = obj.initialQ; 
%             .Qdi   = zeros(7,1); 
%             .Qdf   = zeros(7,1); 
%             .Qddi  = zeros(7,1); 
%             .Qddf  = zeros(7,1); 
%             .Ti    = 0;
%             .Ti    = 0;
%             .Cs
%% Table

T_field1='Dimensions';   T_value1=[2.74 1.525 0.06 1000];  % Length, Width, Thickness, Density 
T_field2='Height';       T_value2=0.76;
T_field3='COR';          T_value3= [0.88;0.88;-0.98];%[0.7684;0.7684;-0.95];  %Coefficient of restitution
Table=struct(T_field1,T_value1,T_field2,T_value2,T_field3,T_value3);

clear T_field1 T_value1 T_field2 T_value2 T_field3 T_value3;

%% Robot1
% R1_field11='Distance_Table';    R1_value11=[0.3 0 0];  % x,y,z
% R1_field12='initialQ';    R1_value12=[0 pi/4 0 -pi/4 0 pi/4 0]';  
% Robot1=struct(R1_field11,R1_value11);

%% Ball

B_field1='Radius';              B_value1=0.02;
B_field2='Volume';              B_value2=4/3*pi*B_value1^3;
B_field3='Density';             B_value3=2.7/(1000*B_value2);
B_field4='Initial_Position';    B_value4=[1 -0.5 1.3]; % x,y,z
B_field5='Initial_Velocity';    B_value5=[-3 0 1]; % vx, vy, vz
B_field6='State';               B_value6='H1';
% B_field6='Positions';           B_value6=zeros(200,3); %previous trajectory points
% B_field7='Velocities';           B_value7=zeros(200,3); %previous trajectory points velocities
% B_field8='N';                   B_value8=0; %Number of previous points (Length of Positions)
Ball=struct(B_field1,B_value1,B_field2,B_value2,B_field3,B_value3,...
    B_field4,B_value4,B_field5,B_value5,B_field6,B_value6);

clear B_field1 B_value1 B_field2 B_value2 B_field3 B_value3 ...
    B_field4 B_value4 B_field5 B_value5 B_field6 B_value6

%% Paddle
% P_field1='Radius';           P_value1=0.075;
% P_field2='thickness';        P_value2=0.015;
% P_field3='density';          P_value3=100;
% Paddle = struct(P_field1,P_value1,P_field2,P_value2,P_field3,P_value3);
% clear P_field1 P_value1 P_field2 P_value2 P_field3 P_value3

R2_field9='Paddle';             R2_value9=[0.0775 0.03 100]; % r h rho
R2_field11='Distance_Table';    R2_value11=[0.3 0 0];          % x,y,z
% R2_field13='SetPaddlePosition'; R2_value13=[0.1 0.1 1 10 10];
Robots=struct(R2_field9,R2_value9,R2_field11,R2_value11);
clear R2_field9 R2_value9 R2_field11 R2_value11
%% Ball_Trajectory
BT_field1='Positions';           BT_value1=zeros(200,3); %previous trajectory points
BT_field2='Velocities';          BT_value2=zeros(200,3); %previous trajectory points velocities
BT_field3='moments';             BT_value3=zeros(200,1); %previous trajectory points
BT_field4='N';                   BT_value4=0; %Number of previous points (Length of Positions)
BT_field5='Predicted_P';         BT_value5=zeros(200,3); %predicted trajectory points after the N points
BT_field6='Predicted_V';         BT_value6=zeros(200,3); %predicted trajectory points velocities after the N points
BT_field7='dt';                  BT_value7=0.02; % The time step of the predicted trajectory
Ball_Traj=struct(BT_field1,BT_value1,BT_field2,BT_value2,BT_field3,BT_value3,...
    BT_field4,BT_value4,BT_field5,BT_value5,BT_field6,BT_value6,BT_field7,BT_value7);
clear BT_field1 BT_value1 BT_field2 BT_value2 BT_field3 BT_value3 ...
    BT_field4 BT_value4 BT_field5 BT_value5 BT_field6 BT_value6 BT_field7 BT_value7

%% Net

N_field1='Dimensions';  N_value1=[1.525, 0.001, 0.1525, 1000]; %Length, Width, Height, Density
Net=struct(N_field1,N_value1);

clear N_field1 N_value1

 %% PMDC
% 
% PMDC_field1='J';            PMDC_value1=1.55*10^(-3); % Rotor Inertia                   kg*m^2 
% PMDC_field2='B';            PMDC_value2=0.03; % Short circuit damping (viscous friction)N*m*s/rad 
% PMDC_field3='kt';           PMDC_value3=0.067;  % Torque constant                       N*m/A
% PMDC_field4='kb';           PMDC_value4=PMDC_value3; %  Back EMF constant               N*m/A
% PMDC_field5='Ra';           PMDC_value5=0.03; % Armature Resistance                     Ohm
% PMDC_field6='La';           PMDC_value6=0.1*10^(-3); % Armature Inductance              Henry
% PMDC_field7='Voltage_Range';PMDC_value7=24;
% PMDC_field8='Gear_Ratio';   PMDC_value8=1/2;
% PMDC=struct(PMDC_field1,PMDC_value1,PMDC_field2,PMDC_value2,PMDC_field3,PMDC_value3,...
%     PMDC_field4,PMDC_value4,PMDC_field5,PMDC_value5,PMDC_field6,PMDC_value6,...
%     PMDC_field7,PMDC_value7,PMDC_field8,PMDC_value8);
% 
% clear PMDC_field1 PMDC_value1 PMDC_field2 PMDC_value2 PMDC_field3 PMDC_value3...
%       PMDC_field4 PMDC_value4 PMDC_field5 PMDC_value5 PMDC_field6 PMDC_value6... 
%       PMDC_field7 PMDC_value7 PMDC_field8 PMDC_value8

%% Contact Forces

CF_field1='Ball_Table';         CF_value1=[1e10 1e2];     % Contact Stiffness, Contact Damping
CF_field2='Ball_Net';           CF_value2=[1e7  1e12];  
CF_field3='Ball_Paddle';        CF_value3=[1e8 1e3];
Contact_Forces=struct(CF_field1,CF_value1,CF_field2,CF_value2,CF_field3,CF_value3);

clear CF_field1 CF_value1 CF_field2 CF_value2 CF_field3 CF_value3
%% Environment constants % Online optimal trajectory generation for robot table tennis.PDF APPENDIXES

E_field1='Table_Friction';         E_value1=0.22;%0.25;%0.102;     %
E_field2='Paddle_Friction';        E_value2=0.010;
E_field3='Table_COR';              E_value3=0.883;     % coefficient of restitution
E_field4='Paddle_COR';             E_value4=0.80;  
E_field5='AirDrag';                E_value5=0.1425;     % coefficient air drag Cd 1/m
E_field6='Magnus';                 E_value6=0.0114;% lift coefficient Cl 1/rad
Environment=struct(E_field1,E_value1,E_field2,E_value2,E_field3,E_value3,E_field4,E_value4,E_field5,E_value5,E_field6,E_value6);

clear E_field1 E_value1 E_field2 E_value2 E_field3 E_value3...
    E_field4 E_value4 E_field5 E_value5 E_field6 E_value6

%% Set Paddle Position

R1_SetPaddlePosition=[0.1 0.1 1 10 10];
R2_SetPaddlePosition=[0.1 0.1 1 10 10];

