function [P,V,W,currentTime,R1CurrentAngles,R1CurrentVelocity,R1CurrentAcceleration,R2CurrentAngles,R2CurrentVelocity,R2CurrentAcceleration,NetNormalForce]= ... 
    GetDataFromSimulation()
%function [P,V,currentTime,R1CurrentAngles,R1CurrentVelocity,R2CurrentAngles,R2CurrentVelocity, NetNormalForce]= ... 

% Px
Px=get_param('RobotAndBall/Px', 'RuntimeObject');
Px=Px.InputPort(1);
Px=Px.Data;

% Py
Py=get_param('RobotAndBall/Py', 'RuntimeObject');
Py=Py.InputPort(1);
Py=Py.Data;

% Pz
Pz=get_param('RobotAndBall/Pz', 'RuntimeObject');
Pz=Pz.InputPort(1);
Pz=Pz.Data;

P=[Px,Py,Pz];

% Vx
Vx=get_param('RobotAndBall/Vx', 'RuntimeObject');
Vx=Vx.InputPort(1);
Vx=Vx.Data;

% Vy
Vy=get_param('RobotAndBall/Vy', 'RuntimeObject');
Vy=Vy.InputPort(1);
Vy=Vy.Data;

% Vz
Vz=get_param('RobotAndBall/Vz', 'RuntimeObject');
Vz=Vz.InputPort(1);
Vz=Vz.Data;

V=[Vx,Vy,Vz];

% currentTime
time=get_param('RobotAndBall/currentTime', 'RuntimeObject');
time=time.InputPort(1);
currentTime=time.Data;
% 
% R1CurrentAngles
R1CA=get_param('RobotAndBall/R1CurrentAngles', 'RuntimeObject');
R1CA=R1CA.InputPort(1);
R1CurrentAngles=R1CA.Data;
% 
% R1CurrentVelocity
R1CV=get_param('RobotAndBall/R1CurrentVelocity', 'RuntimeObject');
R1CV=R1CV.InputPort(1);
R1CurrentVelocity=R1CV.Data;

% R1CurrentAcceleration
% R1CAcc=get_param('RobotAndBall/R1CurrentAcceleration', 'RuntimeObject');
% R1CAcc=R1CAcc.InputPort(1);
% R1CurrentAcceleration=R1CAcc.Data;
R1CurrentAcceleration = 0;
% % R2CurrentAngles
R2CA=get_param('RobotAndBall/R2CurrentAngles', 'RuntimeObject');
R2CA=R2CA.InputPort(1);
R2CurrentAngles=R2CA.Data;
% 
% % R2CurrentVelocity
R2CV=get_param('RobotAndBall/R2CurrentVelocity', 'RuntimeObject');
R2CV=R2CV.InputPort(1);
R2CurrentVelocity=R2CV.Data;
%Acceleration
R2CurrentAcceleration = 0;

Wx=get_param('RobotAndBall/Wx', 'RuntimeObject');
Wx=Wx.InputPort(1);
Wx=Wx.Data;
% 
Wy=get_param('RobotAndBall/Wy', 'RuntimeObject');
Wy=Wy.InputPort(1);
Wy=Wy.Data;

Wz=get_param('RobotAndBall/Wz', 'RuntimeObject');
Wz=Wz.InputPort(1);
Wz=Wz.Data;

W = [Wx,Wy,Wz];
% NetNormalForce
NetNormalForce=get_param('RobotAndBall/NetNormalForce', 'RuntimeObject');
NetNormalForce=NetNormalForce.InputPort(1);
NetNormalForce=NetNormalForce.Data;
end