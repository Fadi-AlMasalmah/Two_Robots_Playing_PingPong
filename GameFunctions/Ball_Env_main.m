%  clear;
%  bdclose all
% slCharacterEncoding('ISO-8859-1')
%%
Ball_Env
bTraj = BallTraj(Table,Ball);
% Set the update frequency of the Simulink model
updateFrequency=0.01;
NetNormalForce=0;
% Open the Simulink Model
open_system('Table_Ball')
% Set Acceleration Mode (No video)
%  set_param('Pong3D','SimulationMode','Accelerator')
% Set FastRestart (No compiling between rounds)
% set_param('Pong3D','FastRestart','on')
% pause(5)

%Preparation
% Get random start parameters of the Ball 
% Ball starts on right side
[P,V]=RespawnBall('right', Table);
% P0=P;V0=V;
% These are all values which the Simulink model needs
Ball.Initial_Position=P;
Ball.Initial_Velocity=V;
InitialBall=true;
% LastP=P;
% LastV=V;
bTraj.update(P,V,0);
% Start the Game
set_param('Table_Ball','SimulationCommand','start')

%%
i=0;
draw = 0;
previousTime = 0;
% figure
while ((strcmp(get_param('Table_Ball','SimulationStatus'),('running'))) || (strcmp(get_param('Table_Ball','SimulationStatus'),('paused'))))
    tic
    % Get the Data from the simulation
    [P, V, currentTime,NetNormalForce] = GetDataFromSimulation();
   
    if(abs(currentTime-previousTime)<5e-3) %in order to give simulink time to process
         pause(0.0011);
         i=i+1;
         continue;
    end
    previousTime = currentTime;
    
    while(InitialBall && V(1)==0)
        pause(0.02)
        [P, V, currentTime,NetNormalForce] = GetDataFromSimulation();
        disp('V(1)=0')
    end
    InitialBall=false;
    bTraj.update(P,V,currentTime);
    if(bTraj.tHitTable > 0)
%         hold on
%         plot(bTraj.tHitTable,bTraj.pHitTable(1),'o')
%         hold on
%         plot(bTraj.tHitTable,bTraj.pHitTable(2),'x')
%         hold on
%         plot(bTraj.tHitTable,bTraj.vInHitTable(3),'o')
%         hold on
%         plot(bTraj.tHitTable,bTraj.vOutHitTable(3),'x')
    end
%%    % Check if the Ball is out of the Game Cube
%     [LeftReward, RightReward, winner, terminal] = ...
%         CheckBall(P, V, LastV, Table, Robot1, Robot2, NetNormalForce);
%     if terminal
%         disp('terminal')
%     else
%         if LastV(3)*V(3)<0 && V(3)>0
%             Ball_Traj.N=0;
%             i=i+1;
%             set_param('Table_Ball','SimulationCommand','pause')
%         end
%         Ball_Traj.N=Ball_Traj.N+1;
%         Ball_Traj.Positions(Ball_Traj.N,:) = P;
%         Ball_Traj.Velocities(Ball_Traj.N,:) = V;
%         Ball_Traj.moments(Ball_Traj.N) = currentTime;
%         if(Ball_Traj.N>10)
%             [fitz,Ball_Traj.Predicted_P] = Traj_Predict(Ball_Traj.moments,Ball_Traj.Positions,Ball_Traj.N);
%         end
%% tests
%         if bTraj.N == 15 && draw == 0
%           figure 
%           plot(bTraj.Predicted_t,bTraj.Predicted_P) ;
%             draw = 1;
%         end
        bTraj.N
%     end
% 
% LastP=P;
% LastV=V;
%   pause(0.0011);
% Ball_Traj.N
%%

end