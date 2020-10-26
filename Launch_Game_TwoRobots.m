slCharacterEncoding('ISO-8859-1')
%%
addpath('Matrices_Calculation')
addpath('meshes\iiwa7\visual')
addpath('KinematicsMath')
addpath('GameFunctions')
THEWHOLETIME = tic;
global PLOT
PLOT=0;
global NeedDbg
NeedDbg = 0;
% fixHitplaneX = 0.4;
global MyIk
MyIk = 1;
global checkLimitsAndCollision
checkLimitsAndCollision = 1;

NumOfGames = 1;

BALLSTATE = strings(NumOfGames,1);
PLANE = zeros(NumOfGames,1);
NumLeftHit = 0;
NumSuccessfulReturn = 0;
NotValidShot = 0;
LeftWins = 0;
RightWins = 0;

pOpponentSidePoint = [2.4;0;0.02]'; %[2 ; -0.6 ; BallRadius] + [ 1 ; 1.3 ; 0 ].*rand(3,1);
zAboveNet = 0.45;
AvgRoundTime = 0;

%% FOR  %%%%%%%%%%%%%%%%%
for GameId = 1:NumOfGames
    GameId
    GameTime = tic;
    Repeat = 0;      %% REPEAT
    
    initialQ = [0 pi/4 0 -pi/4 0 pi/2 0]';
    PingPongEnvironment
    bTraj = BallTraj2(Table,Ball,Environment);
    
    % -------Robot1--------
    %Transformation Matrix form world to Robot(inverse of the usual)
    TransMat1 = eye(4);
    DistToTable = [0.3,0,0];
    TransMat1(1:3,4) = [Table.Dimensions(1)/2+DistToTable(1),0,0]';
    Robot1 = RobotPlayer2(initialQ,TransMat1,DistToTable,'Left');
    
    Cs1 = [initialQ,zeros(7,5)];
    Cs1_reshaped = reshape(Cs1,1,42);
    
    % -------Robot2--------
    TransMat2 = zeros(4,4);
    TransMat2(1:3,1:3) = rotz(pi);
    TransMat2(1:3,4) = [Table.Dimensions(1)/2+DistToTable(1),0,0]';
    Robot2 = RobotPlayer2(initialQ,TransMat2,DistToTable,'Right');
    
    Cs2 = [initialQ,zeros(7,5)];
    Cs2_reshaped = reshape(Cs2,1,42);
    
    %----------------------
    minYAxis = -1.5;
    maxYAxis = 1.5;
    MaxNormQd = 0;
    MaxQdV = 0;
    maxTravel = 0;
    maxAcc = 0;
    maxNormTravel = 0;
    maxNormAcc = 0;
    % Set the update frequency of the Simulink model
    updateFrequency=0.01;
    NetNormalForce=0;
    % Open the Simulink Model
    
    if (~bdIsLoaded('RobotAndBall')) || strcmp(get_param('RobotAndBall','Shown'),'off')
        open_system('RobotAndBall')
    end
    set_param('RobotAndBall/Cs1_vector', 'Value','Cs1_reshaped');
    set_param('RobotAndBall/timeToHit', 'Value','10000');
    set_param('RobotAndBall/Cs2_vector', 'Value','Cs2_reshaped');
    set_param('RobotAndBall/timeToHitR2', 'Value','10000');
    %%%%%%% Preparation %%%%%%%%%%%
    % Get random start parameters of the Ball % Ball starts on right side
    [P,V]=RespawnBall('right',Table,Robot1.DistToTable); %%%%%%
    Ball.Initial_Position=P;
    Ball.Initial_Velocity=V;
    initaialSpin = [10,-0,-0];
    if(Repeat == 1)
        Ball.Initial_Position=Pinit;
        Ball.Initial_Velocity=Vinit;
    else
        Vinit = Ball.Initial_Velocity;
        Pinit = Ball.Initial_Position;
    end
    InitialBall=true;
    
    bTraj.updateWithAir(P,V,[0,0,0],0);
    
    %% %%%%%%%%  Start the Game %%%%%%%%%%%
    set_param('RobotAndBall', 'PacingRate', 1)
    set_param('RobotAndBall', 'EnablePacing', 'on')
    %set_param('RobotAndBall','SimulationMode','Accelerator')
    set_param('RobotAndBall','SimulationCommand','start')
    
    i=0;
    draw = 0;
    previousTime = 0;
    hitPointId = -1;
    tRealHit = 0;
    
    l = 1;
    Points = zeros(100,3);
    Velocities= zeros(100,3);
    tStamps = zeros(100,1);
    tHits = zeros(100,1);
    pHits = zeros(100,3);
    tStart = tic;
    lastV = V;
    LeftHit =  false;
    SuccessfulReturn = false;
    ValidShot = false;
    pRealHit = [0,0,0];
    
    P1 = zeros(350,3);
    Pr = zeros(350,3);
    T1 = zeros(350,1);
    id = 0;
    R = 1; % robot1 turn
    omega = [0,0,0]; %zero angular velocity 
    
    %% While the Game is still going
    while ((strcmp(get_param('RobotAndBall','SimulationStatus'),('running'))) || (strcmp(get_param('RobotAndBall','SimulationStatus'),('paused'))))
        % Get the Data from the simulation
        
        [P, V,W, currentTime,R1CurrentAngles,R1CurrentVelocity,R1CurrentAcceleration,R2CurrentAngles,R2CurrentVelocity,R2CurrentAcceleration,NetNormalForce] = GetDataFromSimulation();
        if(abs(currentTime-previousTime)<5e-3) %in order to give simulink time to process
            pause(0.0011);
            i=i+1;
            if(i>10000)
                disp('if it is stuck, you might check wether NeedDbg is == 1')
            end
            continue;
        end
        previousTime = currentTime;
        
        %make sure that game has started
        while(InitialBall && V(1)==0)
            pause(0.02)
            [P, V, W,currentTime,R1CurrentAngles,R1CurrentVelocity,R1CurrentAcceleration,R2CurrentAngles,R2CurrentVelocity,R2CurrentAcceleration,NetNormalForce] = GetDataFromSimulation();
            disp('V(1)=0')
        end
        
        id=id+1;
        P1(id,:) = P;
        %V1(id,:) = V;
        FK = calc_T07(R1CurrentAngles);
        Pr(id,:) = FK(1:3,4);
        T1(id) = currentTime;
        InitialBall=false;
        pOpponentError = [100,100,100];
        [Reward1,Reward2,Ball.State,terminate] = FindBallState(Ball.State,Table,Robot1.DistToTable,P,V,lastV);
        %% Discuss the ball state for
        if strcmp(Ball.State,'H2')
            LeftHit =  true;
        end
        
        if strcmp(Ball.State,'H2+')
            if(MaxQdV > 12)
               % NUMUNSTABLE = NUMUNSTABLE+1;
            else
                SuccessfulReturn = true;
                pOpponentError = abs(P-pOpponentSidePoint);
            end
        end
        if strcmp(Ball.State,'Win2')&(MaxQdV < 12)
            LeftWins=LeftWins+1;
            disp(['Left: ', num2str(LeftWins), ' Right: ', num2str(RightWins)])
        end
        if strcmp(Ball.State,'Win1')
            RightWins=RightWins+1;
            disp(['Left: ', num2str(LeftWins), ' Right: ', num2str(RightWins)])
        end
        if strcmp(Ball.State,'H1+')
            ValidShot = true;
        end
        
        if(abs(P(1) - Robot1.pHit(1))<0.025 && tRealHit < 0.01)
            pRealHit = P;
            tRealHit = currentTime;
        end
        %% ---- Predicting And Updating --------
        if(V(1)>0)
            R = 2; % Robot2 should be updated
        else
            R = 1;
        end
        
        if bTraj.state == 3 && omega(1) == 0
            omega = W;
        end
        [tHit,pHit,paddleOrientHit,paddleVHit,hitPointId] = bTraj.updateWithAir(P,V,W,currentTime);  %paddleVHit = [3x1]
        
        %% --------UPDATE ROBOT1-------------
        %     if (currentTime >= Robot1.tHit|| (hitPointId>-1)% && (errInP>pEps || errInTime>tEps))) %if Robot1.state==2
        [Cs1,Cs1Updated] = Robot1.updateCoeffitients(V,currentTime,tHit,pHit,...
            paddleOrientHit,[zeros(length(tHit),3) paddleVHit*1],...
            hitPointId,R1CurrentAngles,R1CurrentVelocity,R1CurrentAcceleration);
        if(Cs1Updated)
            Cs1_reshaped = reshape(Cs1,[1,42]);
            set_param('RobotAndBall/Cs1_vector', 'Value', ['[', num2str(Cs1_reshaped),']']);
            set_param('RobotAndBall/timeToHit', 'Value', ['[', num2str(tHit(Robot1.planeId,:)),']']);
            % set_param(bdroot,'SimulationCommand','update')
        end
        
        %% --------UPDATE ROBOT2-------------
        if(NeedDbg == 1)
            set_param('RobotAndBall','SimulationCommand','pause')
            x=1;
        end
        Wt = zeros(length(tHit),3);
        Wt(:,2) = -1;
        
        [Cs2,Cs2Updated] = Robot2.updateCoeffitients(V,currentTime,tHit,pHit,...
            paddleOrientHit,[Wt paddleVHit*1],...
            hitPointId,R2CurrentAngles,R2CurrentVelocity,R2CurrentAcceleration);
        
        if(Cs2Updated)
            Cs2_reshaped = reshape(Cs2,[1,42]);
            set_param('RobotAndBall/Cs2_vector', 'Value', ['[', num2str(Cs2_reshaped),']']);
            set_param('RobotAndBall/timeToHitR2', 'Value', ['[', num2str(tHit(Robot2.planeId,:)),']']);
            %             set_param(bdroot,'SimulationCommand','update')
            
        end
        
         %% Plot
        if(hitPointId>-1)
            hold off
            n = bTraj.N;
            plot3(bTraj.Predicted_P(:,1),bTraj.Predicted_P(:,2),bTraj.Predicted_P(:,3),'b-');
            hold on
            plot3(bTraj.Positions(1:n,1),bTraj.Positions(1:n,2),bTraj.Positions(1:n,3),'r-');
            plot3(P(1),P(2),P(3),'x')
            if(R==1)
                g = bTraj.GoalOnLeft;
            else
                g = bTraj.GoalOnRight;
            end
            plot3(g(1),g(2),g(3),'rx','LineWidth',4,'MarkerSize',4)
            planes = (bTraj.minHitplaneX:bTraj.HitPlaneDx:bTraj.minHitplaneX+bTraj.HitPlaneDx*bTraj.numOfPlanes);
            plot(planes,zeros(length(planes)),'c*','LineWidth',2,'MarkerSize',3)
            plot(-planes,zeros(length(planes)),'c*','LineWidth',2,'MarkerSize',3)
            if(V(1)<0)
                plot3(pHit(Robot1.planeId,1),pHit(Robot1.planeId,2),pHit(Robot1.planeId,3),'*','LineWidth',2,'MarkerSize',10)
            else
                plot3(pHit(Robot2.planeId,1),pHit(Robot2.planeId,2),pHit(Robot2.planeId,3),'*','LineWidth',2,'MarkerSize',10)
            end
            xlim([-2 2]);
            ylim([-0.75 0.75]);
            zlim([-0.2 2.5]);
            % axis('equal')
            view(7,10)
            grid on
        else
            hold off
        end
        
        lastV = V;
    end %WHILE
    
    %%
    if(ValidShot)
        if(LeftHit)
            NumLeftHit =  NumLeftHit + 1;
            if(strcmp(Ball.State,'H1+'))
                disp('something isnt right, BallState is H1+ although LeftHit is true ')
            end
        end
        if(strcmp(Ball.State,'H1+'))
            RightWins = RightWins + 1;
        end
        if(SuccessfulReturn)
            NumSuccessfulReturn = NumSuccessfulReturn + 1;
        end
    else
        NotValidShot = NotValidShot +1;
    end % if(ValidShot)
    % set_param('RobotAndBall', 'EnablePacing', 'off');
    %% Error calc
    pErr = (pHit(Robot1.planeId,:)-pRealHit)*100;
    tErr = tHit(Robot1.planeId)-tRealHit;
    totalTime = toc(tStart);
    OneRoundTime = toc(GameTime);
    AvgRoundTime = ((GameId-1)*AvgRoundTime + OneRoundTime)/GameId
    T = num2str(round(OneRoundTime));
    mxNQd = num2str(round(MaxNormQd,2));
    mxQd = num2str(round(MaxQdV,2));
    disp(['bState = ',Ball.State,'  |   T = ',T,'  |  pOpponentError = ',num2str(pOpponentError),'    |     MaxQdV = ',mxQd,'   |  MaxNormQd = ',mxNQd,'   |   Xplane = ',num2str(10*round(Robot1.pHit(1)*10))])
end %The big For (GameId)

missedShots = NumOfGames - NotValidShot - NumSuccessfulReturn;
% ReturnedButNotStable = NumOfGames - LeftWins - RightWins;
% disp(['NmuLeftHit = ',num2str(NmuLeftHit),'    |     SuccessfulReturn = ',num2str(NumSuccessfulReturn),'   |  NotValidShot = ',num2str(NotValidShot)])
TIME = toc(THEWHOLETIME);



