function  [Reward1,Reward2,BallState,terminate] = FindBallState(ballState,Table,RobotDistanceToTable,P,V,lastV)
    %UNTITLED3 Summary of this function goes here
    %   Detailed explanation goes here
    zThresh = 0.35; % height to consider that the ball was on the table
%     P(1) = P(1) - Table.Dimensions(1)/2- RobotDistanceToTable(1); %convert to coordinate of the table center

    xMax = 2.2;
    yMax = 1.5;
    zMax = 5;

    OUT = false;
    T1 = false;
    T2 = false;
    H1 = false;
    H2 = false;

    if(any(abs(P)>[xMax,yMax,zMax]))
        OUT = true;
    end
    if(V(3)>0 & lastV(3)<0 &(sign(V(1))==sign(lastV(1)))&P(3)<zThresh& P(1)<0)
        T2 = true;
    end
    
    if(V(3)>0 & lastV(3)<0 &(sign(V(1))==sign(lastV(1))) &P(3)<zThresh &  P(1)>0)
        T1 = true;
    end
    
    if(V(1)>0 & lastV(1)<0 & P(1)<-0.4 & P(3)> zThresh/2)
        H2 = true;
    end
    if(V(1)<0 & lastV(1)>0 & P(1)>+0.4 & P(3)> zThresh/2)
        H1 = true;
    end
    
    switch ballState
        case 'H1' 
            if(T2)
                ballState = 'H1+';
            elseif(OUT|T1)
                ballState = 'Win2';
            elseif(H2)
                ballState = 'Win1';
            end
        case 'H1+' 
            if(H2)
                ballState = 'H2';
            elseif(OUT|T2)
                ballState = 'Win1';
            end
        case 'H2' 
            if(T1)
                ballState = 'H2+';
            elseif(OUT|T2)
                ballState = 'Win1';
            elseif(H1)
                ballState = 'Win2';
            end
        case 'H2+' 
            if(H1)
                ballState = 'H1';
            elseif(OUT|T1)
                ballState = 'Win2';
            end    
    end
    Reward1 = 0;
    Reward2 = 0;
    terminate = false;

    if(strcmp(ballState,'Win2'))
        Reward2 = 1;
        terminate = true;
        set_param('RobotAndBall','SimulationCommand','Stop')
    elseif(strcmp(ballState,'Win1'))
        terminate = true;
        set_param('RobotAndBall','SimulationCommand','Stop')
    end
    if(T1)
        disp('it is T1')
    end
    if(T2)
        disp('it is T2')
    end
    BallState = ballState;
end

