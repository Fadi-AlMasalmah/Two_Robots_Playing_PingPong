classdef BallTraj2<handle
%% Ball Trajectory predictor
% Two robots playing Version 1
    
    properties ( GetAccess = 'public', SetAccess = 'private' )
        g
        maxNpos         % the maximum number of points in trajectory  
        maxNpredPos     % the maximum number of points in the predicted trajectory  
        Positions       %previous trajectory points velocities
        Velocities      %previous trajectory points
        tStamps     %timeStamps of Positions and Velocities of the real Ball
        N               %Number of points at the moment (Length of Positions)
        Predicted_P     %predicted trajectory points after the N points
        Predicted_V     %predicted trajectory points velocities after the N points
        Predicted_t     %moments of the predicted trajectory
        minNpos         %the minimum number of points to start predictiong the trajectry
        tStep           %time step of the predicted trajectory
                   %takes value = 1,2,3,4 it describes the state of the trajectory (before bounding or after, and the direction) 
        tHitTable
        pHitTable
        vInHitTable
        vOutHitTable
        
        pOpponentSidePoint
        zAboveNet
        trigger         %logical 0 or 1 , tells the simulink that there is new point 
        tHit
        pHit
        paddleOrientHit
        paddleVHit
        Wout
        Env         %Environment coefficient :% kd (AirDrag) % km (Magnus) % Table_Friction
%         lastP           %the last position of the ball
%         lastV           %
        
        numOfPlanes = 9;
        minHitplaneX = 0.3 - 1.37 - 0.3 ; %Table.Dimensions(1)/2 + Robot.DistToTable = 1.37 + 0.3
        HitPlaneDx = 0.08;
        
        GoalOnLeft =  [-0.55;0.0;0.02];     %the left player will shoot towards this point
        ZnetToLeft = 0.45;
        
        GoalOnRight = [0.55;0.0;0.02];
        ZnetToRight = 0.45;
    end
    properties
        Table
        Ball
        hitPointId
        PredictEnd
        paddleCor  %coefficient of restituion
        fixHitplaneX = 0.5;
        state
    end
    methods
        function ballTraj=BallTraj2(Table,Ball,Env)
        %% Configure and set up the trjactory
            ballTraj.Table          =Table;
            ballTraj.Ball           =Ball;
            ballTraj.paddleCor      =Env.Paddle_COR;
%             ballTraj.kd             =Env.AirDrag;
%             ballTraj.km             =Env.Magnus;
%             ballTraj.Table_Friction =Env.Table_Friction;
            ballTraj.Env            = Env;
            ballTraj.PredictEnd     =false;
            ballTraj.g              =9.81;  %m/s^2
            ballTraj.maxNpos        = 120;
            ballTraj.maxNpredPos    = 100;
            ballTraj.minNpos        = 8;
            ballTraj.Positions      = zeros(ballTraj.maxNpos,3);
            ballTraj.Velocities     = zeros(ballTraj.maxNpos,3);
            ballTraj.tStamps        = zeros(ballTraj.maxNpos,1);
            ballTraj.N              = 0;
            ballTraj.tStep          = 0.01; %sec
            ballTraj.Predicted_P    = zeros(ballTraj.maxNpredPos,3);
            ballTraj.Predicted_V    = zeros(ballTraj.maxNpredPos,6);
            ballTraj.Predicted_t    = zeros(ballTraj.maxNpredPos,1);
            ballTraj.state          = 1;
            ballTraj.tHitTable      = -100;     %not known yet
            ballTraj.pHitTable      = [0,0,0];    %not known yet
            ballTraj.vInHitTable    = [0,0,0];    %not known yet
            ballTraj.vOutHitTable   = [0,0,0];    %not known yet
            ballTraj.Wout           = [0,0,0];
            ballTraj.hitPointId        = -1;
            ballTraj.trigger        =  0;
            ballTraj.tHit = ones(ballTraj.numOfPlanes,1)*-100;
            ballTraj.pHit = zeros(ballTraj.numOfPlanes,3);
            ballTraj.paddleOrientHit = zeros(ballTraj.numOfPlanes,3);
            ballTraj.paddleVHit = zeros(ballTraj.numOfPlanes,3);
        end 
        


%% updateWithAir
        function [tHit,pHit,paddleOrientHit,paddleVHit,hitPointId] = updateWithAir(self,P, V,W,t)
        global PLOT
            self.updateState(P, V, t);
            if(self.N < self.minNpos)
                if(self.state==2||self.state==4)
                    tHit = self.tHit;
                    pHit = self.pHit;
                    paddleOrientHit = self.paddleOrientHit;
                    paddleVHit = self.paddleVHit;
                    hitPointId = self.hitPointId;
                else
                    tHit = 10000*ones(self.numOfPlanes,1);
                    pHit = zeros(self.numOfPlanes,3);
                    paddleOrientHit= zeros(self.numOfPlanes,3);
                    paddleVHit= zeros(self.numOfPlanes,3);
                    hitPointId = -1;
                end
                if(self.state == 1 || self.state == 3  || self.N < 2)
                    return;
                end
            end
            if (self.state == 1) || (self.state == 3)
%                 dbstop if error
                
%                 timeVec = (t : self.tStep : t+(self.maxNpredPos-1)*self.tStep)';
                %% State = 1 - flight model
                pos      = zeros(2*self.maxNpredPos,3);
                timeVec  = zeros(1,2*self.maxNpredPos);
                i=1;
                kd = self.Env.AirDrag;
                km = self.Env.Magnus;
                dt = 0.005;
                Vb = V;
                Pb = P;
                tb = t;
                a = [0,0,-9.8];
                while (Pb(3)>self.Ball.Radius) && (i<2*self.maxNpredPos) 
                    a = [0,0,-9.8] - kd*norm(Vb)*Vb + km*cross(W,Vb);
                    Vb = Vb + dt*a;
                    Pb = Pb + dt*Vb;
                    tb = tb + dt;
                    pos(i,:) = Pb;
                    timeVec(i) = tb;
                    i = i+1;
                end
                c = (Pb(3)-self.Ball.Radius)/(Vb(3)*dt);
                self.tHitTable       = tb - c*dt;
                self.pHitTable       = Pb - c*dt*Vb;
                self.vInHitTable     = Vb - c*dt*a;
                
                if PLOT
                    hold off
                    plot(timeVec(1:i-1),pos(1:i-1,3),'color','blue');
                    hold on;
                    plot(self.tStamps(1:self.N),self.Positions(1:self.N,3),'x');
                end  
%                 hold on;
%                 plot(self.tStamps(1:self.N),self.Positions(1:self.N,1),'x');
%                 hold on
%                 plot(timeVec(1:i-1),pos(1:i-1,1),'-','color','black');
             %% State = 1 - find vOutHitTable 
                u = self.Env.Table_Friction; %Environment.Env.Table_Friction 
                kv = -self.Table.COR(3);
                rr = sqrt( (Vb(1) - W(2)*0.02)^2 + (Vb(2) + W(1)*0.02)^2);
                
                K = (u*(1+kv)*abs(Vb(3)))/rr;
                if (K<0.4)
                    Vex = (u*(1+kv)*abs(Vb(3))*(W(2)*0.02 -Vb(1)))/rr + Vb(1);
                    Vey = (u*(1+kv)*abs(Vb(3))*(-W(1)*0.02-Vb(2)))/rr + Vb(2);
                    Vez = -kv*Vb(3);
                    Vo = [Vex,Vey,Vez];  

                    self.Wout(1) = W(1) + (3*u*(1+kv)*abs(Vb(3))*(-W(1)*0.02 - Vb(2)))/(2*0.02*rr);
                    self.Wout(2) = W(2) - (3*u*(1+kv)*abs(Vb(3))*( W(2)*0.02 - Vb(1)))/(2*0.02*rr);
                    self.Wout(3) = W(3);
                else
                    Vex =   0.4*W(2)*0.02  + 0.8*Vb(1);
                    Vey =  -0.4*W(1)*0.02  + 0.6*Vb(2);
                    Vez =  -kv*Vb(3);
                    Vo = [Vex,Vey,Vez]; 
                    self.Wout(1) = 0.4*W(1) - 0.6*Vb(2)/0.02; 
                    self.Wout(2) = 0.4*W(2) + 0.6*Vb(1)/0.02;
                    self.Wout(3) = W(3);
                end
%                 self.vOutHitTable = Vo;
                if(self.hitPointId > 0)
                    self.vOutHitTable = (self.N*self.vOutHitTable + 5*Vo)/(self.N+5);
                else
                    self.vOutHitTable = Vo;
                end
           %% State = 1  - predicting the second half
                i=1;
                Vb = self.vOutHitTable;
                Pb = self.pHitTable;
                tb = self.tHitTable;
                dt = self.tStep;
                while (i<=self.maxNpredPos)
                    self.Predicted_P(i,:) = Pb;
                    self.Predicted_V(i,:) = [Vb self.Wout];
                    self.Predicted_t(i) = tb;
                    
                    a = [0,0,-9.8] - kd*norm(Vb)*Vb + km*cross(self.Wout,Vb);
                    Vb = Vb + dt*a;
                    Pb = Pb + dt*Vb;
                    tb = tb + dt;
                    
                    i = i+1;
                end
                if(PLOT)
                    hold on
                    legend off
                    xaxis(0,1.5)
                    yaxis(0,1.5)
                    grid on
                    plot(self.Predicted_t(1:self.maxNpredPos),self.Predicted_P(1:self.maxNpredPos,3),'--','color','black')
                end
            else
            %% State = 2 - flight model
                i = 1;
                kd = self.Env.AirDrag;
                km = self.Env.Magnus;
                Vb = V;
                Pb = P;
                tb = t;
                dt = self.tStep;
                while (i<self.maxNpredPos)
                    a = [0,0,-9.8] - kd*norm(Vb)*Vb + km*cross(W,Vb);
                    Vb = Vb + dt*a;
                    Pb = Pb + dt*Vb;
                    tb = tb + dt;
                    self.Predicted_P(i,:) = Pb;
                    self.Predicted_V(i,:) = [Vb W];
                    self.Predicted_t(i) = tb;
                    i = i+1;
                end
                if PLOT
                    hold on
                    legend off
                    grid on
                    plot(self.Predicted_t(1:self.maxNpredPos),self.Predicted_P(1:self.maxNpredPos,3),'color','blue')
                    hold on
                    plot(t,P(3),'x')
                    grid on
                    xaxis(0,1.5)
                    yaxis(0,1.5)
                end
%                 if( t > sel)
%                     plot(self.tStamps(1:self.N),self.Positions(1:self.N,3),'color','green');
%                 end
            end
            self.PredictEnd     = true;
           %% Desired Hit 
% numOfPlanes = 8;
%             tHit = zeros(numOfPlanes,1);
%             pHit = zeros(numOfPlanes,3);
%             paddleOrientHit = zeros(numOfPlanes,3);
%             paddleVHit = zeros(numOfPlanes,3);
%             hitPointId
%              self.fixHitplaneX = 0.2;
    self.fixHitplaneX = self.minHitplaneX;
    for i=1:self.numOfPlanes
        if(self.state > 2)
            self.fixHitplaneX = -self.fixHitplaneX;
        end
            [self.tHit(i),self.pHit(i,:),self.paddleOrientHit(i,:),self.paddleVHit(i,:),self.hitPointId] = self.DesiredPaddleHit();
%             tHit(i) = self.tHit;
%             pHit(i,:) = self.pHit;
%             paddleOrientHit(i,:) = self.paddleOrientHit;
%             paddleVHit(i,:) = self.paddleVHit;
%             hitPointId = self.hitPointId;
            self.fixHitplaneX = self.minHitplaneX + i*self.HitPlaneDx;
    end   
            tHit = self.tHit;
            pHit = self.pHit;
            paddleOrientHit = self.paddleOrientHit;
            paddleVHit = self.paddleVHit;
            hitPointId = self.hitPointId;
            
end
        function updateState(self,P, V, t)
            %updates the state of the trajectory and adds the new point
            if(self.N > 0)
                lastV = self.Velocities(self.N,:);
            else
                self.state                  = 1;
                self.N                      = 1;
                self.Positions(self.N,:)    = P;
                self.Velocities(self.N,1:3) = V;
                self.tStamps(self.N)        = t;
                return;
            end
%             if(V(3)*lastV(3)<0 && V(3)<0)
%                 set_param('RobotAndBall','SimulationCommand','pause')
%                 P
%             end
            if(V(3)*lastV(3)<0 && V(3)>0) || ((V(1)*lastV(1)<0))    %the transition between states
               self.state               = self.state+1;
               if(self.state == 5)
                   self.state            =1;
               end
               if(self.state == 2 || self.state == 4 )
                   self.tHitTable       = t;
                   self.pHitTable       = P;
                   self.vInHitTable     = lastV;
                   self.vOutHitTable    = V;


%                    hold on
%                    plot(self.tStamps(1:self.N),self.Positions(1:self.N,3),'color','blue')
                   
               end
               self.N                   = 0;
               if((V(1)*lastV(1)<0))
                   self.PredictEnd = false;
                   self.hitPointId = -1;
               end
            end
            
                self.N                  = self.N+1;
                self.Positions(self.N,:)  = P;
                self.Velocities(self.N,1:3) = V;
                self.tStamps(self.N)    = t;
        end %updateState
         
        function [tHit,pHit,paddleOrientHit,paddleVHit,hitPointId] = DesiredPaddleHit(self)
            % the input is vectors of predicted trajectory of the ball, N is the length
            %the output are the needed (moments,position, orientation,velocity) of the
            %paddle to hit the ball to return it
            % first we will try the fixed plane 
            BallRadius = 0.02;
            znet       = 0.20;
            g = -9.81;
%             paddleCor = 0.72; % racket coefficient of restitution
            idx = bSearch(self.Predicted_P(1:end-1,1),self.maxNpredPos-1,self.fixHitplaneX);

            tHit = self.Predicted_t(idx);
            pHit = self.Predicted_P(idx,:)';
            Vi = self.Predicted_V(idx,1:3)';   %the velocity of the ball before the moment of hitting ; if the spinning is taken into account then this vector will be (1x6)

            %where to return the ball (a point on the oponent side)
            if(self.hitPointId == -1)
                if(self.state < 3)
                    self.pOpponentSidePoint = self.GoalOnRight; %[2 ; -0.6 ; BallRadius] + [ 1 ; 1.3 ; 0 ].*rand(3,1);
                    self.pOpponentSidePoint(2) = -0.4+0.8*rand;
                    self.zAboveNet = self.ZnetToRight; %znet + 0.1 + 0.3*rand(1);
                else
                    self.pOpponentSidePoint = self.GoalOnLeft;%[-0.6;-0.1;0.02]; %[2 ; -0.6 ; BallRadius] + [ 1 ; 1.3 ; 0 ].*rand(3,1);
                    self.pOpponentSidePoint(2) = -0.4+0.8*rand;
                    self.zAboveNet = self.ZnetToLeft; %znet + 0.1 + 0.3*rand(1);
                end
            end
            % find ball's initial velocity after hit %from primary report 1st year  
            zt = [pHit(3); self.zAboveNet ;self.pOpponentSidePoint(3)];
            xt = [pHit(1); 0 ; self.pOpponentSidePoint(1)];  %Table.Dimensions(1)/2+Robot1.Distance_Table = 1.67
            A = [xt.^2 xt [1;1;1]];
            Cz = A\zt;

            %the ball's initial velocity after hit
            sig = 1;
            if(self.state > 2)
                sig = -1;
            end
            vox =  sig*sqrt(g/(2*Cz(1)));
            voz = Cz(2)*vox + g*pHit(1)/vox;
            voy = (self.pOpponentSidePoint(2) - pHit(2)) / (self.pOpponentSidePoint(1) - pHit(1)) * vox;

            %calculating the desired racket velocity
            Vo = [vox;voy;voz];
            nrd = (Vo-Vi)/norm(Vo-Vi);
            vr = (Vo'*nrd + self.paddleCor*(Vi'*nrd))/(1+self.paddleCor);
            paddleVHit = vr*nrd;

            paddleOrientHit=nrd;
            self.hitPointId = self.hitPointId+1;
            hitPointId = self.hitPointId;

            pHit=pHit';
            paddleOrientHit=paddleOrientHit';
            paddleVHit=paddleVHit';
        end
    end %methods
    
    
    methods(Static)
        function [Cx,Cy,Cz] = fit(self)
            w = linspace(1, 1,self.N);
            ftx = fittype('a*x+b','coefficients', {'a','b'});
            [fitx] = fit(self.tStamps(1:self.N),self.Positions(1:self.N,1),ftx,'StartPoint',[1 1],'Weight',w);

            fty = fittype('a*x+b','coefficients', {'a','b'});
            [fity] = fit(self.tStamps(1:self.N),self.Positions(1:self.N,2),fty,'StartPoint',[1 1],'Weight',w);

            ftz = fittype('a*x^2+b*x+c','coefficients', {'a','b','c'});
            [fitz] = fit(self.tStamps(1:self.N),self.Positions(1:self.N,3),ftz,'StartPoint',[1 1 1],'Weight',w);
           
            Cx = [fitx.a fitx.b];
            Cy = [fity.a fity.b];
            Cz = [fitz.a fitz.b fitz.c];
            
%             if(self.state == 1)
%                 hold all
%                 plot(fitz);
%                 legend off
%                 xaxis(0,1.5)
%                 yaxis(-0.2,1.3)
%                 grid on
%             end

            if(Cz(1) > 0 )
                warning('The parabola is facing up %d %d %d %d',self.N,Cz(1),Cz(2),Cz(3));
            end
        end% fit()
    end
end
