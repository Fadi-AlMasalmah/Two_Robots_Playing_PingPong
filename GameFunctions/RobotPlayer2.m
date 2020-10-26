classdef RobotPlayer2<handle
    %RobotPlayer2 is when two robots are playing
    % Version 1
    
    properties
        initialQ  
        currP
        currQ
        currV
        currQd
        currQdd
        currTime
        
        tHit
        pHit
        vhpId %virtual hitting point id 
        vHit
        qHit
        
        Cs                  % matrix [7x6] coefficients of the 5th deg Qtrajectories
        initialP
        noUpdateThreshold % if (t > tHit-noUpdateThreshold) then the VHP will not be updated anymore       
        dTtoHome           % amount of time to go home after hitting the ball
        tHome              % moment in time to be at home        
        state             %0 = waiting , 1=moving to vhp, 2 = going home
        
        RobotModel
        ik
        DistToTable ;%= [0.3 0 0];  % x,y,z
        Safety_Pr = 0.15;   %disctance above the table to prevent collision
        Safety_W = 0.15;
        Safety_E = 0.10;
        Qlimits = [170,120,170,120,170,120,175]'*pi/180;
        QdLimits = [98 98 100 130 140 180 180]'*pi/180 * 4;
        planeId = 1;
        lambdaRange = pi; %rad
        alphaRange = 2*pi; %rad
        numOfAlphaTrials = 15;
        numOflambdaTrials = 1;
        reachDist = 1.2; % the distance that the robot can reach
        Config_mat;
        TransMat;
        RobotId;
        goalExist = false; % this becomes true when the robot has a gaol point and starts moving towards it 
    end
    
%% Methods
    methods
        function obj = RobotPlayer2(varargin)
            %RobotPlayer Construct an instance of this class
            %   Detailed explanation goes here

                obj.initialQ = varargin{1};
                obj.TransMat = varargin{2};
                obj.DistToTable = varargin{3}; 
                obj.RobotId = varargin{4};
            obj.RobotModel = importrobot('iiwa7.urdf');
            obj.RobotModel.DataFormat = 'column';
            racket = rigidBody('racket');
            racketJoint = rigidBodyJoint('racket_Joint','fixed');
%             tform1 = trvec2tform([0 0 0.12])
            setFixedTransform(racketJoint,trvec2tform([0 0 0.12]))  %relative to iiwa_link_ee
            racket.Joint = racketJoint;
            addBody(obj.RobotModel,racket,'iiwa_link_ee');              
            
            obj.ik = inverseKinematics('RigidBodyTree',obj.RobotModel); %,'SolverAlgorithm','LevenbergMarquardt'
            obj.ik.SolverParameters.MaxTime = 0.1;
            obj.ik.SolverParameters.SolutionTolerance = 0.03;
            obj.initialP    = zeros(7,1);          
            obj.currP       = obj.initialP;
            obj.currQ       = obj.initialQ; 
            obj.currV       = zeros(6,1);
            obj.currQd      = zeros(7,1);
            obj.currQdd      = zeros(7,1);
            obj.currTime    = 0;     
            obj.state       = 0;  %0 = waiting ,1 = moving to hit , 2 = going back home
            obj.vhpId       = -1;  % in order to know wether there is a new hitting point or not
            obj.tHit = 100;
            obj.tHome = 1.8;
            obj.pHit = [0.2;0.4;0.3];
            obj.vHit = [0;0;0;0.2;0.4;0.3];
            obj.qHit = obj.initialQ;
            
            obj.noUpdateThreshold = 0.15; %sec befor hitting we stop traj-updating
            obj.dTtoHome    = 0.9; %sec time to go home after finishing the strike
            obj.Cs = zeros(7,6);
            obj.Cs(:,1) = obj.initialQ;
%             setProperties(obj,nargin,varargin{:}) 
            obj.Config_mat = zeros(8,3);
            len = 1;
            for i = 1:2
                for j = 1:2
                    for k = 1:2
                        obj.Config_mat(len,1) = FLIP(i);
                        obj.Config_mat(len,2) = FLIP(j);
                        obj.Config_mat(len,3) = FLIP(k);
                        len = len+1;
                    end
                end
            end
            
            function f = FLIP(i)
                if i ==2
                    f = -1;
                else
                    f = 1;
                end   
            end
            
        end

%% UPDATE-Coeffitients
        function [Cs,CsUpdated] = updateCoeffitients(obj,Vball,t,tHit,pHit,paddleOrientHit,paddleVHit,poinId,currQ,currQd,currQdd)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            %Robot is waiting and not moving 
global MyIk
global checkLimitsAndCollision
global NeedDbg
global use_5rd_order
%             if(NeedDbg == 1)
%                 set_param('RobotAndBall','SimulationCommand','pause')
%                 NeedDbg
%             end
            CsUpdated = false;
            BallTowardsRobot = false;
            if(Vball(1)<0 && strcmp(obj.RobotId,'Left')) || (Vball(1)>0 && strcmp(obj.RobotId,'Right'))
                BallTowardsRobot = true;
            end

            obj.currQ = currQ; obj.currQd = currQd; obj.currQdd = currQdd; obj.currTime = t;
            [obj.currQdd,obj.currQd ,obj.currQ] = traj_5_order(t,obj.Cs);
%             jerk = obj.Cs*[0  0   0    6    24*t 60*t^2]';
            if obj.state == 0 && BallTowardsRobot && poinId>-1 %  tHit(end)<100 && 
                obj.goalExist = false;
                obj.pHit        = pHit(end,:)';
                obj.vHit        = paddleVHit(end,:)';
                obj.tHit        = tHit(end);
                obj.state = 1;
            end
        
            % robot is moving towards the hitting point
%---------State == 1 -----------------------
            if obj.state == 1
                %transform to Robot's coordinate
                np = length(tHit);
                pHit1 = [pHit ones(np,1)]*obj.TransMat';
                pHit = pHit1(:,1:3);
                RMat = obj.TransMat(1:3,1:3);
                paddleOrientHit = paddleOrientHit*RMat';
                paddleVHit1 = paddleVHit(:,1:3)*RMat';
                paddleVHit2 = paddleVHit(:,4:6)*RMat';
                paddleVHit = [paddleVHit1 paddleVHit2];

                tEps = 0.02; %sec
                pEps = 0.03; %m
                errInP = norm(pHit(obj.planeId,:)'-obj.pHit);
                errInTime = abs(tHit(obj.planeId)-obj.tHit);
                if(errInP<pEps && errInTime<tEps)
                    Cs = obj.Cs;
                    CsUpdated = false;
                    return;
                end
                %if the hitting is not so close, so we update the Qtraj Coeffictients
                if(t<obj.tHit-obj.noUpdateThreshold)
                    if(1||obj.vhpId<0)%%%%%%%%%%
                        tic
                        minCost = 100000000;
                        solved = false;
                        for pId = 1:length(tHit)
                            initialguess    = obj.qHit;
                            obj.pHit        = pHit(pId,:)';
                            obj.vHit        = paddleVHit(pId,:)';
                            obj.tHit        = tHit(pId);
                            if(abs(obj.tHit)<0.01)
                                continue
                            end
                            %inverse kinematics
                            numberOfTrials  = 20;
                            
                            if (checkLimitsAndCollision)
                              [Qd,info]          = obj.bestInvKin4(pHit(pId,:)',paddleOrientHit(pId,:)',initialguess,paddleVHit(pId,:)');
                            elseif(MyIk == 1)
                              [Qd,info]          = obj.bestInvKin3(pHit(pId,:)',paddleOrientHit(pId,:)',initialguess,paddleVHit(pId,:)');
                            else % numerical solution
                              [Qd,info]          = obj.bestInvKin1(pHit(pId,:)',paddleOrientHit(pId,:)',numberOfTrials,initialguess);
                            end
                            if(info>0)
                                [cost,CdistToQh , maxVCost , maxDistToQi , manipCost , distToLimitCost , distToQiCost] = obj.ikCostFunMinV(Qd,paddleVHit(pId,:)');
                                if(cost<minCost)
                                    minCost = cost;
                                    bestQd = Qd;
                                    obj.planeId = pId;
                                    solved = true;
                                end
                            end
                        end
                        if(~solved)
                            disp([obj.RobotId,'  has no solution on any plane --xxxx--'])
                            Cs = obj.Cs;
                            CsUpdated = false;
                            return
                        else
                            disp(['VHP X is ',num2str(pHit(obj.planeId,1))])
                            timeIK = toc;
                            obj.pHit        = pHit(obj.planeId,:)';
                            obj.vHit        = paddleVHit(obj.planeId,:)';
                            obj.tHit        = tHit(obj.planeId);
                            obj.qHit        = bestQd;
                            J               =  geometricJacobian(obj.RobotModel,bestQd,'racket');
                            [Qd_final,qdError,maxQd] = obj.getQd_pinvJ(J,paddleVHit(obj.planeId,:)');
                            if qdError>0.1
                               disp([obj.RobotId,' has no solution for the velocity ---VVVVVV---']); 
                            end
                            [cost,CdistToQh , maxVCost , maxDistToQi , manipCost , distToLimitCost , distToQiCost] = obj.ikCostFunMinV(obj.qHit,paddleVHit(pId,:)');
                            disp(['Cost = ',num2str(cost),' | CdistToQh = ',num2str(CdistToQh),' | maxVCost = ',num2str(maxVCost),' | maxDistToQi = ',num2str(maxDistToQi),' | manipCost=',num2str(manipCost),' | distToLimitCost',num2str(distToLimitCost),' | distToQiCost=',num2str(distToQiCost)])

                        end
                    %calculating current according to the old trajectory inorder to update it and keep it continuous
                    %finding the new coefficients of the new trajectory
                    if(use_5rd_order == 1) 
                        obj.Cs    = obj.find_coeff_traj_5(t,tHit,obj.currQ,bestQd,obj.currQd,Qd_final,obj.currQdd,jerk);
                    else
                        obj.Cs    = obj.find_coeff_traj_3(t,tHit(obj.planeId),obj.currQ,bestQd,obj.currQd,Qd_final);
                    end
                    CsUpdated = true;
                    obj.goalExist = true;
                    end %%%%%%%%%%%%
                elseif (t >= obj.tHit)
                    obj.state       = 2;
                     %new coefficients of the trajectory to homePosition
                    obj.tHome      = t+obj.dTtoHome;
                    if(use_5rd_order == 1) 
                        obj.Cs    = obj.find_coeff_traj_5(t,obj.tHome,obj.currQ,obj.initialQ,obj.currQd,zeros(7,1),obj.currQdd,jerk);
                    else
                        obj.Cs    = obj.find_coeff_traj_3(t,obj.tHome,obj.currQ,obj.initialQ,obj.currQd,zeros(7,1));
                    end
                    obj.vhpId = poinId;
                    CsUpdated = true;
                    obj.goalExist = false;
                else
                    % robot still moving to the same virtual hit point (there is no new point)
                    obj.vhpId = poinId; %taking the id of the new point
                end
            end % if stat == 1
            obj.vhpId = poinId;
            
            %robot is going home
            if obj.state == 2
                if(t>obj.tHome-0.02)
                    obj.Cs = [obj.initialQ zeros(7,5)];
                    obj.vhpId   = -1;
                    obj.state   = 0;
                    CsUpdated = true;
                end  
            end  
            Cs  =obj.Cs;
        end

%% IK_Functions        
        function [bestSolution,SolInfo] = bestInvKin1(obj,p,paddleOrientHit,numberOfTrials,initialguess)
            if(norm(p)>obj.reachDist)
                bestSolution = zeros(7,1);
                SolInfo = 0;
                return;
            end
            SolInfo  = 1;
            alpha = -pi;
            bestSolution = initialguess;
            minCost = 1000000000;
            n = obj.numOfAlphaTrials;
            for i = 1:n 
                [configSoln,info] = obj.InverseKinematics1(p,paddleOrientHit,alpha,initialguess);
                 alpha = alpha + 2*pi/n;

                cost = obj.ikCostFun(configSoln);
                if strcmp(info.Status,'success')
                   cost = cost*0.8;
                end
                if cost < minCost
                    minCost = cost;
                    bestSolution = configSoln;
                end
            end
           if(minCost > 1000000)
%                disp(['no solution on Xplan x =',num2str(p(1))])
               SolInfo  = 0;
               bestSolution = obj.initialQ;
           end
        end
        
        function [bestSolution,SolInfo] = bestInvKin3(obj,p,paddleOrientHit,initialguess,v)
            if(norm(p)>obj.reachDist)
                bestSolution = zeros(7,1);
                SolInfo = 0;
                return;
            end
            minCost = 1000000000;
            SolInfo = 1;
            nAlpha = obj.numOfAlphaTrials;
            nLambda= obj.numOflambdaTrials;
            JointLimits = [170,120,170,120,170,120,175]'*pi/180;

            alpha = -obj.alphaRange/2;
            
            for i = 1:nAlpha
                lambda = pi-obj.lambdaRange/2;
                for j = 1:nLambda 

                    wrist = 1;
                    for k = 1:2    
                        [configSoln,info] = obj.InverseKinematics2(p,paddleOrientHit,alpha,lambda,initialguess,wrist);
                        insideLimits = 0;
                        if info==0          %there is no solution
                            continue
                        end
                        if (abs(configSoln)<(JointLimits) & (obj.isCollisionFree(configSoln)))
                            insideLimits = 1;
                            [cost,CdistToQh , maxVCost , maxDistToQi , manipCost , distToLimitCost , distToQiCost] = obj.ikCostFunMinV(configSoln,v);
                        else
%                             lambda = lambda+2*pi/nLambda;
                            wrist = -1;
                            continue
                        end
                    
                        if cost < minCost  && insideLimits == 1  && info == 1 
                            minCost = cost;
                            bestSolution = configSoln;
    %                         disp('Fooooooouuuuuuuuuuuuuunnnnnnnndddddddddddd');
                        end
                    wrist = -1;
                    end
                    lambda = lambda + obj.lambdaRange/nLambda;
                  
                end
                alpha = -obj.alphaRange/2 + i*obj.alphaRange/nAlpha;
                 
            end
           if(minCost > 1000000)
%                disp(['no solution on Xplan x =',num2str(p(1))])
               SolInfo  = 0;
               bestSolution = obj.initialQ;
           end
        end        
       
        function [bestSolution,SolInfo] = bestInvKin4(obj,p,paddleOrientHit,initialguess,v)
           
            if(norm(p)>obj.reachDist)
                bestSolution = zeros(7,1);
                SolInfo = 0;
                return;
            end
            SolInfo = 1;

            bestSolution = initialguess;
            minCost = 1000000000;
            nAlpha = obj.numOfAlphaTrials;
            nLambda= obj.numOflambdaTrials;
%             JointLimits = [170,120,170,120,170,120,175]'*pi/180;

            alpha = -pi;
            for i = 1:nAlpha
                lambda = -pi;
                for j = 1:nLambda 
                    wrist = 1;
                    for k = 1:2
                        [configSoln,info] = obj.InverseKinematics2(p,paddleOrientHit,alpha,lambda,initialguess,wrist);
                        if info==0          %there is no solution
                            continue
                        end
                        isInsideLimits = obj.isInsideLimits_3rd_traj(configSoln,v);
%                         noCollision = obj.checkTrajCollision(configSoln);
                        if (isInsideLimits & (obj.isCollisionFree(configSoln)))  %abs(configSoln)<(JointLimits) 
                            insideLimits = 1;
                            [cost,CdistToQh , maxVCost , maxDistToQi , manipCost , distToLimitCost , distToQiCost] = obj.ikCostFunMinV(configSoln,v);
                        else
%                             lambda = lambda+2*pi/nLambda;
                            wrist = -1;
                            continue
                        end
                    
                        if cost < minCost  && insideLimits == 1  && info == 1 
                            minCost = cost;
                            bestSolution = configSoln;
    %                         disp('Fooooooouuuuuuuuuuuuuunnnnnnnndddddddddddd');
                        end
                    wrist = -1;
                    end
                    lambda = lambda+2*pi/nLambda;
                end
                alpha = alpha + 2*pi/nAlpha;
            end
            
            if(minCost > 1000000)
               disp(['no solution on Xplan x =',num2str(p(1))])
               SolInfo  = 0;
               bestSolution = obj.initialQ;
           end
        end
        
        function  [configSoln,solnInfo]= InverseKinematics1(obj,p,nrd,theta,initialguess)
            %nrd is the normal to the racket
            %theta is the rotation around nrd
            %we first rotate x to nrd , then rotate the frame around nrd by
            %theta
%              nrd = [0,1,0];

            nrd = nrd/norm(nrd);
            nz = [0;0;1];
            g = acos(nrd'*nz);
            u = cross(nz,nrd);
            u = u/norm(u);
            quat1 = [cos(g/2),u'*sin(g/2)];

            quat2 = [cos(theta/2),sin(theta/2)*nrd'];
%             quat2 = [cos(theta/2),sin(theta/2)*nz'];
            R1 = quat2rotm(quat1);
            R2 = quat2rotm(quat2);
            R3 = R2*R1 ;
            weights = [0.25 0.25 0.25 1 1 1];
            tform = [R3,p;[0 0 0 1]];
            tform1 = trvec2tform([-0.12 0 0]);
            tform = tform*tform1;
            [configSoln,solnInfo] = obj.ik('racket',tform,weights,initialguess); 
        end

        function  [configSoln,solnInfo]= InverseKinematics2(obj,p,nrd,alpha,lambda,initialguess,wrist)
            %nrd is the normal to the racket
            %theta is the rotation around nrd
            %we first rotate x to nrd , then rotate the frame around nrd by
            %theta
            solnInfo = 1;  % solution is found
            nrd = nrd/norm(nrd);
            nx = [-1;0;0];
            g = acos(nrd'*nx);
            u = cross(nx,nrd);
            u = u/norm(u);
            quat1 = [cos(g/2),u'*sin(g/2)];
            quat2 = [cos(alpha/2),sin(alpha/2)*nx'];
            R1 = quat2rotm(quat1);
            R2 = quat2rotm(quat2);
            R3 = R1*R2 ;
            Td07 = [R3,p;[0 0 0 1]];
            config = [1,-1,wrist];
            try
              configSoln = IK_7DOF(0,0,Td07,lambda,config)';
%               configSoln = IK_IIWA7_mex(Td07,lambda,config)';
            catch
                configSoln = obj.initialQ;
                solnInfo = 0; % solution not found
            end
        end %InverseKinematics2
        
        function cost = ikCostFun(obj,Qd)
%             Q = rand(7);
%             D = diag(abs(randn(7, 1)) + 4.3);
%             W = Q'*D*Q; %this shouldn't be random here
                J    =  geometricJacobian(obj.RobotModel,Qd,'racket');
                Jpi = pinv(J);
                Jnt4Effect1 = Jpi(4,4);
                Jnt4Effect = 1/norm(J(4:6,4));
    
                manipInd = sqrt(det(J*J'));
                Qbound = [170 120 170 120 170 120 175]'*pi/180;
                Qcentr = [0,0,0,0,0,0,0]';   % the comfort position
                W  = diag([1 2 1 8 1 2 1]);
%                 W  = diag([2 2 1 2 1 1 1]);
                Qtemp = zeros(7,1);
                Qtemp =  abs(Qd);
%                 distToLimit = ones(7,1)-((Qtemp-Qbound)./(Qbound)).^4;
                t = (Qtemp-Qbound)./(Qbound);
                distToLimit = (exp((-t.^2).*25)).^2;
                Qtemp = Qd;
                distToCenter = (Qtemp - Qcentr)./(Qbound - Qcentr);
                distToQi = (Qd-obj.initialQ)./Qbound;
                [maxDistToQi,idx] = max(abs(Qd-obj.initialQ));
                 maxDistToQi = maxDistToQi/Qbound(idx);
%                 distToQi = (Qd-obj.currQ)./Qbound;
                manipCost = 1/manipInd;
                 d2cCost = distToCenter'*W*distToCenter;

                cost = 8*Jnt4Effect1 + 0*5*Jnt4Effect+15*maxDistToQi+2.5*manipCost + distToLimit'*W*distToLimit+6*distToQi'*W*distToQi;
        end        
        
        function [cost,CdistToQh , maxVCost , maxDistToQi , manipCost , distToLimitCost , distToQiCost] = ikCostFunMinV(obj,Q,v)  % calculates MaxV
%             Q = rand(7);
%             D = diag(abs(randn(7, 1)) + 4.3);
%             W = Q'*D*Q; %this shouldn't be random here
                J    =  geometricJacobian(obj.RobotModel,Q,'racket');
                Jpi = pinv(J);
                Jnt4Effect1 = Jpi(4,4);
                Jnt4Effect = 1/norm(J(4:6,4));
            
         %minimize velocity
                T = obj.tHit - obj.currTime;
                q0 = obj.currQ;
                q0_ = obj.currQd;
                qf = Q;
                qf_ = Jpi*v;
                d = q0;
                c = q0_;
                b = 3*(qf-q0)./T.^2 - 2*q0_./T - qf_./T;
                a = -2*(qf-q0)./T.^3 + (qf_ + q0_)./T.^2;
                maxV0 =  - b.^2./(3*a) + c;
                inTime = -b./(3*a)>obj.currTime & -b./(3*a)<obj.tHit;
                maxV1 = zeros(7,1);
                maxV1(inTime) = maxV0(inTime);
%                 ztemp = zeros(7,1);
%                 maxV1(abs(maxV1)<abs(q0_)|abs(maxV1)<abs(qf_))=0;
                maxV = max([abs(q0_) abs(qf_) abs(maxV1)]');
%                 maxV = max([q0_ qf_ maxV1],[],2)
                maxVCost = max(maxV);

%manipulability index
                manipInd = sqrt(det(J*J'));
                Qbound = [170 120 170 120 170 120 175]'*pi/180;
                Qcentr = [0,0,0,0,0,0,0]';   % the comfort position
                W  = diag([1 2 1 8 1 2 1]);
%                 W  = diag([2 2 1 2 1 1 1]);
                Qtemp = zeros(7,1);
                Qtemp =  abs(Q);
%                 distToLimit = ones(7,1)-((Qtemp-Qbound)./(Qbound)).^4;
                t = (Qtemp-Qbound)./(Qbound);
                distToLimit = (exp((-t.^2).*25)).^2;
                Qtemp = Q;
                distToCenter = (Qtemp - Qcentr)./(Qbound - Qcentr);
                distToQi = (Q-obj.initialQ)./Qbound;
                [maxDistToQi,idx] = max(abs(Q-obj.initialQ));
                 maxDistToQi = maxDistToQi/Qbound(idx);
%                 distToQi = (Qd-obj.currQ)./Qbound;
                manipCost = 1/manipInd;
                 d2cCost = distToCenter'*W*distToCenter;
                
% distnce to current qHit
                CdistToQh = 0;
                if obj.goalExist
                    CdistToQh =  (obj.qHit - Q)'*W*(obj.qHit - Q);
                end
                
%                 W2 = diag([1 1 1 1 1 1 1]);
                CdistToQh = 5*CdistToQh;
                maxVCost = 5*maxVCost;
                maxDistToQi = 15*maxDistToQi;
                manipCost = 2.5*manipCost;
                distToLimitCost = distToLimit'*W*distToLimit;
                distToQiCost = 6*distToQi'*W*distToQi;
               
                cost = CdistToQh + maxVCost + maxDistToQi + manipCost + distToLimitCost + distToQiCost;
                
%                 cost = 5*CdistToQh + 5*maxVCost + 0*3*maxV*W2*maxV' + 0*8*Jnt4Effect1 + 0*5*Jnt4Effect+15*maxDistToQi+2.5*manipCost + distToLimit'*W*distToLimit+6*distToQi'*W*distToQi;

                
        end
        
        function isInsideLimits = isInsideLimits_3rd_traj(obj,Qh,v)
            result = true;
            
            if ~(abs(Qh)<obj.Qlimits)
                isInsideLimits = false;
                return;
            end
            J    =  geometricJacobian(obj.RobotModel,Qh,'racket');
            Jpi = pinv(J);
            T = obj.tHit - obj.currTime;
            q0 = obj.currQ;
            q0_ = obj.currQd;
            qf = Qh;
            qf_ = Jpi*v;
            d = q0;
            c = q0_;
            b = 3*(qf-q0)./T.^2 - 2*q0_./T - qf_./T;
            a = -2*(qf-q0)./T.^3 + (qf_ + q0_)./T.^2;
            
            
            %Qmax
             t1=  -(b + (b.^2 - 3*a.*c).^(1/2))./(3*a);
             t2=  -(b - (b.^2 - 3*a.*c).^(1/2))./(3*a);
              
             Qmax1 = (27*a.^2.*d + 2*(b.^2 - 3*a.*c).^(3/2) + 2*b.^3 - 9*a.*b.*c)./(27*a.^2);
             Qmax2 = (27*a.^2.*d - 2*(b.^2 - 3*a.*c).^(3/2) + 2*b.^3 - 9*a.*b.*c)./(27*a.^2);
             
             inTime1 = t1>obj.currTime & t1<obj.tHit; 
             mQ1 = zeros(7,1);
             mQ1(inTime1) = Qmax1(inTime1);      
             result = result & (abs(mQ1)<obj.Qlimits);
             
             inTime2 = t2>obj.currTime & t2<obj.tHit; 
             mQ2 = zeros(7,1);
             mQ2(inTime2) = Qmax2(inTime2);
             result = result & (abs(mQ2)<obj.Qlimits);
             
             tv = -b./(3*a);
             maxV =  - b.^2./(3*a) + c;
             inTimeV = tv>obj.currTime & tv<obj.tHit;
             mV = zeros(7,1);
             mV(inTimeV) = maxV(inTimeV);
             result = result & (abs(mV) < obj.QdLimits);
             
%              noCollision = obj.checkTrajCollision(Qh,a,b,c,d);
%              result = result & noCollision;
             
             if(result)
                 isInsideLimits=true;
             else
                 isInsideLimits=false;
             end
           
        end
        
        function noCollision = checkTrajCollision(obj,Q,a,b,c,d)
            noCollision = true;
            T0 = obj.currTime;
            Tf = obj.tHit;
            for t=linspace(0,Tf-T0,4)
                q = a*t^3+b*t^2+c*t+d;
                noCollision = noCollision & obj.isCollisionFree(q);
            end
        end
          
        function collisionFree = isCollisionFree(obj,Q)
            [E  W  Pr] = Calc_E_W_Pr(Q);
            Lpr = false;
            Lw = false;
            Le = false;
            if(Pr(1)<0.1 || (Pr(3)>obj.Safety_Pr))
                Lpr = true;
            end
            
            if(W(1)<0.1 || (W(3)>obj.Safety_W))
                Lw = true;
            end
            
            if(E(1)<0.1 || (E(3)>obj.Safety_E))
                Le = true;
            end
            
            collisionFree = Lpr&Lw&Le;
            
            % I should check the collision with the robot itself
        end
            
    end
%% Static Methods 
    methods(Static, Access = protected)
     
        function [bestQd,velocity_error,minMaxQd] = getQd_pinvJ(J,v) 
            bestQd = pinv(J)*v;
            velocity_error = (max(abs(J*bestQd-v)));
            minMaxQd = max(abs(bestQd));
        end
   
        function [q,qd,qdd] = traj_5_order(t,traj_Cs)
            tt = [1 t t^2 t^3 t^4 t^5]';
            q = traj_Cs*tt;
            
            ttd = [0 1 2*t 3*t^2 4*t^3 5*t^4]';
            qd = traj_Cs*ttd;
            
            ttdd = [0 0 2 6*t 12*t^2 20*t^3]';
            qdd = traj_Cs*ttdd;
        end
           
        function [traj_Cs] = find_coeff_traj_3(Ti,Tf,Qi,Qf,Qdi,Qdf)
            %T*c=q
%             global update_delay
            
%             Qddi_del = Qddi +  jerk*update_delay;
%             Qdi_del = Qdi + Qddi*update_delay+0.5*jerk*update_delay^2;
%             Qi = Qi+Qdi*update_delay+0.5*Qddi*update_delay^2+1/6*jerk*update_delay^3;
%             Ti = Ti+update_delay;
%             a = -(2*sqrt(6)-3)*(Ti-Tf);
%             b = (4*sqrt(6)-1)*(Ti-Tf);
%             c = (4-6^(3/2));
%             d = (sqrt(6)+1)*(Ti-Tf)^2;
%             Qddf = (a*Qdi_del' + b*Qdf + c*(Qi'-Qf))/d;
%             Qddf = zeros(7,1);
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
            traj_Cs = [traj_3_Cs zeros(7,2)];
        end
    end
end




