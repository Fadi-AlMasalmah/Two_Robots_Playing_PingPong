function [configSoln,solnInfo] = IK_Codgen(tform,initialguess)
dr = 0.120; %(m) distance between center of R7 and the center of the racket (the hit point)
d1=0.340;
d3 = 0.4;
d5 = 0.4;
d7 = 0.126;
dhparams = [[0,0,d1,0]
           [0,-pi/2 ,0 , 0]
           [0,pi/2,d3, 0]
           [0,pi/2,0 , 0]
           [0,-pi/2,d5,0]
           [0,-pi/2,0 , 0]
           [0, pi/2  ,d7+dr, 0]];
% dhparams(:,4) = zeros(7,1);

iiwa7 = rigidBodyTree("MaxNumBodies",9,"DataFormat","column");

body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');
setFixedTransform(jnt1,dhparams(1,:),'dh');
body1.Joint = jnt1;
addBody(iiwa7,body1,'base')

body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
body5 = rigidBody('body5');
jnt5 = rigidBodyJoint('jnt5','revolute');
body6 = rigidBody('body6');
jnt6 = rigidBodyJoint('jnt6','revolute');

body7 = rigidBody('body7');
jnt7 = rigidBodyJoint('jnt7','revolute');

setFixedTransform(jnt2,dhparams(2,:),'dh');
setFixedTransform(jnt3,dhparams(3,:),'dh');
setFixedTransform(jnt4,dhparams(4,:),'dh');
setFixedTransform(jnt5,dhparams(5,:),'dh');
setFixedTransform(jnt6,dhparams(6,:),'dh');
setFixedTransform(jnt7,dhparams(7,:),'dh');

body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;
body7.Joint = jnt7;

addBody(iiwa7,body2,'body1')
addBody(iiwa7,body3,'body2')
addBody(iiwa7,body4,'body3')
addBody(iiwa7,body5,'body4')
addBody(iiwa7,body6,'body5')
addBody(iiwa7,body7,'body6')
% 
% show(iiwa7);
% axis([-0.5,0.5,-0.5,0.5,-0.5,0.5])
% axis off
weights = [0.25 0.25 0.25 1 1 1];
ik = inverseKinematics('RigidBodyTree',iiwa7); %,'SolverAlgorithm','LevenbergMarquardt'
ik.SolverParameters.MaxTime = 0.1;
[configSoln,solnInfo] = ik('body7',tform,weights,initialguess); 
end

% endEffectorName = 'tool';       
% tform = trvec2tform([0.7 -0.7 0]);
% weights = [0.25 0.25 0.25 1 1 1];
% initialGuess = zeros(7,1);
% codegen IK_Codgen -args {tform,initialGuess}
% 
% time = timeit(@() IK_Codgen(tform,initialGuess))
% mexTime = timeit(@() IK_Codgen_mex(tform,initialGuess))
% timeit(@()ik('body7',tform,weights,initialGuess)) 