%% Final project: EN530.678 Nonlinear Control and Planning in Robotics
%
% Algorithm:RRT blossom
% Reference: RRT with a local ?ood-?ll behavior, Kalisiak M., van de Panne M., ICRA06
% http://www.dgp.toronto.edu/~mac/pubs/rrt-blossom.pdf
%
% Arthur: Site Hu, Han Shi
% 05/14/2018
% Please download librobotics from http://srl.informatik.uni-freiburg.de/downloads
% It's used to draw the unicycle
%
% The kinematic of EduMIP is descrirbed by the EdumipKinematics.m script.


%%
clear all
clc
close all

% add librobotics path
addpath('c:/librobotics')

%% Parameters
PRINTSAMPLE=1;
LOAD=1;
BLOSSOM=1;


global addedVert

% size of robot
global widthrobot
global lengthrobot

widthrobot=0.4;
lengthrobot=0.6;


% configuration space
N=3;
dim= [0 5 0 5 0 2*pi];


% Define Tree
% Tree consists of vertex
% v.id  -- ID of the vertex
% v.pose -- Pose of the vertex
% v.edgeq -- edge where to save intermediate configurations
% v.edgeu -- edge where to save intermediate controls u
% v.pId -- parent ID
RRT_tree={};

% initial and end pose
qinit=[2;1;0];
qgoal=[4;3.5;0];
radiusGoal=0.50;


%% Define obstacles: consider obstacles as ellipses
%  obs(i).position
%  obs(i).width
%  obs(i).heigth

if(LOAD)
    obst = dlmread('environment.txt');
    [nr_obst, nc_obst]=size(obst);
    for i=1:nr_obst
        obs(i).position=[obst(i,1);obst(i,2)];
        obs(i).radius=obst(i,3);
    end
else
    obs(1).position=[3;2.5];
    obs(1).radius=0.3;
    
    obs(2).position=[1;1];
    obs(2).radius=0.2;
    
    obs(3).position=[1;3];
    obs(3).radius=0.51;
    
    
    obs(4).position=[1;2.5];
    obs(4).radius=0.4;
    
    obs(5).position=[1;5];
    obs(5).radius=0.5;
    
    obs(5).position=[4;3];
    obs(5).radius=0.2;
end

[h, nobs]=size(obs);

%% plot enviroment, initial and goal position
for j = 1:2
    % plot environment
    figure(j),hold on
    for i=1:nobs
        drawellipse([obs(i).position;0],obs(i).radius,obs(i).radius,'r');
    end
    
    % plot initial position
    drawrobot(qinit,'b',2,widthrobot,lengthrobot);
    % plot goal region
    drawellipse(qgoal,radiusGoal,radiusGoal,'b');
end


%%  motion primitives array
motionPrimitiveCommandArray = [1 0; 1 1.3; 1 -1.3; 0.8 0.3; 0.8 -0.3; 1 2; 1 -2];
delta=0.25;


%% global id counter
global cntId
cntId=1;

%% RRT

% add qinit to the tree
RRT_tree(1).id=1;
RRT_tree(1).pose=qinit;
RRT_tree(1).edgeq= [];
RRT_tree(1).edgeu= [];
RRT_tree(1).pid=0;


addedVert=1;
collision=0;
goalFound=0;


while(goalFound<1)
    %% Generate Random Configuration
    
    qrand=randomstate(N,dim);
    % Check
    if(isempty(qrand))
        break;
    end
    
    if(checkcollision(qrand(1:2)',obs)>0)
        disp('wrong qrand')
        continue
    end
    
    if(PRINTSAMPLE)
        figure(1),plot(qrand(1),qrand(2),'*g')
    end
    
    
    
    %% Find Nearest Vertex
    v_near=NearestTree(RRT_tree,qrand);
    figure(1)
    hv=plot(v_near.pose(1),v_near.pose(2),'*r');
    
    
    %% Extend the tree
    [v_new, RRT_tree]=extendblossom(RRT_tree,obs,v_near,qrand,motionPrimitiveCommandArray,delta);
    
    
    
    if(isempty(v_new.edgeq))
        continue;
    end
    
    %% If Goal return path
    
    if(norm(v_new.pose(1:2)-qgoal(1:2))<radiusGoal)
        disp('GOAL!! reached!!!')
        goalFound=1;
        p=getPath(v_new,RRT_tree);
        u=getControls(v_new,RRT_tree);
        break;
    end
end


%% Plot Results
if(goalFound==1)
    figure(1),hold on,
    [npoints, ~]=size(p);
    
    figure(1)
    for i=1:npoints
        drawrobot(p(i,:),'r',1,widthrobot,lengthrobot);
    end
    
    % plot path
    figure(2)
    for i=1:(npoints-1)
        plot([p(i,1),p(i+1,1)],[p(i,2),p(i+1,2)],'r');
    end
    
    % plot desired controls
    figure(3),hold on
    subplot(2,1,1),plot(u(:,1)),title('desired v')
    subplot(2,1,2),plot(u(:,2)),title('desired w')
end

%% Trajectory Tracking
% Feedback Linearization

figure(2)
[ninputs,~]=size(u);
tSpan = [0 delta];
Q0 = qinit;
for i=1:ninputs
    % simulate system
    [T, Q] = ode45(@(t,q) EduMIP_ode(i,p, u, t, q),tSpan,Q0); % solve ODE
    plot(Q(:,1), Q(:,2), '-g');
    Q0 = Q(end,:);
end

%% tracking control law
function u_ctrl = EduMIP_ctrl(i,p, u, t, q)
[ninputs,~]=size(u);
[npoints, ~]=size(p);
p_u = floor(npoints/ninputs);

% get desired outputs (x,y positions)
yd = [p((i-1)*p_u+1,1);p((i-1)*p_u+1,2)];
% desired angle
angle = p((i-1)*p_u+1,3);
qd = [yd; angle];

% get desired inputs
ud = [u(i,1);u(i,2)];

% linearization, i.e. \dot (x - xd) = A(x - xd) + B(u - ud)
A = [0 0 -sin(angle);
    0 0 cos(angle);
    0 0 0]*ud(1);

B = [cos(angle) 0;
    sin(angle) 0;
    0 1];

% compute gain matrix for linear system (this solves the Lyapunov
% equation for the linearization dynamics)
[~,~,K] = care(A, B, eye(3));
% set control law
u_ctrl = ud - K*(q(1:3) - qd);
end

%% EduMIP control ODE
function dq = EduMIP_ode(i,p, u, t, q)
u_ctrl = EduMIP_ctrl(i,p, u, t, q);

dq = [cos(q(3))*u_ctrl(1);
    sin(q(3))*u_ctrl(1)
    u_ctrl(2)];
end