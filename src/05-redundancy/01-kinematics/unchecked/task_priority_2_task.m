    %% Sopravvivenza_AIRO! 
% Author: Massimo, Leonardo, Paolo, Francesco

clear all
close all
clc

syms q1 q2 q3

% This script compute the dq velocity of joints for 2 tasks with priority

%% DEFINITION OF TASKS
% Define your task with priority (Example: dr1 has higher priority wrt dr2)
% Example of definition:
% dr1 = [0;1] -> Task with higher priority
% dr2 = [0] -> Task with lower priority 

dr1 = [0]
dr2 = [2;-1]

%% DEFINITION OF JACOBIANS
% Define the associated jacobians (Example: J1 is the jacobian of task dr1)
% Example of definition:
% J1 = [-sin(q1), -sin(q2), -sin(q3); cos(q1), cos(q2), cos(q3)]
% J2 = [0,1,0]

J1 = [1,1,1]

J2 = [-sin(q1)-sin(q1+q2)-sin(q1+q2+q3), -sin(q1+q2)-sin(q1+q2+q3), -sin(q1+q2+q3);
      cos(q1)+cos(q1+q2)+cos(q1+q2+q3),  cos(q1+q2)+cos(q1+q2+q3),  cos(q1+q2+q3) ]

% Evaluate the symbolic Jacobians with your values
J2 = vpa(subs(J2, {q1,q2,q3}, {pi/4, 0, pi/4}),3) 

JA = [J1;J2]

pinvJ1 = vpa(pinv(J1),3)
pinvJ2 = vpa(pinv(J2),3)


%% COMPUTATION OF VELOCITY

% Projector Matrix
P1 = vpa((eye(3)-pinvJ1*J1),3)
J2P1 = round(vpa((J2*P1),4),5)

% Check if we are in an algorithmic singularity (the matrix J2*P1 is zeros)
% In fact in this case we use the damped least square solution
% because the pseudoinverse is not computable.
% Otherwise we use the pseudoinverse solution.
% if ()
%     mu = 0.001;
%     J2P1t = transpose(J2P1);
%     J2P1_dls = J2P1t*(J2P1*J2P1t + mu*eye(size(J2P1*J2P1t,1)))^-1;
%     dq = round(vpa(pinvJ1*dr1 + J2P1_dls*(dr2 - J2*pinvJ1*dr1),3),6)
% else

    dq = round(vpa(pinvJ1*dr1 + pinv(eval(J2P1))*(dr2 - J2*pinvJ1*dr1),3),6)



