%% Sopravvivenza_AIRO! 
% Authors: Massimo, Leonardo, Paolo, Francesco
% Implementation of Koning Theorem to compute the 
% dynamic model of a generic planar robot,
% given the pose of CoM and angular velocities.

clc 
close all
clear all

my_path = getenv("ROB2LIB_PATH");
addpath(my_path);
FunObj = Rob2Lib();

%% INPUT
% PAY ATTENTION: update for each problem!

N = 4; % number of joints

% Standard symbolic variables
syms q [1 N]
syms q_dot [1 N]
syms m [1 N]
syms L [1 N]
syms d [1 N]
syms t

% q(t) = symfun(q(t), t) % experimental

% Inertia Matricies (Evaluate if you have numbers!)
% Set this value to false if you want to use a full inertia matrix
diagonal_inertia = true
% X
syms Ixx [1 N]
syms Ixy [1 N] 
syms Ixz [1 N]
% Y
syms Iyx [1 N]
syms Iyy [1 N]
syms Iyz [1 N]
% Z
syms Izz [1 N]
syms Izx [1 N]
syms Izy [1 N]

I = cell(1,3);
if diagonal_inertia
    for i =(1:N)
        I{i} = [
            Ixx(i), 0, 0;
            0, Iyy(i), 0;
            0, 0, Izz(i);
        ];
    end
else 
    for i =(1:N)
        I{i} = [
            Ixx(i), Ixy(i), Ixz(i);
            Ixy(i), Iyy(i), Iyz(i);
            Ixz(i), Iyz(i), Izz(i);
        ];
    end
end

%celldisp(I) % uncomment for debug

% PAY ATTENTION: this part require a precomputation that depends on the problem
% Vectors of the centers of masses w.r.t world frame
CoM_VELOCITY = {
    [q_dot1; 0; 0], ... 
    [q_dot1; q_dot2; 0], ...
    [q_dot1 - d3*sin(q3)*q_dot3; 
     q_dot2 + d3*cos(q3)*q_dot3; 
     0], ...
    [q_dot1 - L3*sin(q3)*q_dot3 - d4*sin(q3+q4)*(q_dot3+q_dot4); 
     q_dot2 + L3*cos(q3)*q_dot3 + d4*cos(q3+q4)*(q_dot3+q_dot4); 
     0], ...
};
% uncomment for debug
celldisp(CoM_VELOCITY)

% Vectors of the centers of masses w.r.t world frame
OMEGA = {
    [0; 0; 0], ... 
    [0; 0; 0], ...
    [0; 0; q_dot3], ...
    [0; 0; q_dot3 + q_dot4], ...
};
% uncomment for debug
celldisp(OMEGA)


% Angular velocities of each link

%% END OF INPUTS

% Implementation of Koening theorem for each link
T = cell(1, N);
for i = (1:N) 
     T{i} = (1/2*m(i)*transpose(CoM_VELOCITY{i})*CoM_VELOCITY{i} + ...
         + (1/2*transpose(OMEGA{i})*I{i}*OMEGA{i}));
     T{i} = collect(simplify(expand(T{i})),q_dot.^2); %[q_dot1^2, q_dot2^2, q_dot3^2, q_dot4^2]);
     T{i} = collect(T{i},m);
end
celldisp(T)

KINETIC_ENERGY = sum([T{:}])
 
