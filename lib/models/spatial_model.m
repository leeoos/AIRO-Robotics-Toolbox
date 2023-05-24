%% DATASHEET FOR A ...
% Insert in the following file all the inputs necessary to compute the
% dynamic model of a ... spatial robot


%% INPUTS 
% PAY ATTENTION: update for each problem!

N = 4; % number of joints

% Suppose diagonal ineria matrix for each link
I_diag = true; 

% Load symbols in the workspace
run('rob2symb.m')

% Inertia matrix for each link
%celldisp(I) % uncomment for debug

% Definition of gravity vector
% Mind the position of gravity
g = [
    0;
    g0;
    0;
];

% sigma vector:
% 0: revolout 
% 1: prismatic
sigma = [
    0;
    1;
    0;
];

% Direct kinematics
DHTABLE = [        
    pi/2    0      L1   q1;
    0       0      L2   q2;
    0       L3     0   q3;
];


syms V F D C E
% Vectrors of the centers of mass
R_CoM = {
    [   
        0;
        -L1+d1;
        0;
    ], ...
    [
        0;
        q2+d2;
        0;
    ], ...
    [
        -L3+d3;
        0;
        0;
    ], ...
};

% Initial conditions for angular velocity
initial_omega = [
    0;
    0;
    0;
];
OMEGA = cell(1, N);
OMEGA{1} = initial_omega;

% Initial conditions for linear velocity
initial_velocity = [
    0;
    0;
    0;
];
VELOCITY = cell(1, N);
VELOCITY{1} = initial_velocity;

