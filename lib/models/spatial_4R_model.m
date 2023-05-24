%% DATASHEET FOR A PLANAR 4R 
% Insert in the following file all the inputs necessary to compute the
% dynamic model of a planar 4R spatial robot

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
    0;
    0;
    0;
];

% Direct kinematics.
DHTABLE = [        
    0   L1   0   q1;
    0   L2   0   q2;
    0   L3   0   q3;
    0   L4   0   q4;
];

% Vectors of CoM relative to Reference Frame i
R_CoM = {
    [
        -L1+d1;
        0;
        0;
        1;
    ], ... 
    [
        -L2+d2;
        0;
        0;
        1;
    ], ...
    [
        -L3+d3;
        0;
        0;
        1;
    ], ... 
    [
        -L4+d4;
        0;
        0;
        1;
    ]
};
