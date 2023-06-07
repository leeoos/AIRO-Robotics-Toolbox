%% DATASHEET FOR A PRRRn PLANAR ROBOT
% Insert in the following file all the inputs necessary to compute the
% dynamic model of a ... robot

lib_path = getenv("ROB2LIB_PATH");
addpath(lib_path);
rob2fun = rob2lib();

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
    0;
    g0;
];

syms h

% PAY ATTENTION: this part require a precomputation that depends on the problem
% Vectors of the centers of masses w.r.t world frame
W_CoM = {
    [
        q1; 
        d1; 
        0;
    ], ...
    [
        q1 + d2*cos(q2); 
        h + d2*sin(q2); 
        0;
    ], ...
    [
        q1 + l2*cos(q2) + d3*cos(q3); 
        h + l2*sin(q2) + d3*sin(q3); 
        0;
    ], ...
    [
        q1 + l2*cos(q2) + l3*cos(q3) + d4*cos(q4); 
        h + l2*sin(q2) + l3*sin(q3) + d4*sin(q4); 
        0;
    ], ...
};
% uncomment for debug
celldisp(W_CoM)

% Velocities of the centers of masses w.r.t world frame (derivative of W_CoM)
CoM_VELOCITY = {
    rob2fun.time_derivative(W_CoM{1}, q, q_dot), ...
    rob2fun.time_derivative(W_CoM{2}, q, q_dot), ...
    rob2fun.time_derivative(W_CoM{3}, q, q_dot), ...
    rob2fun.time_derivative(W_CoM{4}, q, q_dot), ...
};
% uncomment for debug
celldisp(CoM_VELOCITY)

% Angular velocities of each link
OMEGA = {
    [
        0; 
        0; 
        0;
    ], ... 
    [
        0; 
        0; 
        q_dot2;
    ], ...
    [
        0; 
        0; 
        q_dot3;
    ], ...
    [
        0; 
        0; 
        q_dot4;
    ], ...
};
% uncomment for debug
celldisp(OMEGA)