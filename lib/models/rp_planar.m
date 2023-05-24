%% DATASHEET FOR A ...
% Insert in the following file all the inputs necessary to compute the
% dynamic model of a ... robot

lib_path = getenv("ROB2LIB_PATH");
addpath(lib_path);
rob2fun = rob2lib();

%% INPUTS
% PAY ATTENTION: update for each problem!

N = 2; % number of joints

% Suppose diagonal ineria matrix for each link
I_diag = true; 

% Load symbols in the workspace
run('rob2symb.m')

% Inertia matrix for each link
%celldisp(I) % uncomment for debug

% Definition of gravity vector
% Mind the position of gravity
g = [
    -g0;
    0;
    0;
];

% PAY ATTENTION: this part require a precomputation that depends on the problem
% Vectors of the centers of masses w.r.t world frame
W_CoM = {
    [
        d1*cos(q1); 
        d1*sin(q1); 
        0;
    ], ...
    [
        q2*cos(q1); 
        q2*sin(q1); 
        0;
    ], ...
};
% uncomment for debug
celldisp(W_CoM)

% Velocities of the centers of masses w.r.t world frame (derivative of W_CoM)
CoM_VELOCITY = {
    rob2fun.time_derivative(W_CoM{1}, q, q_dot), ...
    rob2fun.time_derivative(W_CoM{2}, q, q_dot), ...
};
% uncomment for debug
celldisp(CoM_VELOCITY)

% Angular velocities of each link
OMEGA = {
    [
        0; 
        0; 
        q_dot1;
    ], ... 
    [
        0; 
        0; 
        q_dot1;
    ], ...
};
% uncomment for debug
celldisp(OMEGA)