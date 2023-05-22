%% DATASHEET FOR A ...
% Insert in the following file all the inputs necessary to compute the
% dynamic model of a ... robot

lib_path = getenv("ROB2LIB_PATH");
addpath(lib_path);
rob2fun = rob2lib();

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
    [
        -d1*sin(q1)*q_dot1; 
        d1*cos(q1)*q_dot1;
        0;
    ], ... 
    [
        q_dot2*cos(q1) - q2*sin(q1)*q_dot1; 
        q_dot2*sin(q1) + q2*cos(q1)*q_dot1; 
        0;
    ], ...
};
% uncomment for debug
celldisp(CoM_VELOCITY)

% Experimetal
CoM_VELOCITY_test = {
    rob2fun.time_derivative(W_CoM{1}, q, q_dot), ...
    rob2fun.time_derivative(W_CoM{2}, q, q_dot), ...
}

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