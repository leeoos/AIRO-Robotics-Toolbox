%% DATASHEET FOR A ...
% Insert in the following file all the inputs necessary to compute the
% dynamic model of a ... robot

% PAY ATTENTION: this part require a precomputation that depends on the problem
% Vectors of the centers of masses w.r.t world frame
W_CoM = {
    [
        q1; 
        0; 
        0;
    ], ...
    [
        q1; 
        q2; 
        0;
    ], ...
    [   
        q1 + q3;
        q2;
        0;
    ], ...
    [   
        q1 + q3;
        q2 + q4;
        0;
    ], ...
};
% uncomment for debug
celldisp(W_CoM)

% Velocities of the centers of masses w.r.t world frame
CoM_VELOCITY = {
    [
        q_dot1; 
        0; 
        0;
    ], ... 
    [
        q_dot1; 
        q_dot2; 
        0;
    ], ...
    [   
        q_dot1 + q_dot3; 
        q_dot2; 
        0;
    ], ...
    [   
        q_dot1 + q_dot3; 
        q_dot2 + +q_dot4; 
        0;
    ], ...
};
% uncomment for debug
celldisp(CoM_VELOCITY)

% % Angular velocities of each link
OMEGA = {
    [
        0; 
        0; 
        0;
    ], ... 
    [
        0; 
        0; 
        0;
    ], ...
    [
        0; 
        0; 
        0;
    ], ...
    [
        0; 
        0; 
        0;
    ], ...
};
% uncomment for debug
celldisp(OMEGA)
