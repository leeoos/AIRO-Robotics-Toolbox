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
        (q1+k2) + d2*cos(q2); 
        d2*sin(q2); 
        0;
    ], ...
    [
        (q1+k2) + q3*cos(q2); 
        q3*sin(q2); 
        0;
    ], ...
};
% uncomment for debug
celldisp(W_CoM)

% Velocities of the centers of masses w.r.t world frame (derivative of W_CoM)
CoM_VELOCITY = {
    [
        q_dot1; 
        0;
        0;
    ], ... 
    [
        q_dot1 - d2*sin(q2)*q_dot2;
        d2*cos(q2)*q_dot2; 
        0;
    ], ...
    [
        q_dot1 + q_dot3*cos(q2) - q3*sin(q2)*q_dot2;
        q_dot3*sin(q2) + q3*cos(q2)*q_dot2;  
        0;
    ], ...
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
        q_dot2;
    ], ...
};
% uncomment for debug
celldisp(OMEGA)