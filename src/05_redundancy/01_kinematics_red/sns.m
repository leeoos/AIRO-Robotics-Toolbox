%% Sopravvivenza_AIRO! 
% Author: Massimo, Leonardo, Paolo, Francesco
% 

clc 
close all
clearvars
clear all

lib_path = getenv("ROB2LIB_PATH");
addpath(lib_path);
rob2fun = rob2lib();

%% INPUTS

N = 4; % number of joints

% Symbolic joint variables
syms q [1 N]
q = transpose(q); 

% Other symbolic variables
syms l [1 N]

% Direct kinematics
dir_kin = [
    cos(q1) + cos(q1+q2) + cos(q1+q2+q3) + cos(q1+q2+q3+q4);
    sin(q1) + sin(q1+q2) + sin(q1+q2+q3) + sin(q1+q2+q3+q4);
];

% Configuration for evaluation
current_config = [
    pi/2;
    -pi/2;
    pi/2;
    -pi/2;
];

% Technical bounds:
% Joint position
Q_min = [];
Q_max = [];

% Joint velocities
V_min = [-10; -10; -10; -10];
V_max = [2; 2; 4; 4];

% Joint accelerationsq1 ∈ [−π/2, π/2] , q2 ∈ [0, 2π/3] , q3 ∈ [−π/4, π/4] 
A_min = [];
A_max = [];

% End effector position
ee_psition = [];
ee_velocity = [-4; -1.5];

%% END OF INPUTS

%% PRECOMPUTATION
% Jacobian
J = jacobian(dir_kin, q);
J = subs(J, q, current_config);
disp("evaluated jacobian in current configuration")
J

% TO GENERALIZE Joint velocity limits
Q_dot_min = V_min;
Q_dot_max = V_max;

%% SNS Algorithm
% Preallocation of variables
n = size(q,1);
m = size(ee_velocity,1);
q_bar_dot = zeros(n,1);
q_N_dot = zeros(n,1);
W = eye(n);
limit_exceeded = true;

% Scaling parameters 
s_star = 0;
s = 1;

% Loop to determin
while(limit_exceeded)
    limit_exceeded = false;
    q_bar_dot = q_N_dot + pinv(J*W)*(ee_velocity - J*q_N_dot);
    q_bar_dot = vpa(simplify(q_bar_dot),3);
    disp("Desired velocity")
    q_bar_dot

    % Check if joint velocities bounds are exceeded
    for i = (1 : n)
        if (q_bar_dot(i) < Q_dot_min(i) || q_bar_dot(i) > Q_dot_max(i))
            limit_exceeded = true;
        end
    end

    % Scaling of most critical joint
    if (limit_exceeded)
        a = pinv(J*W)*ee_velocity;
        b = q_bar_dot - a;
        result = getTaskScalingFactor(a, b, n, Q_dot_min, Q_dot_max); 
        tsf = result(1);

        if tsf > s_star 
            s_star = tsf;
            W_star = W;
            q_N_dot_star = q_N_dot;
        end

        j = result(2); % most critical joint
        W(j,j) = 0;

        if q_bar_dot(j) > Q_dot_max(j)
            q_N_dot(j) = Q_dot_max(j);
        elseif q_bar_dot(j) < Q_dot_min(j)
            q_N_dot(j) = Q_dot_min(j);
        end

        if rank(J*W) < m
            s = s_star;
            W = W_star;
            q_N_dot = q_N_dot_star;
            q_bar_dot = q_N_dot + pinv(J*W)*(s*ee_velocity - J*q_N_dot);
            limit_exceeded = false;
        end
    end
end
q_dot_SNS = round(vpa(q_bar_dot),3)

function result = getTaskScalingFactor(a, b, n, Q_dot_min, Q_dot_max)
    S_max = [];
    S_min = [];
    
    for i = (1: n)
        S_min = [S_min; (Q_dot_min(i) - b(i)) / a(i)];
        S_max = [S_max; (Q_dot_max(i) - b(i)) / a(i)];

        if (S_min(i) > S_max(i))
            foo = S_min(i);
            S_min(i) = S_max(i);
            S_max(i) = foo;
        end
    end

    s_max = min(S_max);
    s_min = max(S_min);

    most_critcal_joint = find(S_max == s_max);

    if (s_min > s_max|| s_max < 0 || s_min > 1)
        tsf = 0;
    else
        tsf = s_max;
    end
    result = [tsf; most_critcal_joint];
end
