%% Sopravvivenza_AIRO! 
% Author: Massimo, Leonardo, Paolo, Francesco
% 

clc
close all
syms q1 q2 q3 q4 

%% Inputs to the problem
joints = [q1; q2; q3];

dir_kin = [
    cos(q1) + cos(q1+q2) + cos(q1+q2+q3);
    sin(q1) + sin(q1+q2) + sin(q1+q2+q3);
];

current_config = [
    2*pi/5;
    pi/2;
    -pi/4;
];

% Technical bounds
% Joint position

Q_min = [-pi/2; 0; -pi/4];
Q_max = [pi/2; 2*pi/3; pi/4];

% Joint velocities
V_min = [-2; -2; -2];
V_max = [2; 2; 2];

% Joint accelerationsq1 ∈ [−π/2, π/2] , q2 ∈ [0, 2π/3] , q3 ∈ [−π/4, π/4] 
A_min = [];
A_max = [];

ee_psition = [];
ee_velocity = [-3; 0];

%% Precomputation

% Jacobian
J = jacobian(dir_kin, joints)
J = subs(J, joints, current_config);

%% SNS Algorithm

% Preallocation of variable
n = size(joints,1);
m = size(ee_velocity,1);
s_star = 0;
s = 1;
q_bar_dot = zeros(n,1);
q_N_dot = zeros(n,1);
W = eye(n);
limit_exided = true;

while(limit_exided)
    limit_exided = false;
    q_bar_dot = q_N_dot + pinv(J*W)*(ee_velocity - J*q_N_dot);
    q_bar_dot = myLibrary.rvs(q_bar_dot);

    for i = (1 : n)
        if (q_bar_dot(i) < Q_min(i) || q_bar_dot(i) > Q_max(i))
            limit_exided = true;
        end
    end

    if (limit_exided)
        a = pinv(J*W)*ee_velocity;
        b = q_bar_dot - a;
        result = getTaskScalingFactor(a, b, n, V_min, V_max); % check if vmax == qmax
        tsf = result(1);

        if tsf > s_star 
            s_star = tsf;
            W_star = W;
            q_N_dot_star = q_N_dot;
        end

        j = result(2); % most critical joint
        W(j,j) = 0;

        if q_bar_dot(j) > V_max(j)
            q_N_dot(j) = V_max(j);
        elseif q_bar_dot(j) < V_min(j)
            q_N_dot(j) = V_min(j);
        end

        if rank(J*W) < m
            s = s_star;
            W = W_star;
            q_N_dot = q_N_dot_star;
            q_bar_dot = q_N_dot + pinv(J*W)*(s*ee_velocity - J*q_N_dot);
            limit_exided = false;
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
