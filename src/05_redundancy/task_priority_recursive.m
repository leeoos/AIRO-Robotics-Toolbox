%% Sopravvivenza_AIRO! 
% Author: Massimo, Leonardo, Paolo, Francesco
% This script compute the best joint velocity for a 
% set task with priorities. 

close all
clear all
clc

my_path = getenv("ROB2LIB_PATH");
addpath(my_path);
FunObj = Rob2Lib();

%% INPUTS 
N = 3; % number of joints

% Standard symbolic variables
syms q [1 N] 

% Vector of augmented end effector velocities for k tasks
% Pay attention to the order of the priorities of the tasks
%r_A = {[0.5;0.5], 0, 0};
r_A = {0, [2;-1]};
NUM_OF_TASKS = size(r_A,2);

% Jacobians for the relative tasks

% J_1 = [
%     -(sin(q1)+sin(q2+q1) + sin(q1+q2+q3)), - (sin(q1+q2) + sin(q1+q2+q3)), -sin(q1+q2+q3);
%     (cos(q1)+cos(q2+q1) + cos(q1+q2+q3)), (cos(q1+q2) + cos(q1+q2+q3)), cos(q1+q2+q3);
% ];
% 
% J_2 = [
%     1,0,0;
% ];
% 
% J_3 = [
%      1,1,0;
% ];

J_1 = [
     1,1,1
];

J_2 = [
     - sin(q1 + q2 + q3) - sin(q1 + q2) - sin(q1), - sin(q1 + q2 + q3) - sin(q1 + q2), -sin(q1 + q2 + q3);
     cos(q1 + q2 + q3) + cos(q1 + q2) + cos(q1),   cos(q1 + q2 + q3) + cos(q1 + q2),  cos(q1 + q2 + q3);
];

N = size(J_1, 2); % Number of joints

% Remember to evaluate all the simbolic Jacobians before 
% the beginning of the loop. Also use round and vpa.
%J_1 = round(vpa(subs(J_1, {q1,q2,q3}, {0, pi, 0}),3),5);
J_2 = round(vpa(subs(J_2, {q1,q2,q3}, {pi/4, 0, pi/4}),3),5);

% Augmented Jacobian
J_A = {J_1, J_2};

%% END OF INPUTS

% Initialization of joint velocities
q_dot = zeros(N, 1); 

% Accumulation matrix for J_A, contains J augmented until step k  
J_A_k = [];
P_A = cell(1,N);

eps = 10^-50; % inferior limit for zero equality

full_rank_J_A = 0;
for i = (1:size(J_A,2))
    full_rank_J_A = full_rank_J_A + size(J_A{i},1);
end
full_rank_J_A

% 'recursive' step to update q_dot and P_A up to task k
% see http://www.diag.uniroma1.it/deluca/rob2_en/02_KinematicRedundancy_2.pdf
% slides number: from 2 to 5
for k = (1 : NUM_OF_TASKS)
    disp(["Step", k])

    J_A_k = [J_A_k; J_A{k}] % accumulator
    J_k = J_A{k} % relative jacobian
    r_k = r_A{k} % relative ee task velocity
    
    % First task, inzialization of q_dot and PA_0
    if (k == 1)
        P_A0 = eye(N);
        q_dot = q_dot + pinv((J_k*P_A0))*(r_k - J_k*q_dot)
        P_A{k} = eye(N) - pinv(J_k*eye(N))*J_k*eye(N);

    else
        if (rank(J_A_k) < full_rank_J_A)
            disp(["J_A_", k, " not full rank"])
            
            % Damped least square
            J_SVD = svd(J_A_k);
            mu = (1/1000)*min(J_SVD(J_SVD>eps)); % 1 over min of J_svd different from zero
            mu = 0.001;
            J_DLS = transpose(J_k*P_A{k-1}) * inv(J_k*P_A{k-1}*transpose(J_k*P_A{k-1}) + mu*eye(size(J_k*P_A{k-1},1)));
            
            % damped least square for q dot 
            q_dot = q_dot + J_DLS*(r_k - J_k*q_dot)

            % computation of new null space projector
            J_NULL = null(eval(J_k*P_A{k-1}));
            P_NULL = J_NULL* inv(transpose(J_NULL) * J_NULL) * transpose(J_NULL);
            P_A{k} = P_A{k-1} - P_NULL; 
            
        else
            q_dot = q_dot + pinv((J_k*P_A{k-1}))*(r_k - J_k*q_dot)
            P_A{k} = P_A{k-1} - pinv(J_k*P_A{k-1})*J_k*P_A{k-1};
        end
    end
    disp("----")
end

q_dot_final = round(simplify(q_dot),6)

function dls_res = compute_q_dls(mu, J, q_dot, r_dot)
    J_DLS = transpose(J) * inv(J*transpose(J) + mu*eye(size(J,1)));
    dls_res = q_dot + J_DLS*(r_dot - J*q_dot);
end




