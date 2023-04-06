%% Sopravvivenza_AIRO! 
% Author: Massimo, Leonardo, Paolo, Francesco
% This script compute the best joint velocity for a 
% set task with priorities. 

addpath('/home/leeoos/MATLAB-Drive/Robotics2Scripts/');
FunObj = Rob2Lib();

close all
clc
syms q1 q2 q3

%% INPUTS for the problem
% Pay attention: change this values according to the problem

NUM_OF_TASKS = 2;
N = 3; % Number of joints

% Vector of augmented end effector velocities for k tasks
% Pay attention to the order of the priorities of the tasks
r_A = {[0], [2;-1]};

% Jacobians for the relative tasks
J_2 = [
     - sin(q1 + q2 + q3) - sin(q1 + q2) - sin(q1), - sin(q1 + q2 + q3) - sin(q1 + q2), -sin(q1 + q2 + q3);
     cos(q1 + q2 + q3) + cos(q1 + q2) + cos(q1),   cos(q1 + q2 + q3) + cos(q1 + q2),  cos(q1 + q2 + q3);
];

J_1 = [
     1,1,1
];

% Remember to evaluate all the simbolic Jacobians before 
% the beginning of the loop. Also use round and vpa.
J_2 = round(vpa(subs(J_2, {q1,q2,q3}, {pi/4, 0, pi/4}),3),5);

% Augmented Jacobian
J_A = {J_1; J_2};

%% END OF INPUTS

% Initialization of joint velocities
q_dot = zeros(N, 1); 

% Accumulation matrix for J_A, contains J augmented until step k  
J_A_k = [];

for k = (1 : NUM_OF_TASKS)

    J_A_k = [J_A_k; J_A{k}] % accumulaton
    J_k = J_A{k} % relative jacobian
    r_k = r_A{k} % relative ee task velocity
    
    % First task, inzialization of q_dot and PA_0
    if (k == 1)
        P_A0 = eye(N);
        q_dot = q_dot + pinv((J_k*P_A0))*(r_k - J_k*q_dot)
        P_A{k} = eye(N) - pinv(J_k*eye(N))*J_k*eye(N);
  
    else

        if det(J_A_k) == 0
             mu = 0.001;P_A{k} = P_A{k-1};
%             J_P = J_k*P_A{k-1}
%             J_DLS = transpose(J_P) * inv(J_P*transpose(J_P) + mu*eye(size(J_P,1))) 
%             q_dot = q_dot + J_DLS*(r_A{k} - J_A{k}*q_dot);
%             P_A{k} = P_A{k-1};
              q_dot = FunObj.dls(mu, J_k*P_A{k-1}, q_dot, r_k)
              P_A{k} = P_A{k-1};
        else
            q_dot = q_dot + pinv(eval(J_k*P_A{k-1}))*(r_k - J_k*q_dot)
            P_A{k} = P_A{k-1} - pinv(eval(J_k*P_A{k-1}))*J_k*P_A{k-1};
        end
    end

end

res = round(vpa(simplify(q_dot),4),4)
