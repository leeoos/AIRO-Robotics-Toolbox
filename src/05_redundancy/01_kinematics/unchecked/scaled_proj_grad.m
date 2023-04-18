%% Sopravvivenza_AIRO! 
% Author: Massimo, Leonardo, Paolo, Francesco
% This script compute the best joint velocity to 
% complete assigned task, taking in account some 
% kinematics bounds on the joint velocities

close all
clearvars
clear all
clc

lib_path = getenv("ROB2LIB_PATH");
addpath(lib_path);
rob2fun = rob2lib();

%% INPUTS
% PAY ATTENTION: update for each problem!

N = 3; % number of joints

syms q [1 N]
syms l [1 N]

% trasposition of q vector
q = transpose(q);
disp(q)

% Task desired velocity, change depending on your task
r_dot = [
    -3;
    0;
];
disp(r_dot)

% Direct kinematics of the robot
r = [
      cos(q1) + cos(q1+q2) + cos(q1+q2+q3);
      sin(q1) + sin(q1+q2) + sin(q1+q2+q3);
];

% Given configuration for evaluation
configuration = [
    2*pi/5;
    pi/2;
    -pi/4;
];

% Scaling factor for q_dot_sns
s = 0.96;

% Joint range limits
q_range = {[-pi/2, pi/2],[0,2*pi/3],[-pi/4,pi/4]};

%% END OF INPUTS

fprintf("This is the symbolic Jacobian: \n")
J = jacobian(r, q);
disp(J)

% Singularity Computation
if (size(J, 1) < size(J,2))
    detJJt = simplify(det(J*transpose(J)));

elseif (size(J,1) == size(J,2))
    detJ = simplify(det(J));
end

% Evaluation of Jacobian matrix
fprintf("This is the Jacobian evaluated: \n")
J_eval = vpa(subs(J, q, configuration),3);
disp(J_eval)

% Computation of pseudoinvers of J
fprintf("This is your pseudoinverse evaluated: \n")
J_pinv = vpa(pinv(J_eval),3);
disp(J_pinv)

%% Computation of gradient of h => joint range optimization function

% Computation of q_midpoint
q_midpoint = zeros(1,N);
H_q = 0;
for i=1:N
    q_tmp = (q_range{i}(1)+q_range{i}(2))/2;
    q_midpoint(i) = q_tmp; %[q_midpoint, q_tmp];
    H_q = H_q + ((q(i) - q_midpoint(i))/(q_range{i}(1)-q_range{i}(2)))^2;
end


fprintf("This is your optimization function H(q): \n")
H_q = (1/(2*N))* H_q;
disp(H_q)

% Computation of gradient of H(q)
% Change the sign of gradient depending on 
% "maximization" (+) 
% "minimization" (-)
fprintf("These is the symbolic gradient of H(q): \n")
grad_H = simplify(-gradient(H_q,q));
disp(grad_H)

fprintf("These is the evaluated gradient of H(q): \n")
grad_H = vpa(simplify(subs(grad_H, q, configuration)),3);
disp(grad_H)

%% Computation of lite sns
fprintf("This is the q_dot result by lite sns method: \n")
q_dot_sns = vpa(grad_H + J_pinv*(s*r_dot - J_eval*grad_H),4);
disp(q_dot_sns)


%% TASK SCALING

% % Insert your joint velocity bounds
% q_dot_limit = [4,4,4,4];
% 
% % Computation of s parameter for task scaling
% [max_q, i] = max(q_dot) %Take the maximum of q velocity and its index
% 
% if max_q > q_dot_limit(i)
%     s = vpa((q_dot_limit(i) - grad_H_eval(i))/(max_q-grad_H_eval(i)),4)
% 
% % q_dot computation with task scaling
% fprintf("This is the q_dot result by PG method with task scaling:")
% q_dot = vpa(grad_H_eval + J_pinv*(s*r_dot - J_eval*grad_H_eval),4)
% end
