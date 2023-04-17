%% Sopravvivenza_AIRO! 
% Author: Massimo, Leonardo, Paolo, Francesco

clear all
close all
clc

%% DATA DEFINITION

syms l q1 q2 q3 q4 

% PUT YOUR NUMBER OF JOINTS
N = 4;

q = sym("q", [1 N]);

% CHANGE DEPENDING ON YOUR TASK
r_dot = [1;0;0.5];

% CHANGE DEPENDING ON YOUR DIRECT KINEMATICS
r = [
      l*(cos(q1) + cos(q1+q2) + cos(q1+q2+q3) + cos(q1+q2+q3+q4));
      l*(sin(q1) + sin(q1+q2) + sin(q1+q2+q3) + sin(q1+q2+q3+q4));
      q1 + q2 + q3 + q4;
];

fprintf("This is the symbolic Jacobian:")
J = jacobian(r, q)

% Singularity Computation
if (size(J, 1) < size(J,2))
    detJJt = simplify(det(J*transpose(J)))
elseif (size(J,1) == size(J,2))
    detJ = simplify(det(J))
end

% CHANGE DEPENDING ON YOUR CONFIGURATION
fprintf("This is the Jacobian evaluated:")
J_eval = subs(J, {l,q1,q2,q3,q4}, {0.5,0, 0, pi/2, 0})

% PINV COMPUTATION
fprintf("This is your pseudoinverse evaluated:")
J_pinv = vpa(pinv(J_eval),3)

%% COMPUTATION OF GRADIENT OF H => JOINT RANGE OPTIMIZATION FUNCTION
% This section compute the optimization function H(q) depending on "Joint 
% limits range".

% CHANGE ONLY:
% 1) q_range => Write all joints range in this cell array
q_range = {[2,-2],[2,-2],[2,-2],[2,-2]};

q_midpoint = [];
H_q = 0;
for i=1:N
    q_tmp = (q_range{i}(1)+q_range{i}(2))/2;
    q_midpoint = [q_midpoint, q_tmp];
    H_q = H_q + ((q(i) - q_midpoint(i))/(q_range{i}(1)-q_range{i}(2)))^2;
end

fprintf("This is your optimization function H(q):")
H_q = (1/(2*N))* H_q

%% COMPUTATION OF JOINT VELOCITY

% CHANGE THE SIGN OF GRADIENT DEPENDING ON "MAXIMIZATION" (+) OR
% "MINIMIZATION" (-)

% Computation of gradient of H(q)
fprintf("These are the symbolic and evaluated gradient of H(q):")
grad_H = -transpose(jacobian(H_q,q))
grad_H = subs(grad_H, {q1,q2,q3,q4}, {0, 0, pi/2, 0})

fprintf("This is the q_dot result by PG method:")
q_dot = vpa(grad_H + J_pinv*(r_dot - J_eval*grad_H),4)