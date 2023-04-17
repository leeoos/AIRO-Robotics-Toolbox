%% Sopravvivenza_AIRO! 
% Author: Massimo, Leonardo, Paolo, Francesco
clear all
close all
clc

% Definition of Jacobian

q_syms = (sym('q', [1 3]))

q1 = q_syms(1)
q2 = q_syms(2)
q3 = q_syms(3)


J = [-sin(q1), -sin(q2), -sin(q3); cos(q1), cos(q2), cos(q3); 0, 1, 0]

% Singularity Computation
if (size(J, 1) < size(J,2))
    detJ = simplify(det(J*transpose(J)))
elseif (size(J,1) == size(J,2))
    detJ = simplify(det(J))
end

%% TASK DEFINITION

dr = [0; 1; 0]
J = subs(J, {q1,q2,q3}, {0.5480, 0, 0.5480})

%% PSEUDOINVERSE SOLUTION
J_pinv_sym = simplify(pinv(J))
J_pinv = vpa(pinv(J),3)
dq_ps = vpa((J_pinv*dr),3)

%% DAMPED LEAST SQUARE SOLUTION
Jt = vpa(transpose(J),3);
mu = 0.25;
J_dls = vpa((Jt*(J*Jt+mu*eye(3))^-1),3)
dq_dls = vpa((J_dls*dr),3)

%% WEIGHTED PSEUDOINVERSE
W = [1, 0, 0; 0, 1 , 0; 0, 0, 1];

if (rank(J) == size(J,1))
    invW = inv(W)
    Jw = invW*Jt*(inv(J*invW*Jt))
    dq_wpinv = vpa((Jw* dr),3)

else 
    invW = sqrt(inv(W))
    J_aux = vpa((J*invW),3)
    Jw = vpa((invW*pinv(J_aux)),3)
    dq_wpinv = vpa((Jw* dr),3)
end


