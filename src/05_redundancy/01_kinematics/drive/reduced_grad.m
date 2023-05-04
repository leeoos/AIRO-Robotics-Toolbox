%% Sopravvivenza_AIRO! 
% Author: Massimo, Leonardo, Paolo, Francesco
clc
clear all
close all

N = 3; %to change
syms q [1 N]
syms l [1 N]

%% INPUT 

% Direct Kinematics 
r = [l1*cos(q1) + l2*cos(q1+q2) + l3*cos(q1+q2+q3);
     l1*sin(q1) + l2*sin(q1+q2) + l3*sin(q1+q2+q3)]; %to change

% Optimization function
H_q = sin(q2)^2 + sin(q3)^2; %to change

% Definition of desired velocity
r_dot = [1; -sqrt(3)]; %to change

% Definition of link lenghts and current configuration
l_eval = [1, 1, 1]; %to change
q_eval = [pi/2, pi/3, -2*(pi/3)]; %to change

% Task Jacobian
J = jacobian(r,q)
J_eval = vpa(subs(J, q, q_eval),3);
J_eval = vpa(subs(J_eval, l, l_eval),3)

% Definition of Ja and Jb Matrix
% Ja => Take a FULL ROW RANK MATRIX OF DIMENSION MxM (SQUARE !!!) (Task Dimension)
Ja = vpa([J_eval(:,1), J_eval(:,3)],3)  

% Change "i" in J_eval(:,i) to take the i-esim column to compose Ja
i = [1,3]; % Write the same index of previous case  
Ja_inv = vpa(Ja^-1,3)
Jb=[];
for k=1:N
    if ~ismember(k,i)
         i= [i,k];
         Jb = [Jb,J_eval(:,k)];
    end
end
Jb
%% END OF INPUTS

%% COMPUTATION OF REDUCED GRADIENT SOLUTION 

% Computation of H(q)
grad_H = gradient(H_q, [q1;q3;q2])
% Evaluation of gradient of H(q)
grad_H_eval = vpa(subs (grad_H, q, q_eval),3)

q_dot = vpa(([Ja_inv*r_dot;0] + [-Ja_inv*Jb; eye(N-size(J,1))]*[-transpose(Ja_inv*Jb), eye(N-size(J,1))]*grad_H_eval),3);

q_dot_ordered = [];
for j=1:N
    q_dot_ordered = [q_dot_ordered; q_dot(i(j))];
end

fprintf("This is the q_dot of reduced gradient method: \n")
vpa(q_dot_ordered,3)

