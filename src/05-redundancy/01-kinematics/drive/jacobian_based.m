%% Sopravvivenza_AIRO! 
% Author: Massimo, Leonardo, Paolo, Francesco
% Implementation of differents jacobian based methods
% to handle kinematic redundancy of robot manipulators

close all
clear all
clc

%% INPUTS

N = 3; % number of joints

syms q [1 N]


% Insert the jacobian
J = [
    -sin(q1), -sin(q2), -sin(q3); cos(q1), cos(q2), cos(q3); 
    0, 1, 0
];
J_sym = J;

% Task velocities
r_dot = [
    0; 
    1; 
    0;
];

% Current configuration
configuration = [0.5480, 0, 0.5480];

% Evaluation of jacobian
J = subs(J, q, configuration);
disp("Evaluated jacobian")
J

%% END OF INPUTS

%% Singularity computation
% Uncomment to print symbolic pvin of J
simplify(pinv(J_syms))

disp("Algorithmic singulatity at")
if (size(J_sym, 1) < size(J,2))
    det_J = simplify(det(J_sym*transpose(J_sym)));
    det_J
elseif (size(J_sym,1) == size(J_sym,2))
    det_J = simplify(det(J_sym));
    det_J
end

%% PSEUDOINVERSE SOLUTION
J_pinv = vpa(pinv(J),3);
q_dot_ps = vpa((J_pinv*r_dot),3);
disp("Pseudoinverse solution")
J_pinv
q_dot_ps

%% DAMPED LEAST SQUARE SOLUTION
Jt = vpa(transpose(J),3);
mu = 0.25; % TO CHANGE IF NEEDED
J_dls = vpa((Jt*(J*Jt+mu*eye(3))^-1),3);
q_dot_dls = vpa((J_dls*r_dot),3);
disp("Damped least square solution")
J_dls
q_dot_dls

%% WEIGHTED PSEUDOINVERSE
W = [
    1, 0, 0; 
    0, 1, 0; 
    0, 0, 1;
]; % TO CHANGE IF NEEDED

disp("Weighted pseudoinverse solution")
if (rank(J) == size(J,1))
    % if J is full rank we simply use the pseudo inverse of J
    invW = inv(W);
    Jw = invW*Jt*(inv(J*invW*Jt));
    q_dot_wpinv = vpa((Jw*r_dot),3);
    Jw
    q_dot_wpinv
else 
    % otherwise we use the weighted pseudoinverse method
    invW = sqrt(inv(W));
    J_aux = vpa((J*invW),3);
    Jw = vpa((invW*pinv(J_aux)),3);
    q_dot_wpinv = vpa((Jw*r_dot),3);
    Jw
    q_dot_wpinv
end


