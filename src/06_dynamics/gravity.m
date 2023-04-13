%% Sopravvivenza_AIRO! 
% Authors: Massimo, Leonardo, Paolo, Francesco
% Computation of potential energy of a genral robot

close all
clear all
clc

my_path = getenv("ROB2LIB_PATH");
addpath(my_path);
FunObj = Rob2Lib();

%% INPUTS 
% PAY ATTENTION: update for each problem!

N = 4; % number of joints 

% Standard symbolic variables
syms q [1 N]
syms L [1 N] 
syms d [1 N]
syms m [1 N]
syms g0

assume(m,{'real', 'positive'})
assume(d,{'real', 'positive'})
assume(L,{'real', 'positive'})

% Direct kinematics.
DHTABLE = [        
    0   L1   0   q1;
    0   L2   0   q2;
    0   L3   0   q3;
    0   L4   0   q4;
];

% Vectors of CoM relative to Reference Frame i
i_CoM_i = {[-L1+d1;0;0;1], [-L2+d2;0;0;1], [-L3+d3;0;0;1], [-L4+d4;0;0;1]};

g = [
    0;
    -g0;
    0;
];

%% END OF INPUTS

% Extraction of DH parametrs
pose = FunObj.compute_dir_kin(DHTABLE);
A = pose{1}; % cell array of chain transformations

% Extraction of CoM vectors
CoM = cell(1,N);
A_i = eye(4);

for i = (1:N)
    A_i = A_i*A{i};
    CoM{i} = A_i * i_CoM_i{i};
end

% % Uncomment this part for debug
% for i = (1:N)
%     CoM{i} = simplify(CoM{i})
% end 

% Potential Energy claculation
U = 0;
for i = (1:N)
    U_i = simplify(- m(i)*(transpose(g))*CoM{i}(1:3));
    U = simplify(U + U_i);
end
U;

g_q = simplify(expand(gradient(U, transpose(q))));

%% EXPERIMENTAL
% Extraction of dynamics coefficents
DYN_COEFF = cell(1,N);

% Building subs array
subs_array = [];
for i = (1:N)
    cos_q = cos(sum(q(1:i)));
    sin_q = sin(sum(q(1:i)));
    subs_array = [subs_array, cos_q, sin_q];
end
subs_values = ones(1,size(subs_array,2));

for i = (1:N)
    DYN_COEFF{i} = collect(g_q(i),[cos(q1), cos(q1+q2), cos(q1+q2+q3), cos(q1+q2+q3+q3)]);
    DYN_COEFF{i} = subs(DYN_COEFF{i}, g0, 1);
    DYN_COEFF{i} = subs(DYN_COEFF{i}, subs_array, subs_values);
    DYN_COEFF{i} = subs(DYN_COEFF{i}, q, ones(1,size(q,2)));
end

dynamic_coeff = [];
for i = (1:N)
    if i == N
        a_i = DYN_COEFF{i};
    else
        a_i = DYN_COEFF{i} - DYN_COEFF{i+1}
    end
    dynamic_coeff = [dynamic_coeff; a_i];
end

dynamic_coeff



