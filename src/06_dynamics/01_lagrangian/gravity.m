%% Sopravvivenza_AIRO! 
% Authors: Massimo, Leonardo, Paolo, Francesco
% Computation of potential energy of a genral robot

close all
clear all
clc

lib_path = getenv("ROB2LIB_PATH");
addpath(lib_path);
rob2fun = rob2lib();

%% INPUTS
% PAY ATTENTION: update for each problem!

N = 4; % number of joints 

% Suppose diagonal ineria matrix for each link
I_diag = true; 

% Load symbols in the workspace
run('rob2symb.m')

% Definition of gravity vector
% Mind the position of gravity
g = [
    0;
    -g0;
    0;
];

% load robot datasheet
run(lib_path+"/models/planar_4R_model.m")

%% END OF INPUTS

%% INFORMATION EXTRACTION
% Extraction of DH parametrs
pose = rob2fun.compute_dir_kin(DHTABLE);
A = pose{1}; % cell array of chain transformations

% Extraction of CoM vectors
W_CoM = cell(1,N);
A_i = eye(4);

for i = (1:N)
    A_i = A_i*A{i};
    CoM{i} = A_i * R_CoM{i};
end
% Uncomment this part for debug
% for i = (1:N)
%     CoM{i} = simplify(CoM{i})
% end 

% Computation potential energy 
U = 0;
for i = (1:N)
    U_i = simplify(- m(i)*(transpose(g))*CoM{i}(1:3));
    U = simplify(U + U_i);
end
U;

% Computation of g(q)
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
    DYN_COEFF{i} = subs(DYN_COEFF{i}, transpose(q), ones(1,size(q,1)));
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



