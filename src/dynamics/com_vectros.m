%% Sopravvivenza_AIRO! 
% Authors: Massimo, Leonardo, Paolo, Francesco
% Computation of CoM vectors w.r.t. world frame

my_path = getenv("ROB2LIB_PATH");
addpath(my_path);
FunObj = Rob2Lib();

close all
clc

%% INPUTS FOR THE PROBLEM
% PAY ATTENTION: update for each problem!

syms L1 L2 L3 d1 d2 d3 q1 q2 q3 

% Direct kinematics.
DHTABLE = [        
    0    L1     0   q1;
    0    L2     0   q2;
];
N = size(DHTABLE, 1); % number of joints
%%

pose = FunObj.compute_dir_kin(DHTABLE);
A = pose{1}; % cell array of chain transformations

% Extraction of CoM vectors
CoM = cell(1,N);
A_i = eye(4);

for i = (1:N)
    r_i = A{i}(1:4, 4);
    for j = (1:i-1)
        A_i = A_i*A{j};
    end
    CoM{i} = A_i * r_i;
end

CoM{i}


