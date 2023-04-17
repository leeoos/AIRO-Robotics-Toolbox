%% Sopravvivenza_AIRO! 
% Author: Massimo, Leonardo, Paolo, Francesco
% This script take as input the direct kinemantics (task kinematics) 
% of a robot manipulator and compute the regressor matrix and 
% the new DH parameters.

clc
close all
clear all

%% INPUTS
% PAY ATTENTION: update for each problem!

N = 2; % number of joints

% Standard symbolic variables
syms q [1 N]
syms zero

% Add the necessary symbols for the problem
syms l1 l2 alpha1 alpha2 d1 d2 

% Write here the task kinematics
dir_kin = [ 
    l1*cos(q1) + l2*cos(q1+q2); 
    l1*sin(q1) + l2*sin(q1+q2);
]

% Arrays of symbolics DH parameters.
% Mind the order {alpha, a, d, theta}. Leave empty  the vectors 
% of parameters you don't want calibrate that DH parameters
alpha = [];
a = [l1, l2];
d = [];
theta = [];
sym_dh = {alpha, a, d, theta};

% Nominal DH parameters
nominal_parameters = [
    alpha,...
    a,...
    d,...
    theta
];

% Insert here nominal values for DH parameters
% PAY ATTENTION to the order: alpha, a, d, theta
nominal_values = [
    1,...
    1
];

% Insert measured DH parameters 
experimental_variables = [q1, q2];
experimental_values = {[0,0], [pi/2,0], [pi/4,-pi/4], [0,pi/4]};

% Insert here experimental variables data
ee_measured_positions = [2; 0; 0; 2; 1.6925; 0.7425; 1.7218; 0.6718];

%% END OF INPUTS

% Computation of symbolic regression matrix
PHI_sym = make_sym_regressor_matrix(dir_kin, sym_dh)
regressor_matrix = subs(PHI_sym, nominal_parameters, nominal_values);
regressor_matrix = subs_experiment(regressor_matrix, experimental_variables, experimental_values, 4);

% Multiple experiments
l = 4; % Number of measurements (experiments)

% Full regressor matrix PHI with experimental values 
PHI_full_val = regressor_matrix;
PHI_full_val = round(vpa(PHI_full_val),3)

% Full vector of end effector errors between nominal and measured positions
dir_kin_sub = subs(dir_kin,nominal_parameters, nominal_values);
dir_kin_sub = subs_experiment(dir_kin_sub, experimental_variables, experimental_values, l);
delta_r_full = ee_measured_positions - dir_kin_sub;
delta_r_full = round(vpa(delta_r_full),3)

%% Calibration algorithm
% Accumulators for updates values of phi
delta_phi_values = cell(1,1);
phi_prime_values = cell(1,1);
phi_prime = transpose(nominal_values);
array_end = 1; % accumulation index to append at the end of the array

% Stop condition when delta phi is small enough 
eps = 10^-6;
delta_phi = eps + 1;

while (abs(max(delta_phi)) > eps)
    
    % Computation of delta phi starting fro experimental data
    pinv_PHI = round(vpa(pinv(PHI_full_val)),3);
    delta_phi = round(vpa(pinv_PHI*delta_r_full),3);
    delta_phi_values{array_end} = delta_phi;
    
    % Update of phi_prime values
    phi_prime = phi_prime + delta_phi;
    phi_prime_values{array_end} = phi_prime;

    % Recomputation of full regressor matrix PHI
    PHI_sym_tmp = subs(PHI_sym, nominal_parameters, phi_prime');
    PHI_full_val = subs_experiment(PHI_sym, experimental_variables, experimental_values, l);
    PHI_full_val = round(vpa(PHI_full_val),3);

    % Recomputation of full vector of end effector errors
    dir_kin_sub = subs(dir_kin,nominal_parameters, phi_prime');
    dir_kin_sub = subs_experiment(dir_kin_sub, experimental_variables, experimental_values, l);
    delta_r_full = (ee_measured_positions - dir_kin_sub);
    delta_r_full = round(vpa(delta_r_full), 3);

    array_end = array_end + 1;
    
end
celldisp(delta_phi_values);
celldisp(phi_prime_values);

%% Functions
% A function to build a symbolic regressor matrix from the direct kinematics
% and a  set of dh symbols
function output_matrix = make_sym_regressor_matrix(input_matrix, symbols)
    accumulator = [];
    for i = (1 : size(symbols,2))
        if not(isempty(symbols{i}))
            partial_jacobian = jacobian(input_matrix, symbols{i});
            accumulator = [accumulator, partial_jacobian];
        end
    end
    output_matrix = accumulator;
end

% A function to efficently substitute a set of experimental value 
% inside a regressor matrix 
function subs_mat = subs_experiment(mat_sym, symb, val, index)
    mat_out = [];
    for i = (1 : index)
        new_row = subs(mat_sym, symb, val{i});
        mat_out = [mat_out; new_row];
    end
    subs_mat = mat_out;
end





