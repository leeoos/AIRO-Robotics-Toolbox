%% Sopravvivenza_AIRO! 
% Author: Massimo, Leonardo, Paolo, Francesco
% This script take as input the direct kinemantics (task kinematics) 
% of a robot manipulator and compute the regressor matrix and 
% the new DH parameters

clc
close all

syms da dalpha dd dtheta l1 l2 q1 q2 alpha1 alpha2 d1 d2 zero 

%% Inputs for the calibration problem

% Write here the task kinematics
fprintf("Direct Kinematics \n\n");
dir_kin = [ 
    l1*cos(q1) + l2*cos(q1+q2); 
    l1*sin(q1) + l2*sin(q1+q2);
];
disp(dir_kin);

% Arrays of symbolics DH parameters mantain the order alpha, a, d, theta
% insert "zero" if you don't want calibrate that DH parameters
alpha = [];
a = [l1, l2];
d = [];
theta = [];
sym_dh = {alpha, a, d, theta};

% Insert here nominal values for DH parameters
nominal_parameters = [
    alpha,...
    a,...
    d,...
    theta
];

% Pay attention to the order: alpha, a, d, theta
nominal_values = [
    1,...
    1
];

% Insert measured DH parameters 
experimental_variables = [q1, q2];
experimental_values = {[0,0], [pi/2,0], [pi/4,-pi/4], [0,pi/4]};

% Insert here experimental variables data
ee_measured_positions = [2; 0; 0; 2; 1.6925; 0.7425; 1.7218; 0.6718];

%% Computation of regression matrix
PHI_sym = make_sym_regressor_matrix(dir_kin, sym_dh);
fprintf("This is the regressor matrix: \n");
disp(PHI_sym);

regressor_matrix = subs(PHI_sym, nominal_parameters, nominal_values);
regressor_matrix = subs_experiment(regressor_matrix, experimental_variables, experimental_values, 4);

%% Multiple experiments
l = 4; % Number of measurements

PHI_full_val = regressor_matrix;

dir_kin_sub = subs(dir_kin,nominal_parameters, nominal_values);
dir_kin_sub = subs_experiment(dir_kin_sub, experimental_variables, experimental_values, l);
delta_r_full = ee_measured_positions - dir_kin_sub;

PHI_full_val = round(vpa(PHI_full_val),3)
delta_r_full = round(vpa(delta_r_full),3)

% Iteration steps
delta_phi_values = {};
phi_prime_values = {};
phi_prime = transpose(nominal_values);

eps = 10^-6;
delta_phi = eps + 1;

%% Calibration algorithm
% stop condition when delta phi is small enough 
while (abs(max(delta_phi)) > eps)
    
    pinv_PHI = round(vpa(pinv(PHI_full_val)),3);
    delta_phi = round(vpa(pinv_PHI*delta_r_full),3);
    delta_phi_values{end + 1} = delta_phi;

    phi_prime = phi_prime + delta_phi;
    phi_prime_values{end + 1} = phi_prime;

    PHI_sym_tmp = subs(PHI_sym, nominal_parameters, phi_prime');
    PHI_full_val = subs_experiment(PHI_sym, experimental_variables, experimental_values, l);
    PHI_full_val = round(vpa(PHI_full_val),3);

    dir_kin_sub = subs(dir_kin,nominal_parameters, phi_prime');
    dir_kin_sub = subs_experiment(dir_kin_sub, experimental_variables, experimental_values, l);
    delta_r_full = (ee_measured_positions - dir_kin_sub);
    delta_r_full = round(vpa(delta_r_full), 3);
    
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





