%% Sopravvivenza_AIRO! 
%% Authors: Massimo, Miryam, Leonardo, Francesco, Paolo
% This script take as input DH table and compute
% the related homogeneous transformation matrix.

%% Copyrights to: A. De Luca) 5 Nov 2012 DH frames 
% assigned as in lecture slides 09_DirectKinematics.

clc
close all

%% Symbolical evaluation 

fprintf("Direct Denavit Hartenberg\n")
fprintf("Remember to edit the scripts before using them\n\n")

% Define symbolic variables
syms alpha a d theta 

% Insert DH table of parameters.
fprintf("DH table of parameters \n")
%"Notice that it is not unique!!! \n"

DHTABLE_SYM = [  sym('alpha1') sym('a1') sym('d1') sym('t1');
                 sym('alpha2') sym('a2') sym('d2') sym('t2');
                 sym('alpha3') sym('a3') sym('d3') sym('t3');
                 sym('alpha4') sym('a4') sym('d4') sym('t4');
                 %sym('alpha5') sym('a5') sym('d5') sym('t5');
                 %sym('alpha6') sym('a6') sym('d6') sym('t6');
              ]

% N is the number of joints of the robots
N = size(DHTABLE_SYM,1);

% DH table values
fprintf("Remember to check the numbers in the script\n")
fprintf("Press Enter to keep going \n\n")
pause

alpha1 = pi/2;
alpha2 = pi/2;
alpha3 = pi/2;
alpha4 = 0;
alpha5 = 0;
alpha6 = 0;

a1 = 0; 
a2 = 0;
a3 = 0;
a4 = sym("a4");
a5 = 0;
a6 = 0;

d1 = sym('d1');
d2 = 0;
d3 = sym('q3');
d4 = 0;
d5 = -115.7;
d6 = 92.2;

t1 = sym('q1');
t2 = sym('q2');
t3 = pi;
t4 = sym('q4');
t5 = 0;
t6 = 0;

fprintf("\nDH table of parameters \n")
fprintf("Notice that it is not unique!!! \n")
DHTABLE = subs(DHTABLE_SYM)

fprintf("Press Enter to keep going \n\n")
pause % to check for typos
     
% Build the general Denavit-Hartenberg trasformation matrix
TDH = [ cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
        sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
          0             sin(alpha)             cos(alpha)            d;
          0               0                      0                   1];

% Build transformation matrices for each link
% First, we create an empty cell array
A = cell(1,N);

% Here we save the 0-p-i vectrors and 0-R-i rotations
p_i = cell(1,N);
Ri = cell(1,N);

for i = 1:N
    alpha = DHTABLE(i,1);
    a = DHTABLE(i,2);
    d = DHTABLE(i,3);
    theta = DHTABLE(i,4);
    A{i} = subs(TDH);
    fprintf("%d_A_%d", i-1, i)
    A{i}
end

disp('Direct kinematics of robot in symbolic form (simplifications may need some time)')

disp(['Number of joints N=', num2str(N)])

T = eye(4);

for i=1:N 
    T = T*A{i};
    fprintf("0_T_%d", i)
    T = simplify(T)

    % Extracting 0-p-i 
    p_i{i} = T(1:3, 4);
    fprintf("0_p_%d \n", i)
    p_i{i};

    % Extracting 0-R-i
    Ri{i} = T(1:3, 1:3);
    fprintf("0_R_%d \n", i)
    Ri{i};
end

T0N = T

pe = T(1:3,4)

%% GEOMETRIC JACOBIAN ALGORITHM

% COMPUTATION OF Z_I-1 AND P_I-1_E
% In this section we compute the elements for the Geometric Jacobian Matrix

z_jac = cell(1,N);
p_jac = cell (1,N);
z0 = [0, 0, 1]';

for i=2:N
    z_jac{1} = z0;
    z_jac{i} = Ri{i-1} * z0;
    fprintf("z_%d \n", i-1)
    z_jac{i}

    p_jac{i} = pe - p_i{i};
    fprintf("%d_p_E \n", i-1)
    p_jac{i}
end 

%% COMPUTATION OF LINEAR AND ANGULAR CONTRIBUTION

jac_L = cell(1,N);
jac_A = cell (1,N);

% Change this sequence based on your robot!
fprintf("This is your robot joint sequence \n")
seq_joint = ['r', 'r', 'p', 'r']

% Remember to use t
q = [t1, t2, d3, t4];

fprintf("Press Enter to keep going \n")
pause

for i=1:N
    
    % prismatic
    if seq_joint(i) == 'p'
        jac_A{i} = [0, 0, 0]';
        jac_L{i} = z_jac{i};
        fprintf ("JL_%d", i)
        jac_L{i}
        fprintf("JA_%d", i)
        jac_A{i}

    % revolut 
    elseif seq_joint(i) == 'r'
        jac_A{i} = z_jac{i};
        q_curr = q(i);
        jac_L{i} = diff(pe, q_curr);
        fprintf ("JL_%d", i)
        jac_L{i}
        fprintf("JA_%d", i)
        jac_A{i}
    end 

end

%% GEOMETRIC JACOBIAN 

geom_jacobian = sym('J%d%d', [6 N]);


for i=1:N
    geom_jacobian(1:3,i) = jac_L{i};
    geom_jacobian(4:6,i) = jac_A{i};
end 

fprintf("This is the Geometric Jacobian: \n")
geom_jacobian

%% END