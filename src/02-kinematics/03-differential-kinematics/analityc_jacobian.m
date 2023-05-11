%% Sopravvivenza_AIRO! 
%% Author: Massimo, Miryam, Leonardo, Federico, Francesco
%% This script take as input the direct kinemantics (task kinematics) 
%% of a robot manipulator and compute the analytic Jacobian 

clc
clear all
close all

fprintf("Analytic Jacobian\n")
fprintf("Remember to edit the scripts before using them\n\n")

% Define symbolic variables
% Change the symbolic variables with respect to your robot 
syms q1 q2 q3 q4 L1 L2  M k a b

q = [
        q1;
        q2;
        q3;
        q4;
    ]

% Definition of task kinematics
fprintf ("This is the Direct Kinematics fr(q): \n")
Px = q2*cos(q1) + q4*cos(q1 + q3)
Py = q2*sin(q1) + q4*sin(q1 + q3)
                                                


r = [   
        Px;
        Py;
        q1 + q3
    ]

fprintf("Press Enter to keep going \n\n")
pause % to check for typos
fprintf ("This is the Jacobian Matrix Jr(q): \n")

J = jacobian(r,q)

fprintf("Press Enter to keep going \n\n")
pause % to check for typos

% Computation of the Analytic Jacobian and singularities:
fprintf ("\nAnalytic Jacobian Analysis: \n")

if (size(J, 1) == size(J, 2))
    % Compute the determinant of Jacobian Matrix
    fprintf("Determinant of Jr(q): \n")
    det_J = simplify(det(J))
end

% Evaluate Jr in a singular configuration
% Remember to modify the values according to the 
% problem: insert the one that make  the determinant 0
J_sing_1 = subs(J,{q3},{0}) 
J_sing_2 = subs(J,{q3},{-pi}) 

J_sing = J_sing_1

fprintf("Rank of Jr(q): \n")
rank_J = rank(J_sing)

fprintf("Null Space of Jr(q): \n")
null_space_J = null(J_sing)

% Takes the number of colums of N(Jr(q))
fprintf("Dimention of Null Space of Jr(q): \n")
dim_null_space_J = size(null_space_J, 2)  

%fprintf("Normalizing... \n")
% Now we normalize. It is important to remember that the dimension of Null
% Space in this exercise could be at max 2, because:
% dim N = n - rank(J) = 3 - 1 = 2 (in case in which J is low rank)!
% For this reason we take both 2 columns (if the Null Space has dimens 2)
% and divide each colum by its norm

if dim_null_space_J > 1
  null_space_J(:,1) = null_space_J(:,1)/norm(null_space_J(:,1));
  null_space_J(:,2) = null_space_J(:,2)/norm(null_space_J(:,2));
  null_space_J 

else
  null_space_J = null_space_J/norm(null_space_J )

end

fprintf("Press Enter to keep going \n\n")
pause % to check for typos

% RangeJ=orth(J_sing)
% RangeJ=simplify(orth(J))
% pause

% Number of columns of matrix RangeJ
% dimRangeSpaceJ=size(RangeJ,2)
% pause

%% INVERSE DIFFERENTIAL KINEMATICS WITH PSEUDOINVERSE 

% syms alfa 
% 
% r_dot = [-1;0;0];
% 
% J_sharp = pinv(J_sing);
% 
% simplify(J_sharp)
% 
% q_dot = J_sharp * r_dot;
% 
% simplify(q_dot)
