%% Sopravvivenza_AIRO! 
%% Author: Massimo, Miryam, Leonardo, Federico, Francesco
% This script take as input a rotation matrix and compute  
% the related  axes/angles of rotation. 

clc
close all

%% Inverse problem

fprintf("Inverse Axis-Angle calculator\n")
fprintf("REMEMBER to edit the scripts before using them\n\n")

% Define lower bound parameters 
% to avoid numerical error when comparing values to zero
eps = 10^-10;
eps_norm = 10^-4;

% Insert rotation matrix for the inverse problem
fprintf("Rotation Matrix \n")
R = [ 
        0.0795   -0.9874    0.1370;
        0.5000   -0.0795   -0.8624;
        0.8624    0.1370    0.4874;
    ]

fprintf("Press Enter to keep going \n\n")
pause % to check for typos

% Direct formulas to find theta1 and theta2
a1 = (R(1,2) - R(2,1))^2;
a2 = (R(1,3) - R(3,1))^2;
a3 = (R(2,3) - R(3,2))^2;
x = sqrt(a1 + a2 + a3);
y = R(1,1) + R(2,2) + R(3,3) - 1;

% values for tetha
theta1 = atan2(x,y)
theta2 = atan2(-x, y)

if (theta1 <= eps && theta1 >=-eps) % and (theta2 <= eps && theta2 >=-eps)
    fprintf("Singular case with theta: O -> No Solution \n")
    return

elseif isequal(theta1, pi) % and theta2 = -pi
    fprintf("Singular case with theta: +- "+ pi +" -> 2 Solutions \n")

    rx = sqrt((R(1,1) + 1)/2);
    ry = sqrt((R(2,2) + 1)/2);
    rz = sqrt((R(3,3) + 1)/2);

    r = [rx; ry; rz];
    % Print r with sign ambiguities (just to show)
    r_amb = [   strcat(char(177), string(rx)); 
                strcat(char(177), string(ry)); 
                strcat(char(177), string(rz));
            ]
    fprintf("The following computration resolves the sign ambiguity\n")        
    fprintf("\nCHECK THE SIGNS OF THESE COMPONENTS TO BE SURE!\n")
       
    % vector of the sign of [rxry, rxrz, ryrz]
    sign_vector = [R(1,2)/2 >= 0; R(1,3)/2 >= 0; R(2,3)/2 >= 0 ];
    sign_vector_num_val = (2^2)*sign_vector(1) + (2^1)*sign_vector(2) + (2^0)*sign_vector(3);
    
    % 8 case for all possible sign combination
    switch sign_vector_num_val
    
        case 7 % [1 ; 1 ; 1]
            fprintf("\nall componets have the same sign \n")
            r1 = [rx; ry; rz]
            r2 = -r1
            
        case 6 % [1 ; 1 ; 0]
            fprintf("\nno solution to the ambiguity, check the original matrix for errors\n")
        
        case 5 % [1 ; 0 ; 1]
            fprintf("\nno solution to the ambiguity, check the original matrix for errors\n")
            return 
            
        case 4 % [1 ; 0 ; 0]
            fprintf("\nx and y have the same sign while z has opposite sign\n")
            r1 = [rx; ry; -rz]
            r2 = -r1
            
        case 3%[0 ; 1 ; 1]
            fprintf("\nno solution to the ambiguity, check the original matrix for errors\n")
            return 
     
        case 2 % [0 ; 1 ; 0]
            fprintf("\nx and z have the same sign while y has opposite sign\n")
            r1 = [rx; -ry; rz]
            r2 = -r1
            
        case 1 % [0 ; 0 ; 1]
            fprintf("\ny and z have the same sign while x has opposite sign\n")
            r1 = [-rx; ry; rz]
            r2 = -r1 
            
        case 0 % [0 ; 0 ; 0]
            fprintf("\nno solution to the ambiguity, check the original matrix for errors\n")
            return 
            
        otherwise
            fprintf("\nError!!!\n")
            return 
    end

else % sin(theta1) ~= 0 and sin(theta2) ~= 0

    fprintf("Regular case with sin(theta1) = "+ sin(theta1) +" and sin(theta2) = "+ sin(theta2) +" \n")
    fprintf("                  cos(theta1) = "+ cos(theta1) +" and cos(theta2) = "+ cos(theta2) +" \n")
    k = 1/(2*sin(theta1));
    r1x = k*(R(3,2) - R(2,3));
    r1y = k*(R(1,3) - R(3,1));
    r1z = k*(R(2,1) - R(1,2));

    fprintf("This is the axis r1: \n")
    r1 = [r1x; r1y; r1z]
    fprintf ("This is the norm of r1: %f \n", norm(r1))

    k = 1/(2*sin(theta2));
    r2x = k*(R(3,2) - R(2,3));
    r2y = k*(R(1,3) - R(3,1));
    r2z = k*(R(2,1) - R(1,2));
    
    fprintf("This is the axis r2: \n")
    r2 = [r2x; r2y; r2z]
    fprintf ("This is the norm of r2: %f \n", norm(r2))
end

%% Checking the solution buy running the inverse problem
fprintf("Press Enter to check the solution by running direct axis angle method \n\n")
pause 

if verify_inv_axis_angle(r1, theta1, R) && verify_inv_axis_angle(r2, theta2, R)
    fprintf("The two matrix are the same yheee :) \n")
    return
else
    fprintf("Something went wrong and now you are fucked :( \n")
    return
end

 
function ver = verify_inv_axis_angle(r, theta, R)
    % Define lower bound parameters 
    % to avoid numerical error when comparing values to zero
    eps = 10^-10;
    eps_norm = 10^-4;
    rx = r(1);
    ry = r(2);
    rz = r(3);

    % Check if vector norm is 1 (else is not valid)
    if not(abs(norm(r)-1) <= eps_norm)
        fprintf("Error: vector norm not uniary " + norm(r) + "\n")
        ver = false;
        return 
    end
    
    %  Singular case with theta: 0 -> No solution
    if (theta <= eps && theta >= -eps)
        fprintf("Singular case with theta: O -> No Solution \n")
        ver = false;
        return
    
    %  Singular case with theta: +- pi-> One solution
    elseif (theta == +pi  || theta == -pi)
        fprintf("Singular case with theta: +- "+ pi +" -> 2 Solutions \n")
        
        % Original matrix
        R

        % Rotation by Axis-Angle direct formula for singular case
        R_axis_angle = 2*r*(transpose(r)) - eye(3)
    
        % Comparing the computed matrix with the original one
        ver = all(all((R_axis_angle - R <= eps_norm)));
        return
    
    % Regular case with theta: generic 
    else
        fprintf("Regular case with theta: "+ theta +" -> 1 Solution \n")
    
        % Skew Symmetric of r
        Sr = [0, -rz, ry; rz, 0, -rx; -ry, rx, 0];
    
        % Original matrix
        R

        % Rotation by Axis Angle direct formula for regular case
        R_axis_angle = r*(transpose(r)) + (eye(3) - r*(transpose(r)))*cos(theta) + Sr*sin(theta)

        % Comparing the computed matrix with the original one
        ver = all(all((R_axis_angle - R <= eps_norm)));
        return
    end 
end

