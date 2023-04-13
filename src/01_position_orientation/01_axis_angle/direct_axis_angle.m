%% Sopravvivenza_AIRO! 
%% Author: Massimo, Miryam, Leonardo, Federico, Francesco, Paolo
% This script take as input an axis/angle of rotation and 
% compute the related rotation matrix.

clc
close all

%% Direct problem

fprintf("Direct Axis-Angle \n")
fprintf("REMEMBER to edit the scripts before using them\n\n")

% Define lower bound parameters 
% to avoid numerical error when comparing values to zero
eps = 10^-10;
eps_norm = 10^-4;

% Insert values for rotation vector and angle
fprintf("Values for rotation vector and angle\n");
rx = 0.408248; 
ry = 0.816497; 
rz = -0.408248; 
theta = pi; 
r = [rx; ry; rz]

fprintf("Press Enter to keep going \n\n")
pause % to check for typos

% Check if vector norm is 1 (else is not valid)
if not(abs(norm(r)-1) <= eps_norm)
    fprintf("Error: vector norm not uniary\n")
    return 
end

%  Singular case with theta: 0 -> No solution
if (theta <= eps && theta >= -eps)
    fprintf("Singular case with theta: O -> No Solution \n")
    return

%  Singular case with theta: +- pi-> One solution
elseif (theta == +pi  || theta == -pi)
    fprintf("Singular case with theta: +- "+ pi +" -> 2 Solutions \n")

    % Rotation by Axis-Angle direct formula for singular case
    R_axis_angle = 2*r*(transpose(r)) - eye(3)
    
    if validate(R_axis_angle, r)
        evalc('writeInOutput(r, theta, R_axis_angle)');
    end

% Regular case with theta: generic 
else
    fprintf("Regular case with theta: "+ theta +" -> 1 Solution \n")

    % Skew Symmetric of r
    Sr = [0, -rz, ry; rz, 0, -rx; -ry, rx, 0] 

    % Rotation by Axis Angle direct formula for regular case
    R_axis_angle = r*(transpose(r)) + (eye(3) - r*(transpose(r)))*cos(theta) + Sr*sin(theta)
    
    if validate(R_axis_angle, r)
        evalc('writeInOutput(r, theta, R_axis_angle)');
    end
end 

% Check if R is valid for the axis angle rotation.
% If r is invariant then R is valid
function inv = validate(R, r)
    eps_check = 10^4; 
    invariant = R*r
    if (invariant - r <= eps_check)
        fprintf("Invariant property OK :) \n")
        inv = true;
    end
    %inv = false;
end


%% Output File: Write matrix on output.txt
function w = writeInOutput(r, theta, R)
    % Open output file
    
    fileID = fopen('output_axis_angle.txt', 'a');
    fprintf("This is the content of output_axis_angle.txt")
    
    fprintf(fileID, "\nRotation of angle: %f\n", theta);
    fprintf(fileID, "On vector: ");
    for index=(1:3) 
        fprintf(fileID, "%f    ", r(index));
    end
    fprintf(fileID, "\nR_axis_angle = [");
    for row=(1:3)
        for col=(1:3)
            if (col == 3) 
                fprintf(fileID, "%f;", R(row,col));
            else
                fprintf(fileID, "%f,", R(row,col));
            end
        end
    end  
    fprintf(fileID, "]");

    type 'output_axis_angle.txt'
    fprintf(fileID, "\n");
    fclose(fileID);
    w = true;
end
