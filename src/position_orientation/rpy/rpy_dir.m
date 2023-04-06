%% Sopravvivenza_AIRO! 
%% Author: Massimo, Miryam, Leonardo, Federico, Francesco, Paolo
% This script take as input a sequence of rotation and compute the 
% symbolic rotation matrix of Roll Pitch Yaw (left side multiplications)

clc
close all

fprintf("Direct RPY script \n")
fprintf("REMEMBER to edit the scripts before using them\n\n")

%% Direct Problem

% IMPORTANT: change in the list of symbols and in the array of angles
% the names of the angles with the name given by the problem 
syms phi theta psi
syms alpha beta gamma
syms alpha1 alpha2 alpha3 

%angles =   [   
%            phi
%            theta
%            psi
%           ]

%angles =   [   
%            alpha
%            beta 
%            gamma
%           ]

angles =   [   
            alpha1
            alpha2
            alpha3
           ]

% Accumulator matrix for the product, inizialized as I3 
Res = eye(3);

num_of_rotation = input('How many rotation: ');

fprintf("Define the sqequence of rotation axes \n")

axis_names = "";
% Note: this code is not 'safe' double check the input to be sure of the result
for i=(1:num_of_rotation)
    % This loop genarate the proper R matrix for each angle in angles
    % according to the given axis of rotation taken as input

    axis = input('Insert an axis of rotation (x, y, z): ', 's');
    stop = false;

    while (not(stop))
        if isequal(axis, 'x')
            R = [   1       0                   0;
                    0 cos(angles(i))            -sin(angles(i));
                    0 sin(angles(i))            cos(angles(i));
                ];
            stop = true;
            axis_names = strcat(axis_names, axis);

        elseif isequal(axis, 'y')
            R = [   cos(angles(i))      0       sin(angles(i));
                    0                   1       0;
                    -sin(angles(i))     0       cos(angles(i));
                ];
            stop = true;
            axis_names = strcat(axis_names, axis);

        elseif isequal(axis, 'z')
            R = [   cos(angles(i))      -sin(angles(i))     0;
                    sin(angles(i))      cos(angles(i))      0;
                        0              0                   1;
                ];
            stop = true;
            axis_names = strcat(axis_names, axis);

        else
            fprintf("Invalid Input \n")
            axis = input('Insert an axis of rotation (x, y, z): ', 's');
            stop = false;
        end
    end

    % Perform the matrix multiplication in a fixed order (left)
    % Res will contain the composition of rotation at each step
    Res = R * Res;
end

% Final result (composition of all rotation)
fprintf("Final Matrix: right rotation composition \n")
R_rpy = Res

fprintf("Fast to copy symbolic RPY matrix \n")
fast_copy(R_rpy);
evalc('writeInOutput(R_rpy, axis_names)');

% Uncomment the right section of this code and assig the value 
% to obtain a numerical evaluation of Euler matrix 
% phi =
% theta =
% psi =
% 
% alpha =
% beta =
% gamma =
% 
% alpha1 = 
% alpha2 = 
% alpha3 = 
% 
% fprintf("Evaluated matrix \n")
% R_rpy_eval = subs(R_rpy)
%
% fprintf("Fast to copy eval RPY matrix \n")
% fast_copy(R_rpy_eval)
% evalc('writeInOutput(R_rpy_eval, axis_names)');


%% Fast to copy matrix
function ftc = fast_copy(R)
    fprintf("R_rpy = [");
    for row=(1:3)
        for col=(1:3)
            if (col == 3) 
                fprintf("%s;", string(R(row,col)));
            else
                fprintf("%s,", string(R(row,col)));
            end
        end
    end 
    fprintf("]\n\n");

    ftc = true;
end

%% Output File: Write matrix on output.txt
function w = writeInOutput(R, axis_names)
    % Open output file
    
    fileID = fopen('output_rpy.txt', 'a');

    fprintf("This is the content of output_euler.txt")
    fprintf(fileID, "\nRotation sequence: %s\n", axis_names);
    
    fprintf(fileID, "R_rpy = [");
    for row=(1:3)
        for col=(1:3)
            if (col == 3) 
                fprintf(fileID, "%s;", string(R(row,col)));
            else
                fprintf(fileID, "%s,", string(R(row,col)));
            end
        end
    end 

    fprintf(fileID, "]");
    type 'output_rpy.txt'
    fprintf(fileID, "\n");
    fclose(fileID);
    w = true;
end