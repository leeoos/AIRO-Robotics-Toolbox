%% Sopravvivenza_AIRO! 
%% Author: Massimo, Miryam, Leonardo, Federico, Francesco, Paolo
% This script compute a simple R(r, theta) matrix symbolic in theta

clc
close all

% Define symbols
syms theta

% define values
rx = 0.408248; 
ry = 0.816497; 
rz = -0.408248; 
r = [rx; ry; rz]

% Skew Symmetric of r
Sr = [0, -rz, ry; rz, 0, -rx; -ry, rx, 0] 

% Rotation by Axis Angle direct formula for regular case
R_axis_angle = vpa( round(r*(transpose(r)),4) + (eye(3) - round(r*(transpose(r)),4))*cos(theta) + round(Sr, 4)*sin(theta))

fprintf("Fast to copy symbolic R(r, theta) matrix \n")
fast_copy(R_axis_angle);

%% Fast to copy matrix
function ftc = fast_copy(R)
    fprintf("R_axis_angle = [");
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