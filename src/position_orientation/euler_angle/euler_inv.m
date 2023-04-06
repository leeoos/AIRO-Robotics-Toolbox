%% Sopravvivenza_AIRO! 
%% Author: Massimo, Miryam, Leonardo, Federico, Francesco, Paolo
% This script take as input a euler rotation matrix  
% and compute the related sequence of Euler angles 

clc
close all

fprintf("Inverse Euler problem \n")
fprintf("REMEMBER to edit the scripts before using them\n\n")

%% Iverse problem
fprintf("Rotation matrix for the inverse problem")
R = [   0, 1, 0;
        1/2, 0, sqrt(3)/2;
        sqrt(3)/2, 0, -1/2;
    ]

fprintf("Press Enter to procede with the inverse problem \n")
pause

fprintf("Remember to change the values inside the script!!!")
% Insert your values

% Computation of a2

sin_a2 = -R(2,3);
cos_a2 = sqrt(R(1,3)^2 + R(3,3)^2);
a2_1 = atan2(sin_a2, cos_a2);
a2_2 = atan2(sin_a2, -cos_a2);

% Computation of a1
sin_a1_1 = R(1,3) / (cos(a2_1));
cos_a1_1 = R(3,3) / (cos(a2_1));
a1_1 = atan2(sin_a1_1, cos_a1_1);

sin_a1_2 = R(1,3) / (cos(a2_2));
cos_a1_2 = R(3,3) / (cos(a2_2));
a1_2 = atan2(sin_a1_2, cos_a1_2);

% Computation of a3
sin_a3_1 = R(1,2) / (cos(a2_1));
cos_a3_1 = R(1,1) / (cos(a2_1));
a3_1 = atan2(sin_a3_1, cos_a3_1);

sin_a3_2 = R(1,2) / (cos(a2_2));
cos_a3_2 = R(1,1) / (cos(a2_2));
a3_2 = atan2(sin_a3_2, cos_a3_2);

fprintf("The two sequence of angles are: \n")
sequence1 = [a1_1, a2_1, a3_1]
sequence2 = [a1_2, a2_2, a3_2]


      
%axis_order = "xyz"
% switch axis_order
% 
%     case "xyz"
%         fprintf("Singularity for a2: pi/2 \n")
% 
%         % Computation of a2
%         sin_a2 = R(1,3);
%         cos_a2 = sqrt(R(1,1)^2 + R(1,2)^2);
%         a2_1 = atan2(sin_a2, cos_a2);
%         a2_2 = atan2(sin_a2, -cos_a2);
% 
%         % Computation of a1
%         sin_a1_1 = R(2,3) / (-cos_a2);
%         cos_a1_1 = R(3,3) / (cos_a2);
%         a1_1 = atan2(sin_a1_1, cos_a1_1);
% 
%         sin_a1_2 = R(2,3) / (cos_a2);
%         cos_a1_2 = R(3,3) / (-cos_a2);
%         a1_2 = atan2(sin_a1_2, cos_a1_2);
% 
%         % Computation of a3
%         sin_a3_1 = R(1,2) / (-cos_a2);
%         cos_a3_1 = R(1,1) / (cos_a2);
%         a3_1 = atan2(sin_a3_1, cos_a3_1);
% 
%         sin_a3_2 = R(1,2) / (cos_a2);
%         cos_a3_2 = R(1,1) / (-cos_a2);
%         a3_2 = atan2(sin_a3_2, cos_a3_2);
% 
%         fprintf("The two sequence of angles are: \n")
%         sequence1 = [a1_1, a2_1, a3_1]
%         sequence2 = [a1_2, a2_2, a3_2]
% 
%     otherwise
% 
%         fprintf("Remember to change the values inside the script!!!")
%         % Insert your values
% 
%         % Computation of a2
% 
%         sin_a2 = -R(2,3);
%         cos_a2 = sqrt(R(1,3)^2 + R(3,3)^2);
%         a2_1 = atan2(sin_a2, cos_a2);
%         a2_2 = atan2(sin_a2, -cos_a2);
% 
%         % Computation of a1
%         sin_a1_1 = R(1,3) / (cos(a2_1));
%         cos_a1_1 = R(3,3) / (cos(a2_1));
%         a1_1 = atan2(sin_a1_1, cos_a1_1);
% 
%         sin_a1_2 = R(1,3) / (cos(a2_2));
%         cos_a1_2 = R(3,3) / (cos(a2_2));
%         a1_2 = atan2(sin_a1_2, cos_a1_2);
% 
%         % Computation of a3
%         sin_a3_1 = R(1,2) / (cos(a2_1));
%         cos_a3_1 = R(1,1) / (cos(a2_1));
%         a3_1 = atan2(sin_a3_1, cos_a3_1);
% 
%         sin_a3_2 = R(1,2) / (cos(a2_1));
%         cos_a3_2 = R(1,1) / (cos(a2_1));
%         a3_2 = atan2(sin_a3_2, cos_a3_2);
% 
%         fprintf("The two sequence of angles are: \n")
%         sequence1 = [a1_1, a2_1, a3_1]
%         sequence2 = [a1_2, a2_2, a3_2]
% 
% end

