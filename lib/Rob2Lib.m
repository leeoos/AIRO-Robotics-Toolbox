%% Sopravvivenza_AIRO! Robotics 2 Library
% Authors: Massimo, Leonardo, Paolo, Francesco
% This is a library of MATLAB functions used to solve robotics problems.
%%% USAGE: %%%
% Import "lib" and "config" directories inside your work directory and 
% run the appropriate script for your system inside ".config".
% Then add the following header on top of your script:
%%% HEADER %%%
% path = getenv("ROB2LIB_PATH")
% addpath(path);
% FunObj = Rob2Lib();
%%% END %%%

% Definition of a class whose ojbects (FunObj) will be used to statically 
% call the function of this library inside other scripts
classdef Rob2Lib 
    methods (Static)

        % compute_dir_kin: a function to compute the direct kinematic 
        % of a N joints robot given the table of DH parameters.
        % OUTPUT: a cell array scructured as:
        % dh_par{1}: i-1_A_i for i = 1, ...,n 
        % dh_par{2}: 0_T_N
        % dh_par{3}: p_ee
        function dh_par = compute_dir_kin(DHTABLE)
            syms DH_ALPHA DH_A DH_D DH_THETA
            N = size(DHTABLE,1);

            % Build the general Denavit-Hartenberg trasformation matrix
            TDH = [ 
                    cos(DH_THETA),  -sin(DH_THETA)*cos(DH_ALPHA),   sin(DH_THETA)*sin(DH_ALPHA),    DH_A*cos(DH_THETA);
                    sin(DH_THETA),  cos(DH_THETA)*cos(DH_ALPHA),    -cos(DH_THETA)*sin(DH_ALPHA),   DH_A*sin(DH_THETA);
                    0,              sin(DH_ALPHA),                  cos(DH_ALPHA),                  DH_D;
                    0,              0,                              0,                              1
            ];

            % Build transformation matrices for each link
            % First, we create an empty cell array
            A = cell(1,N);
            for i = 1:N
                DH_ALPHA = DHTABLE(i,1);
                DH_A = DHTABLE(i,2);
                DH_D = DHTABLE(i,3);
                DH_THETA = DHTABLE(i,4);
                A{i} = subs(TDH);
            end
                
            % Accumulation matrix
            T = eye(4);
            for i=1:N
                T = T*A{i};
                T = simplify(T);               
            end

            % return values
            p = T(1:4,4);
            dh_par = {A, T, p};
        end
        % end 

        % Function to compute the derivative of a rotation matrix
        function R_dot = diff_rotation_matrix(R, w)
            syms wx wy wz
            w_sym = [wx,wy,wz];
            
            Sw = [  
                    0, -wz, wy; 
                    wz, 0, -wx; 
                    -wy, wx,  0
            ];
            
            Sw = subs(Sw, w_sym, w);
            R_dot = Sw * R;
        end
     
    end
end

     