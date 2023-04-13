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

        % dirKin: a FunObjfunction t compute the direct kinematic of a N joints
        % robot given the table of DH parameters.
        % OUTPUT: a cell array scructured as:
        % dh_par{1}: i-1_A_i for i = 1, ...,n 
        % dh_par{2}: 0_T_N
        % dh_par{3}: p_ee
        function dh_par = compute_dir_kin(DHTABLE)
            syms al a d th
            N = size(DHTABLE,1);

            % Build the general Denavit-Hartenberg trasformation matrix
            TDH = [ 
                    cos(th) -sin(th)*cos(al)  sin(th)*sin(al) a*cos(th);
                    sin(th)  cos(th)*cos(al) -cos(th)*sin(al) a*sin(th);
                      0      sin(al)          cos(al)         d;
                      0      0                0               1
            ];

            % Build transformation matrices for each link
            % First, we create an empty cell array
            A = cell(1,N);
            for i = 1:N
                al = DHTABLE(i,1);
                a = DHTABLE(i,2);
                d = DHTABLE(i,3);
                th = DHTABLE(i,4);
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
     
    end
end

     