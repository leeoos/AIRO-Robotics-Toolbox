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
classdef rob2lib 

    methods (Static)

        function dh_par = compute_dir_kin(DHTABLE)
            % A function to compute the direct kinematic 
            % of a N joints robot given the table of DH parameters.
            % OUTPUT: a cell array scructured as:
            % dh_par{1}: i-1_A_i for i = 1, ...,n 
            % dh_par{2}: 0_T_N
            % dh_par{3}: p_ee

            % DH symbols
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

            % Return values
            p = T(1:4,4);
            dh_par = {A, T, p};
        end
        % end of function

        function M_q = compute_inertia_matrix(KINETIC_ENERGY, N)
            % A function to extract the elements of the inertia matrix M(q) 
            % of a robot's dynamic model

            % Useful symbols for joint velocities
            syms q_dot [1 N]
            q_dot = transpose(q_dot);
            M_foo = [sym("foo")];
            
            for i = (1:N)
                KINETIC_ENERGY = collect(KINETIC_ENERGY, q_dot(i)^2);
            end
            
            for r = (1:N)
                for c = (1:N)
                    if ((r == c))
                        M_foo(r,c) = simplify(diff(KINETIC_ENERGY, q_dot(r), 2));
                    else
                        K_reduced_qr = simplify(diff(KINETIC_ENERGY, q_dot(c)));
                        K_reduced_qrc = simplify(diff(K_reduced_qr, q_dot(r)));
                        M_foo(r,c) = simplify(K_reduced_qrc);
                    end
                end
            end

            % Return value
            M_q = M_foo;
        end
        % end of funtion


        function N_terms = compute_n_terms(M_q, W_CoM, g, N)
            % A funtion to compute the coriolis, centrifugal c(q, q_dot) and 
            % gravity terms g(q) of a robot's dynamic model

            % Useful symbols for joint variables and velocities
            syms q [1 N]
            syms q_dot [1 N]
            q = transpose(q);
            q_dot = transpose(q_dot);

            % masses
            syms m [1 N]
             
            % Computation of coriolis, centrifugal and gravity terms
            c_q_q_dot = []; 
            for i = (1:N)
                dMi_dq = jacobian(M_q(:,i), q);
                dM_dqi = diff(M_q, q(i));
                Ci = 1/2*(dMi_dq + transpose(dMi_dq) - dM_dqi);
                ci = simplify(transpose(q_dot)*Ci*q_dot);
                c_q_q_dot = [c_q_q_dot;ci];
            end
            c_q_q_dot = simplify(expand(simplify(c_q_q_dot)));
            
            % Computation Potential energy 
            U = 0;
            for i = (1:N)
                U_i = simplify(- m(i)*(transpose(g))*W_CoM{i}(1:3));
                U = simplify(U + U_i);
            end
            U;
            
            % Computation of gravity term g(q)
            g_q = simplify(expand(simplify(gradient(U, q))));

            % Return values
            N_terms = {c_q_q_dot, g_q};
        end
        % end of function
             
    end
end

     