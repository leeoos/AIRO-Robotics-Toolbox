%% Sopravvivenza_AIRO! Robotics 2 Library
% Authors: Massimo, Leonardo, Paolo, Francesco
% This is a library of MATLAB functions used to solve robotics problems.
%%% USAGE: %%%
% Import "lib" directory and "start_matlab.sh" inside your work directory 
% and run MATLAB via "start_matlab.sh" script. Then add the following header 
% on top of your script:
%%% HEADER %%%
% clc 
% close all
% clear all
% lib_path = getenv("ROB2LIB_PATH");
% addpath(lib_path);
% rob2fun = rob2lib();
%%% END %%%


classdef rob2lib 
    % Definition of library class. Objact of this class can call 
    % library functions as statics methods.

    methods (Static)
    % Definition of statics methos to use as library function inside
    % robotics scripts.

    %% KINEMATICS
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

    %% DYNAMICS

        function M_q = compute_inertia_matrix(KINETIC_ENERGY, N)
            % A function to extract the elements of the inertia matrix M(q) 
            % of a robot's dynamic model

            % Useful symbols for joint velocities
            syms q_dot [1 N]
            q_dot = transpose(q_dot);
            M_foo = [sym("foo")];
            
            % Collection of q_dot terms
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
            c_q_q_dot = [sym("foo"); sym("bar")]; 
            S_q_q_dot = [];
            for i = (1:N)
                dMi_dq = jacobian(M_q(:,i), q);
                dM_dqi = diff(M_q, q(i));
                Ci = 1/2*(dMi_dq + transpose(dMi_dq) - dM_dqi);
                ci = simplify(transpose(q_dot)*Ci*q_dot);
                c_q_q_dot(i) = ci;
                S_q_q_dot = [S_q_q_dot; transpose(q_dot)*Ci];
                %c_q_q_dot = [c_q_q_dot;ci];
            end
            c_q_q_dot = simplify(expand(simplify(c_q_q_dot)));
            
            % Computation Potential energy 
            g_q = 0;
            U = 0;
            if not(isempty(g))
                U = 0;
                for i = (1:N)
                    U_i = simplify(- m(i)*(transpose(g))*W_CoM{i}(1:3));
                    U = simplify(U + U_i);
                end
                U;
                
                % Computation of gravity term g(q)
                g_q = simplify(expand(simplify(gradient(U, q))));
            end

            % Return values
            N_terms = {c_q_q_dot, S_q_q_dot, U, g_q};
        end
        % end of function

        function dyn_matrix = compute_dyn_matrix(model, da, a, N)
            % Experimental function to compute the dynamic matrix                     
            
            % Rewriting the dynamic model with a1,a2, ..., an
            model = simplify(subs(model, da, a));
            for i = (1: N)
                model(i) = collect(simplify(model(i)), a);
            end
            disp("Dynamic Model with a")
            model
            
            % Computation of Dynamic "regressor" matrix
            NUM_DYN = size(da,2);
            Y = [sym("foo")];
            for r = (1:N)
                for c = (1 : NUM_DYN)
                    Y(r,c) = diff(model(r), a(c));
                end
            end
            dyn_matrix = Y;
        end
        % end of function

        function is_skew = skew_symmetry_test(M, S, q, q_dot, N)
            % Skew simmetry test for energy conservation

            N = size(q, 1);

            % New symbols to express q time dependency
            syms x(t) [1 N]
            
            % Time derivative of M
            M_t_dot = rob2lib().time_derivative(M, q, q_dot);
            
            % Conservation of energy theorem 
            M_2S = simplify(M_t_dot - 2*S);
            
            % Test for skewe symmetry
            is_skew = isequal(M_2S, -transpose(M_2S));
            if is_skew 
                disp("M_dot - 2S is skwe symmetric") 
            end
        end


        function derivative = time_derivative(exp, var, var_dot)
            % This function compute teh time derivative of a symbolic
            % expression. It take as imput the expression itself as well as
            % a column vector of the unknown symbols ans a column vector of 
            % the unkowns differenciate by time.

            N = size(var, 1);

            % Symbols for time dependencies
            syms x(t) [1 N]
            x(t) = transpose(x(t));
            
            diff_symbols = [];
            for i = x(t)
                diff_symbols = [diff_symbols, diff(i,t)];
            end
            diff_symbols;
            
            % Explicitation of time dependency in expression
            exp_t = subs(exp,var, x);
            
            % Time derivative
            exp_t_dot = diff(exp_t, t);

            % Substitution of diff with nominal variables
            exp_t_dot = subs(exp_t_dot, diff_symbols, var_dot);

            % Substitution of x with nominal variables
            exp_t_dot = subs(exp_t_dot, x, var);

            derivative = exp_t_dot;
        end
        % end of function
             
    end
end


%% EXPERIMENTAL
% aliases = sym("alias",[1 N]);
% q_dot_squared = [sym("foo")];
% for i = (1:N)
%     q_dot_squared(i) = str2sym("q_dot"+string(i)+"^2");
% end
% q_dot_squared;
% KIN_alias = subs(KINETIC_ENERGY,q_dot_squared,aliases);
% 
% reduced_q_dot = [sym("foo")];
% M = [sym("bar")];
% for r = (1:N)
%     for c = (1:N)
%         if (r == c)
%             reduced_alias = aliases(aliases~=aliases(r));
%             KIN_q_dot_squared = subs(KIN_alias, transpose(q_dot), zeros(1,N));
%             KIN_qr_dot_squared = subs(KIN_q_dot_squared, reduced_alias, zeros(1,N-1));
%             M(r,c) = simplify(2*subs(KIN_qr_dot_squared, aliases(r), 1));
%         else
%             reduced_q_dot = transpose(q_dot);
%             reduced_q_dot(reduced_q_dot == q_dot(c)) = [];
%             reduced_q_dot(reduced_q_dot == q_dot(r)) = [];
%             KIN_q_dot = subs(KIN_alias, aliases, zeros(1,N));
%             if not(isempty(reduced_q_dot))
%                 KIN_q_dot = subs(KIN_q_dot, reduced_q_dot, zeros(1,N-2));
%             end
%             M(r,c) = subs(KIN_q_dot, {q_dot(r),q_dot(c)}, {1,1});
%         end
%     end
% end
% disp(["Are M_experimental and M_q equal?", isequal(M, M_q)])
%% END OF EXPERIMENTAL

     