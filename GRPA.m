% =========================================================================
% This function is to implement the GRPA algorithm
% =========================================================================
% Related Journal Reference: 
% [1] Q.-V. Pham, T. Huynh-The, M. Alazab, J. Zhao, and W.-J. Hwang, 
%     "Sum-Rate Maximization for UAV-assisted Visible Light Communications 
%      using NOMA: Swarm Intelligence meets Machine Learning," IEEE 
%      Internet of Things Journal, vol. 7, no. 10, pp. 10375-10387, Oct. 2020.
%
% [2] 
%
% COPYRIGHT NOTICE:
% All rights belong to Quoc-Viet Pham (email: vietpq90@gmail.com).
% This simulation code can be freely modified and distributed with the 
% original copyright notice. 
% Using this code with your own risk.
%
% Author: QUOC-VIET PHAM
% E-Mail: vietpq90@gmail.com
% Created: 2019 Dec 12
% Current: 2023 Aug 25
% =========================================================================

function [ solution, obj_value, conv_curve,all_solution ] = GRPA( sim_para, coordinate_GUs )

    % run the HHO algorithm to solve the problem
    power = GRPA_power( sim_para );
    [lb, ub, dim, fobj] = obj_function_GRPA( sim_para, coordinate_GUs,power );
    [obj_value,solution,conv_curve,all_solution] = HHO_GRPA(sim_para,lb,ub,dim,fobj,coordinate_GUs,power);
    
    % return the output
    % NOTE: here, the power allocation is NOT ordered properly
    solution = [power solution];
    
    T = size(all_solution,1);
    all_sol = zeros(T,sim_para.N+2);
    for t = 1:T
        all_sol(t,:) = [power all_solution(t,:)];
    end
    all_solution = all_sol;

end