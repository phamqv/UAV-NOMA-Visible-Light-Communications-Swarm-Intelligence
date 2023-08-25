% =========================================================================
% This function is to implement the proposed HHOPAP algorithm
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
% Created: 2019 Dec 14
% Current: 2023 Aug 25
% =========================================================================

function [ solution, obj_value, conv_curve, all_solution ] = HHOPAP( sim_para, coordinate_GUs )

    % run the HHO algorithm to solve the problem
    [lb, ub, dim, fobj] = obj_function( sim_para, coordinate_GUs );
    [obj_value,solution,conv_curve,all_solution] = HHO(sim_para,lb,ub,dim,fobj,coordinate_GUs);

end