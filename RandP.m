% =========================================================================
% This function is to implement the RandP algorithm
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

function [ solution, obj_value, conv_curve,all_solution, coordinate_UAV ] = RandP( sim_para, coordinate_GUs )
    
    % placement of the UAV
    theta = rand()*2*pi;
    rR = rand()*sim_para.R;
    x_u = rR.*cos(theta);
    y_u = rR.*sin(theta);
    coordinate_UAV = [x_u,y_u sim_para.H];
    
    % run the HHO algorithm to solve the problem
    [lb, ub, dim, fobj] = obj_function_RandP( sim_para, coordinate_GUs, coordinate_UAV );
    [obj_value,solution,conv_curve,all_solution] = HHO_RandP(sim_para,lb,ub,dim,fobj,coordinate_GUs, coordinate_UAV);

end