% =========================================================================
% This function is to creat random power
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
% Created: 2019 Dec 13
% Current: 2023 Aug 25
% =========================================================================

function [ power ] = random_power( sim_para )
    epsilon = 1e-5;
    
    r = zeros(1,sim_para.N);
    r(sim_para.N) = rand();
    for i = sim_para.N-1:-1:1
        r(i) = sum(r(i+1:sim_para.N)) + epsilon*rand();
        % r(i) = sum(r(i+1:sim_para.N)) + (sim_para.N-i)*rand();
    end
    
    r = r / sum(r); % ensure that the total sum power <= p_max
    % r = sort(r,'descend');  
    power = sim_para.p_max*r;

end