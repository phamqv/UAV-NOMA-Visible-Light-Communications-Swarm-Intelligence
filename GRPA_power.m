% =========================================================================
% This function is to compute the power allocation for the GRPA algorithm
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

function power = GRPA_power( sim_para )
    % N = sim_para.N;
    alpha = sim_para.alpha;
    p_max = sim_para.p_max;
    
    % start to allocate the transmit power
    power = zeros(1,sim_para.N);
    for n = 1:sim_para.N    
        power(n) = p_max/(1 - alpha^sim_para.N)*(1 - alpha)*alpha^(n-1);
    end
end

