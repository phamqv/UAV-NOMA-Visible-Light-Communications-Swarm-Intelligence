% =========================================================================
% This file is to define the FNN for our system: UAV-assisted VLC 
% communications using NOMA 
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
% Created: 2019 Dec 19
% Current: 2023 Aug 25
% =========================================================================
function sumRate = computeRate(sim_para,sol,GU_coordinates)
    p = sol(1:sim_para.N);
    coordinate_UAV = [sol(sim_para.N+1:end), sim_para.H];
    coordinate_GUs = zeros(sim_para.N,3);
    for n = 1:sim_para.N
        x = GU_coordinates(n);
        y = GU_coordinates(n+sim_para.N);
        coordinate_GUs(n,:) = [x,y,0];
    end
    
    % calculate the sum rate
    A = ones(sim_para.N, sim_para.N);
    B = triu(A,1)';
    [ h ] = channel_model( coordinate_GUs, coordinate_UAV, sim_para );
    [h,I] = sort(h,'ascend');
    p = p(I);
    
    % compute SINR and sum rate
    SINR = p.*h./(sim_para.n0 + sum((p'*h).*B,1));
    sumRate = sum(log2(1+SINR));
end