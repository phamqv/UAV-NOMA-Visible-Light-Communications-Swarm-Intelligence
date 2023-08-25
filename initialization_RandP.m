% =========================================================================
% Initialize solutions for the RandP algorithm
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

function [X] = initialization_RandP(sim_para,dim,up,down,coordinate_GUs,coordinate_UAV)

% load the simulation settings
X = zeros(sim_para.S,dim);
% p_max = sim_para.p_max;
% alpha = sim_para.alpha;
% N = sim_para.N;
% R = sim_para.R;


Max_S = 10000;
sf_c = 1;
s = 0;
iter = 0;
while s < Max_S
    iter = iter + 1;
    s = s + 1;

    %___________________ check the solution feasibility __________________%
    % coordinate_UAV = [0, 0, sim_para.H];
    A = ones(sim_para.N, sim_para.N);
    B = triu(A,1)';
    [ h ] = channel_model( coordinate_GUs, coordinate_UAV, sim_para );
    % sort the channel gains in the ascending order
    [h,I] = sort(h,'ascend'); 
    p = zeros(1,sim_para.N);
    for n = sim_para.N:-1:1
        p(n) = (2^sim_para.r_req - 1)*(sum(h(n)*p(n+1:sim_para.N)) + sim_para.n0)/h(n);
        p(n) = max(p(n),(sum(h(n)*p(n+1:sim_para.N)) + sim_para.theta*sim_para.n0)/h(n));
    end
    p = p*(rand*rand*sim_para.p_max/sum(p));
    X(sf_c,1:sim_para.N) = sort_back( p, I );
    
    % power and channel gains should follow the same order
    SINR = p.*h./(sim_para.n0 + sum((p'*h).*B,1));
    % feasibility 
    H1 = (sum(p) > sim_para.p_max);
    H2 = (sum(sqrt(p)) > sim_para.C);
    H3 = (sim_para.r_req > log2(1+SINR));
    % H4 = (x_u^2 + y_u^2 > sim_para.R^2);
        
    % constraints on SIC operation
    h_bar = h/sim_para.n0;
    H5 = zeros(1,sim_para.N-1);
    for i = 1:sim_para.N-1
        H5(i) = (sim_para.theta > p(i)*h_bar(i+1) - sum(p(i+1:sim_para.N))*h_bar(i+1));
    end
        
    % H = (H1 + H2 + sum(H3) + H4 + sum(H5) > 0);
    H = (H1 + H2 + sum(H3) + sum(H5) > 0);
    if (H == 0)
        sf_c = sf_c + 1;
        if (sf_c > sim_para.S)
            s = Max_S + 1;
        end
    end
end

end