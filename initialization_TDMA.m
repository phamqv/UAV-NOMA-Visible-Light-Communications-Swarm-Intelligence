% =========================================================================
% Initialize solutions for the TDMA scheme
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

function [X] = initialization_TDMA(sim_para,dim,up,down,coordinate_GUs)

% load the simulation settings
X = zeros(sim_para.S,dim);
R = sim_para.R;


Max_S = 10000;
sf_c = 0;
s = 0;
iter = 0;
while s < Max_S
    iter = iter + 1;
    s = s + 1;
    p = sim_para.p_max*ones(1,sim_para.N);

    % placement of the UAV
    theta = rand()*2*pi;
    rR = rand()*R;
    x_u = rR.*cos(theta);
    y_u = rR.*sin(theta);
    X(sf_c+1,:) = [x_u,y_u];

    %___________________ check the solution feasibility __________________%
    coordinate_UAV = [x_u, y_u, sim_para.H];
    [ h ] = channel_model( coordinate_GUs, coordinate_UAV, sim_para );
    SINR = p.*h./(sim_para.n0 + 0);
    Rate = log2(1+SINR)/sim_para.N;
    
    % feasibility 
    % H1 = (sum(p) > sim_para.p_max);
    % H2 = (sum(sqrt(p)) > sim_para.C);
    H3 = (sim_para.r_req > Rate);
    H4 = (x_u^2 + y_u^2 > sim_para.R^2);
        
    % % constraints on SIC operation
    % h_bar = h/sim_para.n0;
    % H5 = zeros(1,sim_para.N-1);
    % for i = 1:sim_para.N-1
    %     H5(i) = (sim_para.theta > p(i)*h_bar(i+1) - sum(p(i+1:sim_para.N))*h_bar(i+1));
    % end
        
    % H = (H1 + H2 + sum(H3) + H4 + sum(H5) > 0);
    H = (sum(H3) + H4 > 0);
    % H = (sum(H3) + H4 + sum(H5) > 0);
    if (H == 0)
        sf_c = sf_c + 1;
        if (sf_c > sim_para.S)
            s = Max_S + 1;
        end
    end
end

end