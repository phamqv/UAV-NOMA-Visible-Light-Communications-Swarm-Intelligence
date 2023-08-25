% =========================================================================
% This function is to define the objective function for the proposed 
% GRPA algorithm
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

function [lb, ub, dim, fobj] = obj_function_GRPA( sim_para, coordinate_GUs,power )
    % return the outputs
    lb = -sim_para.R*ones(1,2);
    ub = sim_para.R*ones(1,2);
    % 2 UAV's coordinates
    dim = 2; 
    fobj = @get_objective_function;
    
    % create a matrix for NOMA rate computation
    A = ones(sim_para.N, sim_para.N);
    B = triu(A,1)';
    
    % penalty method to deal with inequality constraints
    % X.-S. Yang Nature-Inspired Optimization Algorithms (2014, Elsevier)
    muy = 1e14;     % muy can be taken as 10^13 to 10^15 
    p = power;
    
    function objf = get_objective_function(PaP)
        % transmit power followed the GRPA scheme
        % p = GRPA_power( sim_para );
        coordinate_UAV = [PaP, sim_para.H];
        
        % calculate the channel gain
        [ h ] = channel_model( coordinate_GUs, coordinate_UAV, sim_para );
        % sort the channel gains in the ascending order
        [h,I] = sort(h,'ascend');
        %%% NOTE: don't need to sort back the power allocation
        % p = p(I);   
        % p = sort_back( p, I );
        % SINR = p.*h./(sim_para.n0 + h*(repmat(p,sim_para.N,1).*B));
        SINR = p.*h./(sim_para.n0 + sum((p'*h).*B,1));
        Rate = log2(1+SINR);
        
        % QoS requirements of ground users (GUs)
        H3 = (sim_para.r_req > log2(1+SINR));
        % penalty = penalty + muy*sum(H3.*((sim_para.r_req - log2(1+SINR)).^2));
        penalty = muy*sum(H3.*((sim_para.r_req - log2(1+SINR)).^2));
        
        % deployment of the UAV
        x_u = coordinate_UAV(1);
        y_u = coordinate_UAV(2);
        H4 = (x_u^2 + y_u^2 > sim_para.R^2);
        penalty = penalty + muy*sum(H4.*((x_u^2 + y_u^2 - sim_para.R^2).^2));
        
        % constraints on SIC operation
        h_bar = h/sim_para.n0;
        H5 = zeros(1,sim_para.N-1);
        for i = 1:sim_para.N-1
            H5(i) = (sim_para.theta > p(i)*h_bar(i+1) - sum(p(i+1:sim_para.N))*h_bar(i+1));
            penalty = penalty + muy*H5(i).*((sim_para.theta - p(i)*h_bar(i+1) + sum(p(i+1:sim_para.N))*h_bar(i+1)).^2);
        end
        
        % penalty = 0;
        % objective function (maximization --> minimization problem)
        objf = - sum(Rate) + penalty;
    end
    
end