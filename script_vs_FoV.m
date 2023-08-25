% =========================================================================
% convergence vs different GUs' FoV values
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
% Created: 2019 Dec 17
% Current: 2023 Aug 25
% =========================================================================

% clear all;
% close all;

noRealization = 100;
sim_para = paras_sim;
bigPhi = [40 45 50 55 60 65];

cGUs = zeros(sim_para.N,3,noRealization);
for iR = 1:noRealization
    cGUs(:,:,iR) = get_position( sim_para );
end

% define variables
Rate_HHOPAP = zeros(sim_para.N,sim_para.T,noRealization,length(bigPhi));
sRate_HHOPAP = zeros(length(bigPhi),sim_para.T,noRealization);
Rate_GRPA = zeros(sim_para.N,sim_para.T,noRealization,length(bigPhi));
sRate_GRPA = zeros(length(bigPhi),sim_para.T,noRealization);
Rate_fNOMA = zeros(sim_para.N,sim_para.T,noRealization,length(bigPhi));
sRate_fNOMA = zeros(length(bigPhi),sim_para.T,noRealization);
Rate_RandP = zeros(sim_para.N,sim_para.T,noRealization,length(bigPhi));
sRate_RandP = zeros(length(bigPhi),sim_para.T,noRealization);
Rate_OFDMA = zeros(sim_para.N,sim_para.T,noRealization,length(bigPhi));
sRate_OFDMA = zeros(length(bigPhi),sim_para.T,noRealization);
T = sim_para.T;

% start the simulation
for iPhi = 1:length(bigPhi)
    sim_para.bigPhi = bigPhi(iPhi);
    % bigPhi_matrix = sim_para.bigPhi*pi/180*ones(sim_para.N,1);
    % sim_para.g = sim_para.n^2./(sin(bigPhi_matrix).^2);
    for iReal = 1:noRealization
        coordinate_GUs = cGUs(:,:,iReal);
        
        % __________________ execute the HHOPAP algorithm _________________
        [~, ~, ~, all_solution] = HHOPAP(sim_para,coordinate_GUs);
        % compute the rate
        for t = 1:T
            sol = all_solution(t,:);

            p = sol(1:sim_para.N);
            coordinate_UAV = [sol(sim_para.N+1:end), sim_para.H];

            % calculate the sum rate
            A = ones(sim_para.N, sim_para.N);
            B = triu(A,1)';
            [ h ] = channel_model( coordinate_GUs, coordinate_UAV, sim_para );
            [h,I] = sort(h,'ascend');
            p = p(I);

            % SINR
            SINR = p.*h./(sim_para.n0 + sum((p'*h).*B,1));
            sRate_HHOPAP(iPhi,t,iReal) = sum(log2(1+SINR));
            Rate_HHOPAP(:,t,iReal,iPhi) = log2(1+SINR);
        end
        
        
        
        % ___________________ execute the GRPA algorithm __________________
        [~, ~, ~,all_solution] = GRPA(sim_para,coordinate_GUs);
        for t = 1:T
            sol = all_solution(t,:);

            p = sol(1:sim_para.N);
            coordinate_UAV = [sol(sim_para.N+1:end), sim_para.H];

            % calculate the sum rate
            A = ones(sim_para.N, sim_para.N);
            B = triu(A,1)';
            [ h ] = channel_model( coordinate_GUs, coordinate_UAV, sim_para );
            [h,I] = sort(h,'ascend');
            %%% NOTE: don't need to sort back the power allocation
            % p = p(I);
            SINR = p.*h./(sim_para.n0 + sum((p'*h).*B,1));
            sRate_GRPA(iPhi,t,iReal) = sum(log2(1+SINR));
            Rate_GRPA(:,t,iReal,iPhi) = log2(1+SINR);
        end
        
        
        
        % ========================== Fixed-NOMA ===========================
        % execute the Fixed-NOMA scheme
        [~, ~, ~,all_solution] = FixedNOMA(sim_para,coordinate_GUs);
        % compute the rate
        for t = 1:T
            % assign the solution to power allocation and UAV's placement
            sol = all_solution(t,:); 
            p = sol(1:sim_para.N);
            coordinate_UAV = [sol(sim_para.N+1:end), sim_para.H];

            % calculate the sum rate
            A = ones(sim_para.N, sim_para.N);
            B = triu(A,1)';
            [ h ] = channel_model( coordinate_GUs, coordinate_UAV, sim_para );
            [h,I] = sort(h,'ascend');
            %%% NOTE: don't need to sort back the power allocation
            % p = p(I);
            SINR = p.*h./(sim_para.n0 + sum((p'*h).*B,1));
            sRate_fNOMA(iPhi,t,iReal) = sum(log2(1+SINR));
            Rate_fNOMA(:,t,iReal,iPhi) = log2(1+SINR);
        end
        
        
        % ============================= TDMA ==============================
        % execute the TDMA algorithm
        [~, ~, ~,all_solution] = TDMA(sim_para,coordinate_GUs);
        % compute the rate
        for t = 1:T
            % assign the solution to power allocation and UAV's placement
            sol = all_solution(t,:); 
            coordinate_UAV = [sol, sim_para.H];

            % calculate the sum rate
            [ h ] = channel_model( coordinate_GUs, coordinate_UAV, sim_para );
            p = sim_para.p_max*ones(1,sim_para.N);
            SINR = p.*h./(sim_para.n0 + 0);
            sRate_OFDMA(iPhi,t,iReal) = sum(log2(1+SINR)./sim_para.N);
            Rate_OFDMA(:,t,iReal,iPhi) = log2(1+SINR)./sim_para.N;
        end
        
        
        
        % ============================= RandP =============================
        % execute the GRPA algorithm
        [~, ~, ~,all_solution,coordinate_UAV] = RandP(sim_para,coordinate_GUs);
        % compute the rate
        for t = 1:T
            % assign the solution to power allocation and UAV's placement
            p = all_solution(t,:); 
            % coordinate_UAV = [0 0 sim_para.H];

            % calculate the sum rate
            A = ones(sim_para.N, sim_para.N);
            B = triu(A,1)';
            [ h ] = channel_model( coordinate_GUs, coordinate_UAV, sim_para );
            [h,I] = sort(h,'ascend');
            p = p(I);
            SINR = p.*h./(sim_para.n0 + sum((p'*h).*B,1));
            sRate_RandP(iPhi,t,iReal) = sum(log2(1+SINR));
            Rate_RandP(:,t,iReal,iPhi) = log2(1+SINR);
        end
    end
end

save script_vs_FoV.mat

% ============================ Plot the figure ============================
figure (5)
sR_HHOPAP = mean(sRate_HHOPAP,3);   sR_HHOPAP = sR_HHOPAP(:,end);
sR_GRPA = mean(sRate_GRPA,3);       sR_GRPA = sR_GRPA(:,end);
sR_fNOMA = mean(sRate_fNOMA,3);     sR_fNOMA = sR_fNOMA(:,end);
sR_OFDMA = mean(sRate_OFDMA,3);     sR_OFDMA = sR_OFDMA(:,end);
sR_RandP = mean(sRate_RandP,3);     sR_RandP = sR_RandP(:,end);
hold on
plot(1:length(bigPhi),sim_para.Wi*sR_HHOPAP,'b-s','MarkerSize',14,'linewidth',4.0);
plot(1:length(bigPhi),sim_para.Wi*sR_GRPA,'k-d','MarkerSize',14,'linewidth',4.0);
plot(1:length(bigPhi),sim_para.Wi*sR_fNOMA,'r-^','MarkerSize',14,'linewidth',4.0);
plot(1:length(bigPhi),sim_para.Wi*sR_OFDMA,'b-v','MarkerSize',14,'linewidth',4.0);
plot(1:length(bigPhi),sim_para.Wi*sR_RandP,'k-h','MarkerSize',14,'linewidth',4.0);
hold off;
set(gca,'FontSize',30);
