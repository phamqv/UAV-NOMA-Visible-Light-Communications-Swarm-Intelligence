% =========================================================================
% the sum rate performance vs the cell radius R
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
tic

noRealization = 100;
sim_para = paras_sim;
H_set = [1.0 1.5 2.0 2.5 3.0 3.5 4.0 4.5 5.0 5.5 6.0 6.5 7.0 7.5 8.0];

cGUs = zeros(sim_para.N,3,noRealization);
for iH = 1:noRealization
    cGUs(:,:,iH) = get_position( sim_para );
end

% define variables
Rate_HHOPAP = zeros(sim_para.N,sim_para.T,noRealization,length(H_set));
sRate_HHOPAP = zeros(length(H_set),sim_para.T,noRealization);
Rate_GRPA = zeros(sim_para.N,sim_para.T,noRealization,length(H_set));
sRate_GRPA = zeros(length(H_set),sim_para.T,noRealization);
Rate_fNOMA = zeros(sim_para.N,sim_para.T,noRealization,length(H_set));
sRate_fNOMA = zeros(length(H_set),sim_para.T,noRealization);
Rate_RandP = zeros(sim_para.N,sim_para.T,noRealization,length(H_set));
sRate_RandP = zeros(length(H_set),sim_para.T,noRealization);
Rate_OFDMA = zeros(sim_para.N,sim_para.T,noRealization,length(H_set));
sRate_OFDMA = zeros(length(H_set),sim_para.T,noRealization);
T = sim_para.T;

% start the simulation
for iH = 1:length(H_set)
    sim_para.H = H_set(iH);
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
            sRate_HHOPAP(iH,t,iReal) = sum(log2(1+SINR));
            Rate_HHOPAP(:,t,iReal,iH) = log2(1+SINR);
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
            % p = sort_back( p, I );
            % SINR = p.*h./(sim_para.n0 + h*(repmat(p,sim_para.N,1).*B));
            SINR = p.*h./(sim_para.n0 + sum((p'*h).*B,1));
            sRate_GRPA(iH,t,iReal) = sum(log2(1+SINR));
            Rate_GRPA(:,t,iReal,iH) = log2(1+SINR);
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
            sRate_OFDMA(iH,t,iReal) = sum(log2(1+SINR)./sim_para.N);
            Rate_OFDMA(:,t,iReal,iH) = log2(1+SINR)./sim_para.N;
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
            sRate_RandP(iH,t,iReal) = sum(log2(1+SINR));
            Rate_RandP(:,t,iReal,iH) = log2(1+SINR);
        end
    end
end

save script_vs_altitude.mat

% ============================ plot the figure ============================
figure (7)
sR_HHOPAP = mean(sRate_HHOPAP,3);   sR_HHOPAP = sR_HHOPAP(:,end);
sR_GRPA = mean(sRate_GRPA,3);       sR_GRPA = sR_GRPA(:,end);
sR_OFDMA = mean(sRate_OFDMA,3);     sR_OFDMA = sR_OFDMA(:,end);
sR_RandP = mean(sRate_RandP,3);     sR_RandP = sR_RandP(:,end);
hold on
plot(1:length(H_set),sim_para.Wi*sR_HHOPAP,'b-s','MarkerSize',14,'linewidth',4.0);
plot(1:length(H_set),sim_para.Wi*sR_GRPA,'k-d','MarkerSize',14,'linewidth',4.0);
plot(1:length(H_set),sim_para.Wi*sR_OFDMA,'b-v','MarkerSize',14,'linewidth',4.0);
plot(1:length(H_set),sim_para.Wi*sR_RandP,'k-h','MarkerSize',14,'linewidth',4.0);
hold off;
set(gca,'FontSize',30);
xticks = 1:1:length(H_set);
% xticklabels({'2','3','4','5','6','7','8','9','10'});
xticklabels({'1','1.5','2','2.5','3','3.5','4','4.5','5','5.5','6','6.5','7','7.5','8'});
set(gca,'xtick',xticks); 
xlabel('UAV altitude h (meters)','FontSize',30); 
ylabel('Sum rate (Mbits/s)','FontSize',30);
lgd = legend('HHOPAP','GRPA','OFDMA','RandP');
lgd.FontSize = 30;
box on;

toc
