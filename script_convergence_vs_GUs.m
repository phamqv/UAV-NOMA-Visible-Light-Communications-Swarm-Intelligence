% =========================================================================
% convergence vs the number of GUs
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

noRealization = 100;
sim_para = paras_sim;

noGUs = 10:2:20;
conv_curve = zeros(length(noGUs),sim_para.T,noRealization);
for iNo = 1:length(noGUs)
    sim_para.N = noGUs(iNo);
    cGUs = zeros(sim_para.N,3,noRealization);
    for iR = 1:noRealization
        cGUs(:,:,iR) = get_position( sim_para );
    end
    for iReal = 1:noRealization
        coordinate_GUs = cGUs(:,:,iReal);
        % execute the HHOPAP algorithm
        [solution, obj_value, curve, all_solution] = HHOPAP(sim_para,coordinate_GUs);
        conv_curve(iNo,:,iReal) = curve;
    end
end
conv_curve = mean(conv_curve,3);

figure (2)
hold on
plot(1:length(conv_curve),-sim_para.Wi*conv_curve(1,:),'b-s','MarkerIndices',[1:25:length(conv_curve) length(conv_curve)],'MarkerSize',14,'linewidth', 4.0);
plot(1:length(conv_curve),-sim_para.Wi*conv_curve(2,:),'k-d','MarkerIndices',[1:25:length(conv_curve) length(conv_curve)],'MarkerSize',14,'linewidth', 4.0);
plot(1:length(conv_curve),-sim_para.Wi*conv_curve(3,:),'r-^','MarkerIndices',[1:25:length(conv_curve) length(conv_curve)],'MarkerSize',14,'linewidth', 4.0);
plot(1:length(conv_curve),-sim_para.Wi*conv_curve(4,:),'b-v','MarkerIndices',[1:25:length(conv_curve) length(conv_curve)],'MarkerSize',14,'linewidth', 4.0);
plot(1:length(conv_curve),-sim_para.Wi*conv_curve(5,:),'k-h','MarkerIndices',[1:25:length(conv_curve) length(conv_curve)],'MarkerSize',14,'linewidth', 4.0);
plot(1:length(conv_curve),-sim_para.Wi*conv_curve(6,:),'r-p','MarkerIndices',[1:25:length(conv_curve) length(conv_curve)],'MarkerSize',14,'linewidth', 4.0);
hold off;
% set(gca,'FontSize',30,'XLim',[1 length(SR_1)]);
set(gca,'FontSize',30);
xticks = 0:50:length(conv_curve);
set(gca,'xtick',xticks); 
xlabel('Number of iterations'); 
ylabel('Sum Rate (Mbits/s)');
columnlegend(3, {'10 GUs','12 GUs','14 GUs','16 GUs','18 GUs','20 GUs'}, 'location', 'northwest')
box on;