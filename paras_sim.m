% =========================================================================
% Simulation settings
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
% Created: 2019 Dec 11
% Current: 2023 Aug 25
% =========================================================================

function sim_para = paras_sim()

sim_para.R = 10;                % radius of the disc
sim_para.H = 3;                 % unit (m), height of the LED transmitter
sim_para.N = 20;                % number of ground users

sim_para.delta = 3*sqrt(5)/5;   % a coefficient determined by the PAM
sim_para.p_max = 20e-3;         % The maximal transmit power (mW)
sim_para.p_min = 0;             % The maximal transmit power (mW)

sim_para.A = 20;                % 20 sqrt(dBm): DC-offset
sim_para.B = 30;                % 30 sqrt(dBm): peak optical intensity
sim_para.C = (1/sim_para.delta)*min(sim_para.A,sim_para.B-sim_para.A);
sim_para.W = 20*1e6;            % bandwidth = 20 MHz
sim_para.Wi = 20;
sim_para.n0 = 10^(-124/10);     % n0 = -124 dBm
sim_para.r_req = 0.01;          % r_req = 0.01 bits/s/Hz
sim_para.theta = 0.1e-3;

% parameters for the HHO algorithm
sim_para.T = 350;       % Maximum number of iterations
sim_para.S = 30;        % Number of hawks

% alpha for GRPA
sim_para.alpha = 0.4;

% channel modelling
sim_para.n = 1.5;        % the refractive index
sim_para.bigPhi = 50;    % fixed field of view (FOV) of users
% sim_para.g = sim_para.n^2./(sin(sim_para.bigPhi).^2);     % gain of the optical concentrator

% parameter for DL 
sim_para.noSamples = 25000;

end