% =========================================================================
% This function is to create channel modeling in VLC systems
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

function [ h ] = channel_model( coordinate_GUs, coordinate_UAV, sim_para )
% This file is to create the channel model

%__________________________ Simulation settings __________________________#
% R = 10;         % radis of the disc
% N = size(coordinate_GUs, 1);       % number of users
N = sim_para.N;
phi_1_2 = 60*pi/180;            % unit is radian
m = -log(2)/log(cos(phi_1_2));  % order of Lambertian emission
T_s = 1;        % the gain of the optical filter
% n = 1.5;        % the refractive index
A = (10^-2)^2*ones(N,1);        % photodiode (PD) detection area is 1 cm2
% Big_phi = 50*pi/180*ones(N,1);  % fixed field of view (FOV) of users
% g = n^2./(sin(Big_phi).^2);     % gain of the optical concentrator

%__________________ Distance betwen GUs and the UAV ______________________#
h_UAV = coordinate_UAV(3);
d = sqrt(sum((coordinate_GUs - coordinate_UAV).^ 2,2));
% Values of these two angles depend on the locations of GUs
varphi = acosd(h_UAV./d);   % irradiance angle w.r.t. the TX perpendicular axis
phi = acosd(h_UAV./d);      % incidence angle w.r.t. the TX axis
% varphi = 45*pi/180*ones(1,N);               % fixed angle of incidence w.r.t. the TX axis
% phi = (50*ones(1,N)-20*rand(1,N))*pi/180;   % random angle of irradiance w.r.t. the transmitter perpendicular axis
% Convert angle from degrees to radians
varphi = deg2rad(varphi);
phi = deg2rad(phi);
R_0 = (m+1)*cos(varphi).^2/(2*pi);          % Lambertian radiant intensity of the LED transmitter

%_______________ channel modeling according to Eq. (1) ___________________#
% generalized Lambertian emission model
% h is a row vector and has the size of [1xN]
bigPhi_matrix = sim_para.bigPhi*pi/180*ones(sim_para.N,1);
g = sim_para.n^2./(sin(bigPhi_matrix).^2);
h = (A./d.^2).*R_0.*T_s.*g.*cos(phi);  
h = h';

% % Channel gain
% PathLoss_BS_User = 60 + 40*rand(1,M);
% PathLoss_BS_User = sort(PathLoss_BS_User,'descend');
% h = 10.^(-PathLoss_BS_User/10);