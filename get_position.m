% =========================================================================
% This function is to create random positions of the ground users
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

function [ coordinate_GUs ] = get_position( sim_para )
    % initialized position
    N = sim_para.N;     % number of users
    R = sim_para.R;     % radius of the disc

    % create random positions for #N ground users
    coordinate_GUs = zeros(N,3);
    for n = 1:N
        theta = rand*2*pi;
        r = rand();
        x = r*R*cos(theta);
        y = r*R*sin(theta);
        coordinate_GUs(n,:) = [x,y,0];
    end
end

