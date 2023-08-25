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
% Ino: the number of features
% Hno: the number of neurons in the hidden layer
% Ono: the number of outputs
% W: connection weights
% B: bias
% x1, x2, x3, x4: input
% =========================================================================

function o = my_FNN(Ino,Hno,Ono,W,B,H,sim_para)
h = zeros(1,Hno);
o = zeros(1,Ono);

for i = 1:Hno
    W_i = zeros(1,length(H));
    for j = 1:length(H)
        W_i(j) = W((j-1)*Hno + i);
    end
    y_h = sum(H.*W_i) + B(i);
    h(i) = My_sigmoid(y_h);
end

k = length(H)-1;
for i = 1:Ono
    k = k+1;
    for j = 1:Hno
        o(i) = o(i) + (h(j)*W(k*Hno+j));
    end
end
% define the output
y_o = zeros(1,Ono);
for i = 1:Ono
    y_o(i) = o(i) + B(Hno+i);
end
for i = 1:Ono-2
    o(i) = exp(y_o(i))/sum(exp(y_o(1:Ono-2)));          % softmax function
    o(i) = o(i)*sim_para.p_max;
end
for i = Ono-1:Ono
    o(i) = abs(y_o(i))/sqrt(sum(y_o(Ono-1:Ono).^2));    % modifed softmax function
    o(i) = o(i)*sim_para.R;
end

