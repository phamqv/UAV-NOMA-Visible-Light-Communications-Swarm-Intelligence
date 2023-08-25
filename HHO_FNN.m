% =========================================================================
% The HHO trainer
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

function [MinCost,Best] = HHO_FNN(ProblemFunction, DisplayFlag, sim_para)

% Harris Hawk's Optimization (HHO) for optimizing a general function.

% INPUTS: ProblemFunction is the handle of the function that returns 
%         the handles of the initialization, cost, and feasibility functions.
%         DisplayFlag says whether or not to display information during iterations and plot results.

if ~exist('DisplayFlag', 'var')
    DisplayFlag = true;
end

[OPTIONS, MinCost, AvgCost, InitFunction, CostFunction, FeasibleFunction, ...
    MaxParValue, MinParValue, Population] = Init(DisplayFlag, ProblemFunction);

% HHO initialization
dim = OPTIONS.numVar;
lb = MinParValue;
ub = MaxParValue;
Rabbit = Population(1); % global best

% Begin the optimization loop
for GenIndex = 1 : OPTIONS.Maxgen
    % Update the global best if needed
    if Population(1).cost < Rabbit.cost
        Rabbit = Population(1);
    end
    
    % factor to show the decreaing energy of rabbit
    E1 = 2*(1-(GenIndex/OPTIONS.Maxgen)); 
    % Update the location of Harris' hawks
    for i = 1:OPTIONS.popsize
        E0 = 2*rand()-1; %-1<E0<1
        Escaping_Energy = E1*(E0);  % escaping energy of rabbit
        
        if abs(Escaping_Energy)>=1
            % Exploration:
            % Harris' hawks perch randomly based on 2 strategy:
            
            % An equal chance q for each perching strategy
            q = rand();
            rand_Hawk_index = floor(OPTIONS.popsize*rand()+1);
            % X_rand = X(rand_Hawk_index, :);
            X_rand = Population(rand_Hawk_index).chrom;
            if q<0.5
                % perch based on other family members according to Eq. (1)
                % X(i,:) = X_rand - rand()*abs(X_rand-2*rand()*X(i,:));
                Population(i).chrom = X_rand - rand()*abs(X_rand - 2*rand()*Population(i).chrom);
            elseif q>=0.5
                % perch on a random tall tree (random site inside group's home range)
                % X(i,:) = (Rabbit_Location(1,:) - mean(X)) - rand()*((ub-lb)*rand+lb);
                meanX = zeros(1,dim);
                for k = 1:OPTIONS.popsize
                    meanX = meanX + Population(k).chrom;
                end
                meanX = meanX/OPTIONS.popsize;
                Population(i).chrom = (Rabbit.chrom - meanX) - rand()*((ub-lb)*rand+lb);
            end
            
        elseif abs(Escaping_Energy)<1
            % Exploitation:
            % Attacking the rabbit using 4 strategies regarding the behavior of the rabbit
            
            % phase 1: surprise pounce (seven kills)
            % surprise pounce (seven kills): multiple, short rapid dives by different hawks
            
            r = rand(); % probablity of each event
            
            if r>=0.5 && abs(Escaping_Energy)<0.5 % Hard besiege
                Population(i).chrom = Rabbit.chrom - Escaping_Energy*abs(Rabbit.chrom - Population(i).chrom);
            end
            
            if r>=0.5 && abs(Escaping_Energy)>=0.5  % Soft besiege
                Jump_strength = 2*(1-rand()); % random jump strength of the rabbit
                Population(i).chrom = (Rabbit.chrom - Population(i).chrom) - Escaping_Energy*abs(Jump_strength*Rabbit.chrom - Population(i).chrom);
            end
            
            % phase 2: performing team rapid dives (leapfrog movements)
            % Soft besiege 
            % The rabbit try to escape by many zigzag deceptive motions
            if r<0.5 && abs(Escaping_Energy)>=0.5 
                
                Jump_strength = 2*(1-rand());
                X1 = Rabbit.chrom - Escaping_Energy*abs(Jump_strength*Rabbit.chrom - Population(i).chrom);
                
                if cost_VLC(X1,sim_para) < cost_VLC(Population(i).chrom,sim_para)       % improved move?
                    Population(i).chrom = X1;
                else % hawks perform levy-based short rapid dives around the rabbit
                    X2 = Rabbit.chrom - Escaping_Energy*abs(Jump_strength*Rabbit.chrom - Population(i).chrom)+rand(1,dim).*Levy(dim);
                    if (cost_VLC(X2,sim_para) < cost_VLC(Population(i).chrom,sim_para)) % improved move?
                        Population(i).chrom = X2;
                    end
                end
            end
            
            % Hard besiege 
            % The rabbit try to escape by many zigzag deceptive motions
            if r<0.5 && abs(Escaping_Energy)<0.5 
                % hawks try to decrease their average location with the rabbit
                Jump_strength = 2*(1-rand());
                meanX = zeros(1,dim);
                for k = 1:OPTIONS.popsize
                    meanX = meanX + Population(k).chrom;
                end
                meanX = meanX/OPTIONS.popsize;
                X1 = Rabbit.chrom - Escaping_Energy*abs(Jump_strength*Rabbit.chrom - meanX);
                
                if cost_VLC(X1,sim_para) < cost_VLC(Population(i).chrom,sim_para)     % improved move?
                    Population(i).chrom = X1;
                else % Perform levy-based short rapid dives around the rabbit
                    X2 = Rabbit.chrom - Escaping_Energy*abs(Jump_strength*Rabbit.chrom - meanX) + rand(1,dim).*Levy(dim);
                    if (cost_VLC(X2,sim_para) < cost_VLC(Population(i).chrom,sim_para)) % improved move?
                        Population(i).chrom = X2;
                    end
                end
            end
            %
        end
    end

    % Make sure the population does not have duplicates. 
    Population = ClearDups(Population, MaxParValue, MinParValue);
    % Make sure each individual is legal.
    Population = FeasibleFunction(OPTIONS, Population);
    % Calculate cost
    Population = CostFunction(OPTIONS, Population);
    % Sort from best to worst
    Population = PopSort(Population);
    % Compute the average cost of the valid individuals
    [AverageCost, nLegal] = ComputeAveCost(Population);
    % Display info to screen
    MinCost = [MinCost Population(1).cost];
    AvgCost = [AvgCost AverageCost];
    if DisplayFlag
        disp(['The best and mean of Generation # ', num2str(GenIndex), ' are ',...
            num2str(MinCost(end)), ' and ', num2str(AvgCost(end))]);
    end
end
Best = Conclude(DisplayFlag, OPTIONS, Population, nLegal, MinCost);
return;

end % end of the trainer

% _____________________________Levy function_______________________________
function o = Levy(d)
    beta = 1.5;
    sigma = (gamma(1+beta)*sin(pi*beta/2)/(gamma((1+beta)/2)*beta*2^((beta-1)/2)))^(1/beta);
    u = randn(1,d)*sigma;v=randn(1,d);step=u./abs(v).^(1/beta);
    o = step;
end

% ___________________________Fitness function______________________________
function fitness = cost_VLC(X,sim_para)
    % Calculating classification rates
    % sim_para = paras_sim;
    % load tData_samples.mat
    load tData.mat
    % noSamples = size(tData,1);
    N = sim_para.N;
    H = tData(1:size(tData,1),1:N*2);                   % input
    sol = tData(1:size(tData,1),N*2+1:size(tData,2));   % output

    no_trData = floor(0.7*size(tData,1));
    no_ttData = size(tData,1) - no_trData;

    % there are 4 trainers including the HHO trainer
    dim = N*2*(N+2) + (N+2) + (N+2)*(N+2) + (N+2);
    noWeights = N*2*(N+2) + (N+2)*(N+2);
    % noBiases = (N+2) + (N+2);
    fitness = 0;
    W = X(1:noWeights);
    B = X(noWeights+1:dim);
    
    % Ino,Hno,Ono: features, (hidden) neurons, and outputs
    Ino = N*2; Hno = N+2; Ono = N+2;
    for pp = 1:no_ttData
        sol_FNN = my_FNN(Ino,Hno,Ono,W,B,H(pp,:),sim_para);
        
        sumRate = computeRate(sim_para,sol(pp,:),H(pp,:));
        sumRate_FNN = computeRate(sim_para,sol_FNN,H(pp,:));
        
        % compute the fitness value
        fitness = fitness + (sumRate - sumRate_FNN)^2;
        % final_sRate(pp,i) = sumRate;
        % final_sRate_FNN(pp,i) = sumRate_FNN;
    end
    fitness = fitness/no_ttData;
end