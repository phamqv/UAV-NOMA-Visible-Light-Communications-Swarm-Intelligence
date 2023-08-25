function [AveCost, nLegal] = ComputeAveCost(Population)

% Compute the average cost of all legal individuals in the population.
% OUTPUTS: AveCost = average cost
%          nLegal = number of legal individuals in population

% Save valid population member fitnesses in temporary array
Cost = [];
nLegal = 0;
for i = 1 : length(Population)
    if Population(i).cost < inf
        Cost = [Cost Population(i).cost];
        nLegal = nLegal + 1;
    end
end
% Compute average cost.
AveCost = mean(Cost);
return;