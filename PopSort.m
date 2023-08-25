function [Population, indices] = PopSort(Population)

% Sort the population members from best to worst
popsize = length(Population);
Cost = zeros(1, popsize);
indices = zeros(1, popsize);
% Cost after sorting
for i = 1 : popsize
    Cost(i) = Population(i).cost;
end
[Cost, indices] = sort(Cost, 2, 'ascend');
Chroms = zeros(popsize, length(Population(1).chrom));
% Connection weights after sorting
for i = 1 : popsize
    Chroms(i, :) = Population(indices(i)).chrom;
end
% Return the sorted population
for i = 1 : popsize
    Population(i).chrom = Chroms(i, :);
    Population(i).cost = Cost(i);
end