function [Population] = ClearDups(Population, MaxParValue, MinParValue)

% Make sure there are no duplicate individuals in the population.
% This logic does not make 100% sure that no duplicates exist, but any duplicates that are found are
% randomly mutated, so there should be a good chance that there are no duplicates after this procedure.
for i = 1 : length(Population)
    Chrom1 = sort(Population(i).chrom);
    for j = i+1 : length(Population)
        Chrom2 = sort(Population(j).chrom);
        if isequal(Chrom1, Chrom2)
            parnum = ceil(length(Population(j).chrom) * rand);
            Population(j).chrom(parnum) = floor(MinParValue + (MaxParValue - MinParValue + 1) * rand);
        end
    end
end
return;