function samples = mc_generate_samples(spec, N, opts)
%MC_GENERATE_SAMPLES Draw N sets of dispersed inputs according to spec.
%
%   samples = mc_generate_samples(spec, N)
%   samples = mc_generate_samples(spec, N, Seed=1, Method="lhs")
%
%   spec   - table from mc_select / mc_dispersion_catalog
%   N      - number of samples
%   Seed   - RNG seed (default 1). Stored with the results, so any run can
%            be reproduced exactly; uses a private RandStream so the
%            global RNG state is untouched.
%   Method - "lhs" (default): Latin hypercube -- each variable's range is
%            split into N equal-probability strata and every stratum gets
%            exactly one sample, so even a few hundred samples cover every
%            variable's full range without clumps or gaps.
%            "random": plain independent sampling (textbook Monte Carlo).
%
%   Returns an N x k table, one column per spec row (named after it).
%
%   Sampling is done on the unit hypercube first, then each column is
%   mapped through its distribution's inverse CDF, so adding a new
%   distribution type only means adding one case to toDistribution.

arguments
    spec table
    N (1,1) double {mustBeInteger, mustBePositive}
    opts.Seed (1,1) double = 1
    opts.Method (1,1) string {mustBeMember(opts.Method, ["lhs", "random"])} = "lhs"
end

rs = RandStream('mt19937ar', 'Seed', opts.Seed);
k = height(spec);

U = zeros(N, k);
for j = 1:k
    switch opts.Method
        case "lhs"
            % stratum index (random order) + uniform position within it
            U(:, j) = (randperm(rs, N)' - rand(rs, N, 1)) / N;
        case "random"
            U(:, j) = rand(rs, N, 1);
    end
end

samples = table();
for j = 1:k
    samples.(spec.Name(j)) = toDistribution(U(:, j), spec(j, :));
end
end

function x = toDistribution(u, row)
switch row.Distribution
    case "uniform"
        x = row.A + u * (row.B - row.A);
    case "normal"
        % Truncated at +-3 sigma: squeeze u into [Phi(-3), Phi(3)] before
        % inverting, which samples the truncated normal exactly.
        Phi = @(z) 0.5 * erfc(-z / sqrt(2));
        u = Phi(-3) + u * (Phi(3) - Phi(-3));
        x = row.A + row.B * sqrt(2) * erfinv(2*u - 1);
    otherwise
        error('mc_generate_samples:unknownDistribution', ...
            'Unknown distribution "%s" for %s.', row.Distribution, row.Name);
end
end
