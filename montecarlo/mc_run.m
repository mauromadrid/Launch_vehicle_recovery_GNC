function results = mc_run(pNominal, samples, caseFcn, opts)
%MC_RUN Run one case per sample and collect inputs + outputs in a table.
%
%   results = mc_run(pNominal, samples, caseFcn)
%   results = mc_run(..., Verbose=true)
%
%   pNominal - nominal parameter struct
%   samples  - table from mc_generate_samples (one row per case)
%   caseFcn  - @(p) -> struct of scalar metrics (numeric, logical or
%              string). This is the only phase-specific part: Phase 2
%              uses mc_case_boostback; later phases plug in a case that
%              runs the landing guidance or the full Simulink model,
%              without touching sampling, execution or bookkeeping.
%
%   results  - one row per case: the sampled inputs, then every metric
%              caseFcn returned, then
%                status   "ok" or "error" (caseFcn threw -- recorded and
%                         skipped, so one bad case can't kill a long run)
%                message  error message for "error" cases
%                runTime  wall-clock time of the case [s]
%              results.Properties.UserData holds the nominal case's
%              metrics and the run metadata.
%
%   Runs serially: Parallel Computing Toolbox isn't installed here. The
%   cases are independent, so switching the loop to parfor is the only
%   change needed where it is.

arguments
    pNominal struct
    samples table
    caseFcn (1,1) function_handle
    opts.Verbose (1,1) logical = true
end

N = height(samples);

% The nominal case doubles as a template for the metric fields, so failed
% cases can be filled with NaN / "" / false of the right type.
nominal = caseFcn(pNominal);
template = structfun(@emptyLike, nominal, 'UniformOutput', false);

out = repmat(template, N, 1);
status  = strings(N, 1);
message = strings(N, 1);
runTime = zeros(N, 1);

tStart = tic;
for i = 1:N
    tCase = tic;
    try
        p = mc_apply_sample(pNominal, samples(i, :));
        out(i) = orderfields(caseFcn(p), template);
        status(i) = "ok";
    catch err
        status(i) = "error";
        message(i) = string(err.message);
    end
    runTime(i) = toc(tCase);

    if opts.Verbose && (mod(i, max(1, round(N/10))) == 0 || i == N)
        fprintf('  case %4d/%d  (%.0f s elapsed, %d errors)\n', ...
                i, N, toc(tStart), nnz(status == "error"));
    end
end

results = [samples, struct2table(out, 'AsArray', true), ...
           table(status, message, runTime)];
results.Properties.UserData = struct( ...
    'nominal', nominal, ...
    'caseFcn', func2str(caseFcn), ...
    'created', datetime('now'), ...
    'totalTime', toc(tStart));
end

function v = emptyLike(x)
if isstring(x) || ischar(x)
    v = "";
elseif islogical(x)
    v = false;
else
    v = NaN;
end
end
