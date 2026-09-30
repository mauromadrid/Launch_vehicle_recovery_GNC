function bb = boostback_targeting(state0, p, opts)
%BOOSTBACK_TARGETING Boostback guidance that accounts for drag and the
%   entry burn, by aim-point biasing around the (vacuum) convex solve.
%
%   bb = boostback_targeting(state0, p)
%   bb = boostback_targeting(state0, p, MaxIter=6, Tol=10)
%
%   The convex boostback (boostback_guidance) plans in vacuum and aims its
%   coast arc at the landing-burn ignition point. Drag and the entry burn
%   then move where the stage actually arrives. Predictor-corrector:
%
%     bias = 0
%     repeat
%        solve the SOCP aiming at  x_target + bias      (still convex:
%                                                         only a constant moves)
%        predict the descent from the planned cutoff    (predict_descent:
%          with the guidance model -- drag + the entry    exponential atmosphere,
%          burn the flight computer would size)           never the plant)
%        miss = x_predicted - x_target
%        bias <- secant step on miss(bias)              (first step: bias - miss)
%     until |miss| < Tol
%
%   Converges in a few iterations because miss(bias) is close to linear
%   with slope ~1. What remains in flight is the guidance-model error
%   (exponential vs COESA atmosphere), not the targeting.
%
%   state0 - [x; y; vx; vy; m] at burn start; [] = separation state
%   bb     - boostback_guidance result for the final bias, plus:
%              aimBias     - applied downrange bias [m]
%              predictedX  - predicted downrange at ignition [m]
%              predictedMiss, iterations, converged
%              predictedEntry - entry-burn sizing the prediction assumed
%   With neither drag nor an entry burn, it's boostback_guidance unchanged.

arguments
    state0
    p struct
    opts.MaxIter (1,1) double = 6
    opts.Tol (1,1) double = 10          % [m]
end

if isempty(state0)
    state0 = [p.sep.x; p.sep.y; p.sep.vx; p.sep.vy; p.m0];
end

needsBias = p.aero.dragEnabled || (isfield(p, 'entry') && p.entry.enabled);
if ~needsBias
    bb = boostback_guidance(state0, p);
    bb.aimBias = 0; bb.predictedX = p.target.x; bb.predictedMiss = 0;
    bb.iterations = 0; bb.converged = true; bb.predictedEntry = [];
    return
end

xTarget = p.target.x;
biases = []; misses = [];
bias = 0;
for it = 1:opts.MaxIter
    pAim = p;
    pAim.target.x = xTarget + bias;
    bb = boostback_guidance(state0, pAim);
    pred = predict_descent(bb.cutoffState, p);
    miss = pred.final(1) - xTarget;

    biases(end+1) = bias; misses(end+1) = miss; %#ok<AGROW>
    if abs(miss) < opts.Tol
        break
    end
    if isscalar(biases)
        bias = bias - miss;                       % slope ~1 first guess
    else
        slope = (misses(end) - misses(end-1)) / (biases(end) - biases(end-1));
        bias = bias - miss / slope;               % secant
    end
end

bb.rTarget(1)     = xTarget;                      % report the real target
bb.aimBias        = biases(end);
bb.predictedX     = pred.final(1);
bb.predictedMiss  = misses(end);
bb.iterations     = numel(biases);
bb.converged      = abs(misses(end)) < opts.Tol;
bb.predictedEntry = pred.entry;
end
