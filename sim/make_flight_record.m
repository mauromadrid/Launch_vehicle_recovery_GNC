function rec = make_flight_record(raw, p, opts)
%MAKE_FLIGHT_RECORD Turn a simulated flight into a self-describing record
%   that can be saved and plotted later without re-running anything.
%
%   rec = make_flight_record(raw, p)
%   rec = make_flight_record(raw, p, Name="nominal", Source="Simulink")
%
%   raw - struct with t, X = [x y vx vy m], T (thrust [N]), theta (thrust
%         angle [rad]) and ev (tCutoff, tEntry = [start end] or [],
%         tIgnite). simulate_mission's output has exactly these fields;
%         the MATLAB prototype builds the same struct.
%   p   - the parameter set that was flown
%
%   rec fields:
%     schemaVersion  record format version (bump if the layout changes)
%     meta           name, source, created, git commit of the code
%     params         p
%     signals        table: t, one column per catalog signal (SI, with
%                    VariableUnits/VariableDescriptions), phase (categorical)
%     limits         struct per limited signal: lower/upper (columns, NaN =
%                    no limit at that time) and their labels
%     events         table: name, t
%     display        per-signal display unit/scale/plot style, so a saved
%                    record plots the same even if the catalog changes later
%
%   Signals come from flight_signal_defs: add a signal there to have it
%   computed and stored here.

arguments
    raw struct
    p struct
    opts.Name (1,1) string = "flight"
    opts.Source (1,1) string = "unspecified"
end

t = raw.t(:);
n = numel(t);
ev = raw.ev;

% ---- Helper quantities, computed once ----
aux.rho = zeros(n,1); aux.mach = aux.rho; aux.q = aux.rho; aux.heatIdx = aux.rho;
aux.aDrag = zeros(n,2);
for i = 1:n
    [aD, in] = dragAcceleration(raw.X(i,:)', p);
    aux.aDrag(i,:) = aD';
    aux.rho(i) = in.rho; aux.mach(i) = in.mach; aux.q(i) = in.q; aux.heatIdx(i) = in.heat;
end

% Flight phase of every sample
phase = repmat("coast", n, 1);
phase(t <= ev.tCutoff) = "boostback";
if ~isempty(ev.tEntry)
    phase(t >= ev.tEntry(1) & t <= ev.tEntry(2)) = "entry burn";
end
if isfinite(ev.tIgnite)
    phase(t >= ev.tIgnite) = "landing burn";
end
aux.phase = phase;

% Thrust limits of whichever engine configuration that phase uses
aux.thrustMin = nan(n,1); aux.thrustMax = nan(n,1);   % NaN while coasting
aux.thrustMin(phase == "boostback") = p.T_min_boostback;
aux.thrustMax(phase == "boostback") = p.T_max_boostback;
if isfield(p, 'T_max_entry')
    aux.thrustMin(phase == "entry burn") = p.throttle_min * p.T_max_entry;
    aux.thrustMax(phase == "entry burn") = p.T_max_entry;
end
aux.thrustMin(phase == "landing burn") = p.T_min;
aux.thrustMax(phase == "landing burn") = p.T_max;

% ---- Signals table ----
defs = flight_signal_defs();
sig = table(t, 'VariableNames', {'t'});
units = {'s'}; descs = {'Time from separation'};
rec.limits = struct();
for k = 1:numel(defs)
    d = defs(k);
    sig.(d.name) = d.compute(raw, aux, p);
    units{end+1} = char(d.unit); descs{end+1} = char(d.description); %#ok<AGROW>
    lim = struct('lower', [], 'upper', [], 'lowerLabel', d.lowerLabel, 'upperLabel', d.upperLabel);
    if ~isempty(d.lower), lim.lower = expandLimit(d.lower(raw, aux, p), n); end
    if ~isempty(d.upper), lim.upper = expandLimit(d.upper(raw, aux, p), n); end
    if ~isempty(lim.lower) || ~isempty(lim.upper)
        rec.limits.(d.name) = lim;
    end
end
sig.phase = categorical(phase, ["boostback", "coast", "entry burn", "landing burn"]);
sig.Properties.VariableUnits = [units, {''}];
sig.Properties.VariableDescriptions = [descs, {'Flight phase'}];

% ---- Events ----
names = "Separation"; times = 0;
names(end+1) = "Boostback cutoff"; times(end+1) = ev.tCutoff;
if ~isempty(ev.tEntry)
    names(end+1) = "Entry burn start"; times(end+1) = ev.tEntry(1);
    names(end+1) = "Entry burn end";   times(end+1) = ev.tEntry(2);
end
if isfinite(ev.tIgnite)
    names(end+1) = "Landing-burn ignition"; times(end+1) = ev.tIgnite;
end
if raw.X(end,2) <= 1
    names(end+1) = "Ground contact";
else
    names(end+1) = "End of simulation";
end
times(end+1) = t(end);
rec.events = table(names(:), times(:), 'VariableNames', {'name', 't'});

% ---- Display info and metadata ----
rec.display = struct('name', {defs.name}, 'description', {defs.description}, ...
    'displayUnit', {defs.displayUnit}, 'displayScale', {defs.displayScale}, ...
    'plotStyle', {defs.plotStyle});
rec.schemaVersion = 1;
rec.meta = struct('name', opts.Name, 'source', opts.Source, ...
    'created', datetime('now'), 'gitCommit', gitCommit());
rec.params = p;
rec.signals = sig;
rec = orderfields(rec, {'schemaVersion', 'meta', 'params', 'signals', 'limits', 'events', 'display'});
end

% =====================================================================
function v = expandLimit(v, n)
if isscalar(v), v = repmat(v, n, 1); end
v = v(:);
end

function c = gitCommit()
% Commit of the code that produced the record; "+modified" if the working
% tree had uncommitted changes (the data then isn't exactly reproducible
% from that commit alone).
c = "";
root = fileparts(fileparts(mfilename('fullpath')));
[st, out] = system(sprintf('git -C "%s" rev-parse --short HEAD', root));
if st ~= 0, return, end
c = string(strtrim(out));
[st, out] = system(sprintf('git -C "%s" status --porcelain --untracked-files=no', root));
if st == 0 && ~isempty(strtrim(out)), c = c + "+modified"; end
end
