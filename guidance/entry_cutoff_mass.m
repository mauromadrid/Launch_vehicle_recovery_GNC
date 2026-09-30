function mCutoff = entry_cutoff_mass(state, p)
%ENTRY_CUTOFF_MASS Entry-burn engine-cutoff mass for the measured state:
%   entry_burn_sizing reduced to the single number the Simulink model
%   needs. Called extrinsically from the model's MATLAB Function block
%   (a plain double output keeps the block's code-generation typing
%   trivial, instead of mirroring the whole sizing struct).
eb = entry_burn_sizing(state(:), p);
mCutoff = eb.mCutoff;
end
