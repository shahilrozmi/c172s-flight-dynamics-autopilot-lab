function [sys, model] = c172sLateralDirectionalPlant(data)
%C172SLATERALDIRECTIONALPLANT Construct the C172S lateral-directional LTI plant.
%
%   [SYS, MODEL] = C172SLATERALDIRECTIONALPLANT(DATA) constructs the
%   continuous small-disturbance state-space model
%
%       x = [beta; p; r; phi; psi]
%       u = [deltaA; deltaR]
%
%   at a wings-level trim point. All angles are in radians. SYS requires
%   Control System Toolbox. MODEL always contains A, B, C, D and a ledger
%   of dimensional force/moment derivatives.
%
%   The beta equation is
%
%       betaDot = Y/(m*V) - r + (g/V)*phi
%
%   and the coupled roll/yaw equations retain Ixz through
%
%       [Ixx -Ixz; -Ixz Izz]*[pDot; rDot] = [L; N].

arguments
    data (1,1) struct
end

requiredFields = {'trim','geometry','aero','inertia','constants','model'};
for k = 1:numel(requiredFields)
    if ~isfield(data,requiredFields{k})
        error('c172sLateralDirectionalPlant:MissingData', ...
            'DATA is missing the field "%s".',requiredFields{k});
    end
end

V = data.trim.V_mps;
m = data.trim.mass_kg;
rho = data.trim.rho_kgpm3;
S = data.geometry.S_m2;
b = data.geometry.b_m;
g = data.constants.g;
Ixx = data.inertia.Ixx_kgm2;
Izz = data.inertia.Izz_kgm2;
Ixz = data.inertia.Ixz_kgm2;

if any([V,m,rho,S,b,Ixx,Izz] <= 0)
    error('c172sLateralDirectionalPlant:NonPositiveParameter', ...
        'V, m, rho, S, b, Ixx and Izz must all be positive.');
end

inertiaMatrix = [Ixx,-Ixz;-Ixz,Izz];
if min(eig(inertiaMatrix)) <= 0
    error('c172sLateralDirectionalPlant:InvalidInertia', ...
        'The coupled roll/yaw inertia matrix must be positive definite.');
end

qbar = 0.5*rho*V^2;
qS = qbar*S;
rateScale = b/(2*V);

%% Dimensional derivative ledger

d.Y_beta   = qS*data.aero.CY_beta;
d.Y_p      = qS*data.aero.CY_p*rateScale;
d.Y_r      = qS*data.aero.CY_r*rateScale;
d.Y_deltaA = qS*data.aero.CY_deltaA;
d.Y_deltaR = qS*data.aero.CY_deltaR;

d.L_beta   = qS*b*data.aero.Cl_beta;
d.L_p      = qS*b*data.aero.Cl_p*rateScale;
d.L_r      = qS*b*data.aero.Cl_r*rateScale;
d.L_deltaA = qS*b*data.aero.Cl_deltaA;
d.L_deltaR = qS*b*data.aero.Cl_deltaR;

d.N_beta   = qS*b*data.aero.Cn_beta;
d.N_p      = qS*b*data.aero.Cn_p*rateScale;
d.N_r      = qS*b*data.aero.Cn_r*rateScale;
d.N_deltaA = qS*b*data.aero.Cn_deltaA;
d.N_deltaR = qS*b*data.aero.Cn_deltaR;

%% Assemble xDot = A*x + B*u

A = zeros(5,5);
B = zeros(5,2);

% Sideslip equation, including the -r kinematic term.
A(1,1) = d.Y_beta/(m*V);
A(1,2) = d.Y_p/(m*V);
A(1,3) = d.Y_r/(m*V) - 1.0;
A(1,4) = g/V;
B(1,:) = [d.Y_deltaA,d.Y_deltaR]/(m*V);

% Coupled roll and yaw dynamics. Left division preserves nonzero Ixz.
momentStateDerivatives = [ ...
    d.L_beta,d.L_p,d.L_r; ...
    d.N_beta,d.N_p,d.N_r];
momentControlDerivatives = [ ...
    d.L_deltaA,d.L_deltaR; ...
    d.N_deltaA,d.N_deltaR];

A(2:3,1:3) = inertiaMatrix\momentStateDerivatives;
B(2:3,:) = inertiaMatrix\momentControlDerivatives;

% Level-flight Euler-angle kinematics.
A(4,2) = 1.0;
A(5,3) = 1.0;

C = eye(5);
D = zeros(5,2);

model.A = A;
model.B = B;
model.C = C;
model.D = D;
model.dimensionalDerivatives = d;
model.inertiaMatrix = inertiaMatrix;
model.rateScale_s = rateScale;
model.stateNames = data.model.stateNames;
model.inputNames = data.model.inputNames;
model.data = data;

if exist('ss','file') == 2
    sys = ss(A,B,C,D);
    sys.StateName = data.model.stateNames;
    sys.InputName = data.model.inputNames;
    sys.OutputName = data.model.stateNames;
else
    sys = [];
    warning('c172sLateralDirectionalPlant:NoControlSystemToolbox', ...
        'SS was not found. MODEL matrices were still generated.');
end

end
