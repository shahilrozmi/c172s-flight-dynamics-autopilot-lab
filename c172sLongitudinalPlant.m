function [sys, model] = c172sLongitudinalPlant(data)
%C172SLONGITUDINALPLANT Construct the C172S longitudinal LTI plant.
%
%   [SYS, MODEL] = C172SLONGITUDINALPLANT(DATA) constructs the continuous
%   small-disturbance state-space model
%
%       x = [u; alpha; q; theta]
%       input = deltaE, positive trailing-edge down
%
%   where u is the perturbation in flight-path speed. All angles are in
%   radians. SYS requires Control System Toolbox. MODEL always contains
%   A, B, C, D and the dimensional derivative ledger.
%
%   Equations used at a level trim point:
%
%     m*uDot = dT - dD - m*g*(theta - alpha)
%     m*V*(q - alphaDot) = dL
%     Iy*qDot = dM
%     thetaDot = q
%
%   The alphaDot lift and moment terms are retained algebraically rather
%   than discarded.

arguments
    data (1,1) struct
end

requiredFields = {'trim','geometry','aero','inertia','propulsion','constants'};
for k = 1:numel(requiredFields)
    if ~isfield(data, requiredFields{k})
        error('c172sLongitudinalPlant:MissingData', ...
            'DATA is missing the field "%s".', requiredFields{k});
    end
end

V   = data.trim.V_mps;
m   = data.trim.mass_kg;
rho = data.trim.rho_kgpm3;
S   = data.geometry.S_m2;
c   = data.geometry.cBar_m;
Iy  = data.inertia.Iy_kgm2;
g   = data.constants.g;

if any([V,m,rho,S,c,Iy] <= 0)
    error('c172sLongitudinalPlant:NonPositiveParameter', ...
        'V, m, rho, S, c and Iy must all be positive.');
end

qbar = 0.5*rho*V^2;
qS   = qbar*S;

%% Dimensional force and moment derivatives

% Drag and thrust derivatives with respect to speed perturbation u.
d.D_u = qS*(2*data.trim.CD/V);             % N/(m/s)
d.T_u = data.propulsion.dT_dV_N_per_mps;   % N/(m/s)
d.D_alpha = qS*data.aero.CD_alpha;         % N/rad

% Lift derivatives. Rate derivatives include c/(2V) normalization.
d.L_u        = qS*(2*data.trim.CL/V);                       % N/(m/s)
d.L_alpha    = qS*data.aero.CL_alpha;                       % N/rad
d.L_q        = qS*data.aero.CL_q*c/(2*V);                   % N/(rad/s)
d.L_alphaDot = qS*data.aero.CL_alphaDot*c/(2*V);            % N/(rad/s)
d.L_deltaE   = qS*data.aero.CL_deltaE;                      % N/rad

% Pitching-moment derivatives.
d.M_alpha    = qS*c*data.aero.Cm_alpha;                     % N*m/rad
d.M_q        = qS*c*data.aero.Cm_q*c/(2*V);                 % N*m/(rad/s)
d.M_alphaDot = qS*c*data.aero.Cm_alphaDot*c/(2*V);          % N*m/(rad/s)
d.M_deltaE   = qS*c*data.aero.Cm_deltaE;                    % N*m/rad

%% Solve the alpha equation while retaining L_alphaDot

alphaDenominator = m*V + d.L_alphaDot;

aAlpha = zeros(1,4);
aAlpha(1) = -d.L_u/alphaDenominator;
aAlpha(2) = -d.L_alpha/alphaDenominator;
aAlpha(3) = (m*V - d.L_q)/alphaDenominator;
bAlpha    = -d.L_deltaE/alphaDenominator;

%% Assemble xDot = A*x + B*deltaE

A = zeros(4,4);
B = zeros(4,1);

% Speed equation in flight-path axes.
A(1,1) = (d.T_u - d.D_u)/m;
A(1,2) = g - d.D_alpha/m;
A(1,4) = -g;

% Angle-of-attack equation.
A(2,:) = aAlpha;
B(2)   = bAlpha;

% Pitch equation. The M_alphaDot*alphaDot contribution is substituted.
A(3,:) = (d.M_alphaDot/Iy)*aAlpha;
A(3,2) = A(3,2) + d.M_alpha/Iy;
A(3,3) = A(3,3) + d.M_q/Iy;
B(3)   = d.M_deltaE/Iy + (d.M_alphaDot/Iy)*bAlpha;

% Kinematics.
A(4,3) = 1.0;

C = eye(4);
D = zeros(4,1);

model.A = A;
model.B = B;
model.C = C;
model.D = D;
model.dimensionalDerivatives = d;
model.alphaDenominator = alphaDenominator;
model.stateNames = data.model.stateNames;
model.inputName = data.model.inputName;
model.data = data;

% Create the LTI object when Control System Toolbox is available.
if exist('ss','file') == 2
    sys = ss(A,B,C,D);
    sys.StateName = data.model.stateNames;
    sys.InputName = {data.model.inputName};
    sys.OutputName = data.model.stateNames;
else
    sys = [];
    warning('c172sLongitudinalPlant:NoControlSystemToolbox', ...
        'SS was not found. MODEL matrices were still generated.');
end

end
