function data = c172sLateralDirectionalData()
%C172SLATERALDIRECTIONALDATA Provisional C172S lateral-directional data.
%
%   DATA = C172SLATERALDIRECTIONALDATA() returns the research baseline used
%   for the first lateral-directional plant. The project operating point is
%   4000 ft ISA, 110 KTAS and 2550 lb, matching Longitudinal Plant V1.
%
%   Model convention:
%     * SI units internally
%     * states x = [beta; p; r; phi; psi]
%     * body axes x forward, y right and z down
%     * beta positive for velocity toward body +y
%     * p and phi positive right-wing-down
%     * r and psi positive nose-right
%     * deltaA positive for a right-wing-down roll command
%     * deltaR positive for a nose-right yaw command
%     * pHat = p*b/(2*Vtrim), rHat = r*b/(2*Vtrim)
%
%   Primary published source:
%     C. Kasnakoglu, PLOS ONE 11(10), e0165017 (2016), Tables 1-2,
%     DOI 10.1371/journal.pone.0165017.
%
%   IMPORTANT: the source case is 65 m/s, 1000 m and 1043.3 kg. This
%   project reuses the published dimensionless derivatives at its existing
%   cruise point and scales the source inertias by mass, assuming unchanged
%   radii of gyration. Ixz remains zero because the source explicitly uses
%   Jxz = 0. These are transparent research assumptions, not flight-test
%   identification or certification data.

%% Unit conversions and physical constants

data.units.ftToM   = 0.3048;
data.units.inToM   = 0.0254;
data.units.ktToMps = 0.514444444444444;
data.units.lbToN   = 4.4482216152605;
data.constants.g   = 9.80665;       % m/s^2
data.constants.Rair = 287.05287;    % J/(kg*K)

%% Project operating point: identical to Longitudinal Plant V1

data.trim.pressureAltitude_ft = 4000.0;
data.trim.altitude_m = data.trim.pressureAltitude_ft*data.units.ftToM;
data.trim.V_KTAS = 110.0;
data.trim.V_mps = data.trim.V_KTAS*data.units.ktToMps;
data.trim.weight_lb = 2550.0;
data.trim.weight_N = data.trim.weight_lb*data.units.lbToN;
data.trim.mass_kg = data.trim.weight_N/data.constants.g;
data.trim.theta_rad = 0.0;

% ISA troposphere calculation at pressure altitude.
T0 = 288.15;                  % K
p0 = 101325.0;                % Pa
L = 0.0065;                   % K/m
T = T0 - L*data.trim.altitude_m;
p = p0*(T/T0)^(data.constants.g/(data.constants.Rair*L));
rho = p/(data.constants.Rair*T);

data.trim.temperature_K = T;
data.trim.pressure_Pa = p;
data.trim.rho_kgpm3 = rho;
data.trim.qbar_Pa = 0.5*rho*data.trim.V_mps^2;

%% C172 geometry shared with Longitudinal Plant V1

data.geometry.S_m2 = 174.0*data.units.ftToM^2;
data.geometry.b_m = 35.8*data.units.ftToM;
data.geometry.cBar_m = 58.8*data.units.inToM;

%% Published source case and inertia adaptation

data.sourceCase.V_mps = 65.0;
data.sourceCase.altitude_m = 1000.0;
data.sourceCase.mass_kg = 1043.3;
data.sourceCase.Ixx_kgm2 = 1285.3;
data.sourceCase.Iyy_kgm2 = 1824.9;
data.sourceCase.Izz_kgm2 = 2666.9;
data.sourceCase.Ixz_kgm2 = 0.0;
data.sourceCase.S_m2 = 16.1651;
data.sourceCase.b_m = 10.9118;
data.sourceCase.cBar_m = 1.4935;

data.inertia.massScale = data.trim.mass_kg/data.sourceCase.mass_kg;
data.inertia.Ixx_kgm2 = data.sourceCase.Ixx_kgm2*data.inertia.massScale;
data.inertia.Iyy_kgm2 = data.sourceCase.Iyy_kgm2*data.inertia.massScale;
data.inertia.Izz_kgm2 = data.sourceCase.Izz_kgm2*data.inertia.massScale;
data.inertia.Ixz_kgm2 = data.sourceCase.Ixz_kgm2*data.inertia.massScale;

%% Published dimensionless lateral-directional derivatives

% State derivatives are copied directly from Table 2.
data.aero.CY_beta = -0.3100;
data.aero.CY_p    = -0.0370;
data.aero.CY_r    =  0.2100;

data.aero.Cl_beta = -0.0890;
data.aero.Cl_p    = -0.4700;
data.aero.Cl_r    =  0.0960;

data.aero.Cn_beta =  0.0650;
data.aero.Cn_p    = -0.0300;
data.aero.Cn_r    = -0.0990;

% Preserve the published control derivatives in a separate ledger.
data.publishedControl.CY_deltaA =  0.0000;
data.publishedControl.CY_deltaR =  0.1870;
data.publishedControl.Cl_deltaA = -0.1780;
data.publishedControl.Cl_deltaR =  0.0147;
data.publishedControl.Cn_deltaA = -0.0530;
data.publishedControl.Cn_deltaR = -0.0657;

% The source control columns are negated to implement the project command
% convention stated above. In matrix form, uSource = -eye(2)*uProject.
data.inputMapping.projectToPublished = -eye(2);

data.aero.CY_deltaA = -data.publishedControl.CY_deltaA;
data.aero.CY_deltaR = -data.publishedControl.CY_deltaR;
data.aero.Cl_deltaA = -data.publishedControl.Cl_deltaA;
data.aero.Cl_deltaR = -data.publishedControl.Cl_deltaR;
data.aero.Cn_deltaA = -data.publishedControl.Cn_deltaA;
data.aero.Cn_deltaR = -data.publishedControl.Cn_deltaR;

%% Traceability and uncertainty metadata

data.uncertainty.derivativePolicy = [ ...
    'Published dimensionless derivatives are applied at the project ', ...
    'cruise point without Mach/Reynolds correction.'];
data.uncertainty.inertiaPolicy = [ ...
    'Published inertias are scaled by mass with constant radii of gyration.'];
data.uncertainty.IxzPolicy = [ ...
    'Ixz = 0 follows the source nominal and remains an open validation item.'];
data.uncertainty.recommendedParametricSweep_fraction = 0.20;

data.model.stateNames = { ...
    'beta_rad','p_radps','r_radps','phi_rad','psi_rad'};
data.model.inputNames = { ...
    'deltaA_rad_rightWingDown','deltaR_rad_noseRight'};
data.model.version = 'Lateral-Directional Plant V0.1';
data.model.releaseStatus = 'Provisional research baseline';
data.model.primarySource = [ ...
    'Kasnakoglu (2016), PLOS ONE 11(10):e0165017, Tables 1-2'];
data.model.primarySourceDOI = '10.1371/journal.pone.0165017';
data.model.notForCertification = true;

end
