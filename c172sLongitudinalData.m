function data = c172sLongitudinalData(cgIn)
%C172SLONGITUDINALDATA Baseline C172S longitudinal-model data.
%
%   DATA = C172SLONGITUDINALDATA() returns the nominal 44.15 in CG case.
%   DATA = C172SLONGITUDINALDATA(CGIN) uses a C172S CG location in inches
%   aft of datum. At 2550 lb, CGIN must lie from 41.0 to 47.3 in.
%
%   Model convention:
%     * SI units internally
%     * aerodynamic angles and elevator deflection in radians
%     * pitching moment positive nose-up
%     * elevator positive trailing-edge down
%     * qHat        = q*cBar/(2*Vtrim)
%     * alphaDotHat = alphaDot*cBar/(2*Vtrim)
%
%   Primary aerodynamic source:
%     Roesch and Harlan, NASA CR-2605 (1975), Appendix A.
%
%   This is a small-disturbance cruise model. The derivatives must not be
%   interpreted as a nonlinear, full-flight-envelope aerodynamic database.

arguments
    cgIn (1,1) double = 44.15
end

%% Unit conversions and physical constants

data.units.ftToM       = 0.3048;
data.units.inToM       = 0.0254;
data.units.ktToMps     = 0.514444444444444;
data.units.lbToN       = 4.4482216152605;
data.constants.g       = 9.80665;       % m/s^2
data.constants.Rair    = 287.05287;     % J/(kg*K)
data.constants.gamma   = 1.4;

%% Baseline operating point: 4000 ft ISA, 110 KTAS, 2550 lb

data.trim.pressureAltitude_ft = 4000.0;
data.trim.altitude_m          = data.trim.pressureAltitude_ft * data.units.ftToM;
data.trim.V_KTAS              = 110.0;
data.trim.V_mps               = data.trim.V_KTAS * data.units.ktToMps;
data.trim.weight_lb           = 2550.0;
data.trim.weight_N            = data.trim.weight_lb * data.units.lbToN;
data.trim.mass_kg             = data.trim.weight_N / data.constants.g;

% ISA troposphere calculation at the pressure altitude.
T0 = 288.15;                  % K
p0 = 101325.0;                % Pa
L  = 0.0065;                  % K/m
T  = T0 - L * data.trim.altitude_m;
p  = p0 * (T/T0)^(data.constants.g/(data.constants.Rair*L));
rho = p/(data.constants.Rair*T);

data.trim.temperature_K = T;
data.trim.pressure_Pa   = p;
data.trim.rho_kgpm3     = rho;
data.trim.qbar_Pa       = 0.5*rho*data.trim.V_mps^2;

%% C172S geometry and certified MTOW CG range

data.geometry.S_m2       = 174.0 * data.units.ftToM^2;
data.geometry.cBar_m     = 58.8 * data.units.inToM;
data.geometry.LEMAC_in   = 25.9;

data.cg.forwardLimit_in  = 41.0;
data.cg.aftLimit_in      = 47.3;
data.cg.nominal_in       = 44.15;

if cgIn < data.cg.forwardLimit_in || cgIn > data.cg.aftLimit_in
    error('c172sLongitudinalData:CGOutsideEnvelope', ...
        ['At 2550 lb, cgIn must lie from %.1f to %.1f in aft of ', ...
         'datum. Received %.3f in.'], ...
        data.cg.forwardLimit_in, data.cg.aftLimit_in, cgIn);
end

data.cg.selected_in = cgIn;
data.cg.h = (cgIn - data.geometry.LEMAC_in)/58.8;

%% Equilibrium aerodynamic coefficients

qS = data.trim.qbar_Pa * data.geometry.S_m2;
data.trim.CL = data.trim.weight_N/qS;

% Performance-tool drag polar: CD = CD0 + k*CL^2.
data.aero.CD0 = 0.0270;
data.aero.k   = 0.053174;
data.trim.CD  = data.aero.CD0 + data.aero.k*data.trim.CL^2;

%% NASA CR-2605 longitudinal perturbation derivatives

data.aero.CL_alpha       =  5.50;
data.aero.CL_alphaDot    =  1.49;
data.aero.CL_q           =  3.88;
data.aero.CD_alpha       =  0.25;
data.aero.Cm_alphaDot    = -4.36;
data.aero.Cm_q           = -11.40;
data.aero.Cm_deltaE      = -1.26;

% CR-2605 omits CL_deltaE. The provisional nominal is the Cessna 182
% value in NASA CR-1975; it will be swept from 0.30 to 0.50 rad^-1.
data.aero.CL_deltaE      =  0.427;
data.uncertainty.CL_deltaE_range = [0.30, 0.50];

% The NASA reference Cm_alpha is -0.83 at hCG=0.33 MAC. Preserve the
% implied neutral point and transform Cm_alpha to the selected C172S CG.
data.aero.CL_alpha_source = data.aero.CL_alpha;
data.cg.hReference        = 0.33;
data.cg.CmAlphaReference  = -0.83;
data.cg.hNeutral = data.cg.hReference ...
    - data.cg.CmAlphaReference/data.aero.CL_alpha_source;
data.cg.staticMargin = data.cg.hNeutral - data.cg.h;
data.aero.Cm_alpha = data.aero.CL_alpha*(data.cg.h - data.cg.hNeutral);

%% Pitch inertia

% NASA CR-2605 gives Iy=1581 kg*m^2 at m=1000 kg. The nominal assumes
% constant radius of gyration when scaled to the 2550-lb operating mass.
data.inertia.Iy_source_kgm2 = 1581.0;
data.inertia.sourceMass_kg  = 1000.0;
data.inertia.Iy_kgm2 = data.inertia.Iy_source_kgm2 ...
    * data.trim.mass_kg/data.inertia.sourceMass_kg;
data.uncertainty.Iy_range_kgm2 = [1800.0, 2800.0];

%% Propulsion derivative policy

% The C172S thrust-speed derivative will later come from the existing
% performance/propeller tool. Zero means no incremental thrust derivative
% in Plant V1; equilibrium thrust is already implicit in the trim point.
data.propulsion.dT_dV_N_per_mps = 0.0;

%% Source and model metadata

data.model.stateNames = {'u_mps','alpha_rad','q_radps','theta_rad'};
data.model.inputName  = 'deltaE_rad_TE_down_positive';
data.model.version    = 'Longitudinal Plant V1';
data.model.primarySource = 'NASA CR-2605, Appendix A, pp. 77-79';
data.model.CGSource = 'FAA TCDS 3A12, C172S';

end
