function data = c172sNonlinear6DofSourceData()
%C172SNONLINEAR6DOFSOURCEDATA Source and convention contract for 6-DOF V0.1.
%
% This file does not define a nonlinear aerodynamic truth model. It freezes
% the coordinate system, state ordering, initial validation point, evidence
% hierarchy, accepted anchors and unresolved data gaps that must govern the
% future C172S nonlinear six-degree-of-freedom research plant.

%% Artifact and frozen-release boundary

data.artifactRevision = ...
    'NONLINEAR-6DOF-V1-SOURCE-AND-CONVENTION-AUDIT-MATLAB-R1';
data.releaseDate = '2026-08-31';
data.required.longitudinalPlant = 'Longitudinal Plant V1';
data.required.lateralPlant = 'Lateral-Directional Plant V0.1';
data.required.masterRegressionPasses = 18;
data.policy.modifyFrozenRelease = false;
data.policy.claimAircraftApproval = false;
data.policy.claimTruthModel = false;
data.policy.researchPlantName = ...
    'C172S Nonlinear 6-DOF Research Validation Plant V0.1';

%% Units, reference frame and state contract

data.units.internal = 'SI';
data.units.angle = 'rad';
data.units.angularRate = 'rad/s';
data.units.position = 'm';
data.units.velocity = 'm/s';
data.units.force = 'N';
data.units.moment = 'N*m';

data.axes.frame = 'right-handed body axes';
data.axes.x = 'forward';
data.axes.y = 'right wing';
data.axes.z = 'down';
data.axes.navigation = 'NED';
data.axes.p = 'right-wing-down positive';
data.axes.q = 'nose-up positive';
data.axes.r = 'nose-right positive';
data.axes.phi = 'right-wing-down positive';
data.axes.theta = 'nose-up positive';
data.axes.psi = 'clockwise from north / nose-right positive';
data.axes.altitudeDefinition = 'altitude_m = -positionDown_m';

data.state.names = { ...
    'u_mps','v_mps','w_mps', ...
    'p_radps','q_radps','r_radps', ...
    'positionNorth_m','positionEast_m','positionDown_m', ...
    'q0','q1','q2','q3'};
data.state.count = 13;
data.state.quaternionOrder = 'scalar-first body-to-NED';
data.state.quaternionConstraint = 'q0^2+q1^2+q2^2+q3^2 = 1';

%% Control convention contract

data.controls.deltaE = 'positive trailing-edge down';
data.controls.deltaA = 'positive right-wing-down command';
data.controls.deltaR = 'positive nose-right command';
data.controls.throttle = '0 to 1';
data.controls.expectedSigns.Cm_deltaE = -1;
data.controls.expectedSigns.Cl_deltaA = 1;
data.controls.expectedSigns.Cn_deltaR = 1;

%% Initial validation point shared with the frozen plants

data.trim.pressureAltitude_ft = 4000.0;
data.trim.V_KTAS = 110.0;
data.trim.weight_lb = 2550.0;
data.trim.flap_deg = 0.0;
data.trim.bank_deg = 0.0;
data.trim.flightPath_deg = 0.0;
data.trim.ISA_deviation_K = 0.0;

%% Aerodynamic normalization

data.normalization.pHat = 'p*b/(2*V)';
data.normalization.qHat = 'q*cBar/(2*V)';
data.normalization.rHat = 'r*b/(2*V)';
data.normalization.alphaDotHat = 'alphaDot*cBar/(2*V)';
data.normalization.derivativeAngles = 'per radian';
data.normalization.dynamicPressure = 'qbar = 0.5*rho*V^2';

%% Evidence tiers

data.evidence.tierA = ...
    'Direct C172S certified/manufacturer geometry, limits or performance';
data.evidence.tierB = ...
    'Measured flight-test data from another documented C172 variant';
data.evidence.tierC = ...
    'Peer-reviewed analytical model or traceable educational database';
data.evidence.tierD = ...
    'Community simulation model, engineering estimate or project assumption';
data.evidence.acceptanceOrder = {'A','B','C','D'};

%% Source ledger

data.sources(1) = sourceRecord( ...
    'FAA-TCDS-3A12','FAA TCDS 3A12','C172S','A', ...
    'Certified geometry, loading and operating limits', ...
    'Use direct C172S entries only; retain revision and page traceability', ...
    'https://drs.faa.gov/','C172S model entries in current TCDS 3A12', ...
    'United States government publication');
data.sources(2) = sourceRecord( ...
    'NASA-CR-2605','NASA CR-2605 Appendix A','C172 research configuration','B', ...
    'Frozen longitudinal small-disturbance derivative anchor', ...
    'Already adapted and uncertainty-swept in Longitudinal Plant V1', ...
    'https://ntrs.nasa.gov/citations/19760003290', ...
    'Appendix A, report pp. 77-79','United States government report');
data.sources(3) = sourceRecord( ...
    'NASA-CR-168912','NASA CR-168912 / KU-FRL-407-7','1974 C172M N12800','B', ...
    '35 flight-test maneuvers and MMLE derivative estimates', ...
    'Transcribe Tables 6.3 and 6.4, report pp. 97-98; never replace them by one composite row', ...
    'https://ntrs.nasa.gov/citations/19820015371', ...
    'Section 6.6; Tables 6.3/6.4, report pp. 97-98', ...
    'United States government report');
data.sources(4) = sourceRecord( ...
    'KASNAKOGLU-2016','PLOS ONE 11(10):e0165017','published C172 case','C', ...
    'Frozen lateral-directional derivative anchor', ...
    'Analytical/simulation source case, not a C172S flight-test identification', ...
    'https://doi.org/10.1371/journal.pone.0165017', ...
    'Tables 1-2 and supporting information','CC BY 4.0');
data.sources(5) = sourceRecord( ...
    'UIUC-C172','UIUC Applied Aerodynamics C172 models','legacy C172/FlightGear','C', ...
    'Candidate nonlinear coefficient curve shapes', ...
    'May define shape only until re-anchored and independently validated', ...
    'https://m-selig.ae.illinois.edu/apasim/Aircraft-uiuc.html', ...
    'cessna172-71-v2, cessna172-73-v3 and cessna172-aae319-v4', ...
    'Verify individual model-file reuse terms before redistribution');
data.sources(6) = sourceRecord( ...
    'JSBSIM-C172X','JSBSim c172x aircraft definition','1982 C172P-style model','D', ...
    'Architecture, propulsion form and candidate mass-property comparison', ...
    'Community FDM; values are estimates, not certified C172S data', ...
    'https://github.com/JSBSim-Team/jsbsim/blob/master/aircraft/c172x/c172x.xml', ...
    'aircraft/c172x/c172x.xml','GPL aircraft-definition file');

%% Explicitly rejected/misrepresented numerical package

data.rejected.geminiCompositeNASAValues = [ ...
     4.5800, -0.6500, -12.4000, -1.2800, ...
    -0.3100, -0.0890, -0.4700, 0.0650, -0.0990];
data.rejected.reason = [ ...
    'The one-row coefficient package attributed to NASA CR-168912 is ', ...
    'not a transcription of Tables 6.3/6.4 and shall not enter V0.1.'];
data.rejected.NASAReportPageClaim = 'pp. 25-42';
data.rejected.correctNASAReportTables = ...
    'Tables 6.3 and 6.4, report pp. 97-98';

%% Preliminary mass-property policy

data.massProperties.nominalSource = ...
    'Frozen Kasnakoglu/JSBSim-derived project anchor';
data.massProperties.status = 'provisional engineering estimate';
data.massProperties.IxzStatus = 'open validation item';
data.massProperties.requirePositiveDefiniteTensor = true;
data.massProperties.requireWeightCGSchedule = true;
data.massProperties.uncertaintyFraction = 0.20;

%% V0.1 claim boundary (project assumptions, not aircraft limits)

data.envelope.status = 'provisional attached-flow validation boundary';
data.envelope.V_KTAS = [80.0,130.0];
data.envelope.alpha_deg = [-4.0,10.0];
data.envelope.beta_deg = [-8.0,8.0];
data.envelope.bank_deg = [-30.0,30.0];
data.envelope.pitch_deg = [-10.0,10.0];
data.envelope.flap_deg = 0.0;
data.envelope.projectAssumption = true;
data.envelope.stallValidated = false;
data.envelope.spinValidated = false;
data.envelope.postStallValidated = false;

%% Readiness and mandatory next gates

data.readiness.nearTrimNormalEnvelope = 'sufficient to begin implementation';
data.readiness.multiTrimNormalEnvelope = 'partially sufficient';
data.readiness.stall = 'insufficient for validation claims';
data.readiness.spinPostStall = 'insufficient';
data.readiness.readyForEquationsOfMotion = true;
data.readiness.readyForFinalAeroDatabase = false;

data.nextGates = { ...
    'Digitize and independently check all NASA CR-168912 Table 6.3/6.4 rows'; ...
    'Reconcile geometry values and reference stations to one C172S revision'; ...
    'Construct weight/CG-dependent inertia tensor with uncertainty bounds'; ...
    'Calibrate engine/propeller model against C172S POH performance'; ...
    'Match nonlinear numerical Jacobians to both frozen linear plants at trim'; ...
    'Validate trim, energy, quaternion norm and force/moment sign invariants'; ...
    'Keep stall and spin claims disabled until separate evidence gates pass'};

data.validation.minimumSourceCount = 6;
data.validation.minimumNextGateCount = 7;
data.validation.maximumFrozenLinearizationRelativeError = 1.0e-6;
data.validation.maximumQuaternionNormError = 1.0e-12;
data.validation.requireDeterministicReplay = true;

end

function source = sourceRecord(id,title,variant,tier,useText,qualification, ...
    url,locator,reuse)
source.id = id;
source.title = title;
source.variant = variant;
source.tier = tier;
source.intendedUse = useText;
source.qualification = qualification;
source.url = url;
source.locator = locator;
source.reuse = reuse;
end
