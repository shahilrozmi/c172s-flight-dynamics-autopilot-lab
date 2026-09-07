function data = c172sTrimTotalForceData()
%C172STRIMTOTALFORCEDATA Contract for the V0.1 trim/total-force layer.

source = c172sNonlinear6DofSourceData();
kernel = c172sRigidBody6DofData();
environment = c172sAtmosphereAirDataData();
aero = c172sNearTrimAerodynamicsData();
elevator = c172sElevatorActuatorData();

data.artifactRevision = ...
    'NONLINEAR-6DOF-V1-TRIM-TOTAL-FORCE-MATLAB-R1';
data.required.sourceAudit = source.artifactRevision;
data.required.rigidBodyKernel = kernel.artifactRevision;
data.required.atmosphereAirData = environment.artifactRevision;
data.required.nearTrimAerodynamics = aero.artifactRevision;
data.required.longitudinalPlant = aero.required.longitudinalPlant;
data.required.lateralPlant = aero.required.lateralPlant;
data.required.masterRegressionPasses = 18;

data.kernel = kernel;
data.aerodynamics = aero;
data.trim.altitude_m = aero.trim.altitude_m;
data.trim.TAS_mps = aero.trim.V_mps;
data.trim.flightPath_rad = 0.0;
data.trim.bank_rad = 0.0;
data.trim.heading_rad = 0.0;
data.trim.steadyWindNED_mps = zeros(3,1);
data.trim.gustNED_mps = zeros(3,1);

% The frozen linear plants define alpha=0 at their reference equilibrium.
% V0.1 therefore uses an aerodynamic reference axis aligned with the trim
% velocity. This is not a claim that fuselage incidence or measured alpha
% is zero on the aircraft.
data.referenceAxis.definition = ...
    'aerodynamic reference axis aligned with the frozen trim velocity';
data.referenceAxis.fuselageIncidenceIdentified = false;
data.referenceAxis.absoluteAircraftAlphaClaim = false;

qbarS = aero.trim.qbar_Pa*aero.geometry.S_m2;
data.propulsion.trimDrag_N = qbarS*aero.trim.CD;
data.propulsion.nominalTrimThrottle = 0.65;
data.propulsion.maximumThrustAtTrim_N = ...
    data.propulsion.trimDrag_N/data.propulsion.nominalTrimThrottle;
data.propulsion.model = 'T = throttle*Tmax at the single frozen trim datum';
data.propulsion.status = 'calibrated one-point engineering assumption';
data.propulsion.speedScheduleImplemented = false;
data.propulsion.altitudeScheduleImplemented = false;
data.propulsion.propellerMapImplemented = false;
data.propulsion.slipstreamImplemented = false;
data.propulsion.momentImplemented = false;

data.controls.names = {'deltaE_rad_TE_down','deltaA_rad_rightWingDown', ...
    'deltaR_rad_noseRight','throttle_0_to_1'};
data.controls.lower = [elevator.authority.lowerLimit_rad; ...
    deg2rad(-5);deg2rad(-5);0];
data.controls.upper = [elevator.authority.upperLimit_rad; ...
    deg2rad(5);deg2rad(5);1];
data.controls.elevatorBoundSource = elevator.version;

data.solver.unknownNames = {'alpha_rad','deltaE_rad','throttle'};
data.solver.lower = [aero.envelope.alpha_rad(1); ...
    data.controls.lower(1);0];
data.solver.upper = [aero.envelope.alpha_rad(2); ...
    data.controls.upper(1);1];
data.solver.defaultInitialGuess = [deg2rad(1);deg2rad(-1);0.60];
data.solver.maximumIterations = 30;
data.solver.maximumLineSearchIterations = 16;
data.solver.finiteDifferenceStep = [1e-6;1e-6;1e-6];
data.solver.residualScale = [kernel.gravity_mps2; ...
    kernel.gravity_mps2;1.0];
data.solver.maximumScaledResidual = 1e-10;
data.solver.minimumStepNorm = 1e-12;

data.validation.maximumForceResidual_N = 1e-7;
data.validation.maximumMomentResidual_Nm = 1e-7;
data.validation.maximumStateDerivative = 1e-9;
data.validation.maximumTrimSpread = 1e-8;
data.validation.requireDeterministicReplay = true;
data.validation.fullDynamicsLinearizationDeferred = true;
data.validation.coefficientSchedulingIncluded = false;
data.validation.stallIncluded = false;
data.notForAircraftApproval = true;

end
