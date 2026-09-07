function solution = solveC172sNonlinearTrim(initialGuess,model)
%SOLVEC172SNONLINEARTRIM Bounded Newton solve for level near-trim flight.

if nargin < 2 || isempty(model)
    model = c172sTrimTotalForceData();
end
if nargin < 1 || isempty(initialGuess)
    initialGuess = model.solver.defaultInitialGuess;
end
validateattributes(initialGuess,{'double'}, ...
    {'real','finite','column','numel',3},mfilename,'initialGuess',1);
if any(initialGuess < model.solver.lower) || ...
        any(initialGuess > model.solver.upper)
    error('solveC172sNonlinearTrim:InitialGuessOutsideBounds', ...
        'The initial trim guess lies outside the declared bounds.');
end

z = initialGuess;
converged = false;
for iteration = 1:model.solver.maximumIterations
    [residual,detail] = trimResidual(z,model);
    scaled = residual./model.solver.residualScale;
    if norm(scaled,inf) <= model.solver.maximumScaledResidual
        converged = true;
        break
    end
    jacobian = zeros(3);
    for column = 1:3
        h = model.solver.finiteDifferenceStep(column);
        zp = z; zm = z;
        zp(column) = min(model.solver.upper(column),z(column)+h);
        zm(column) = max(model.solver.lower(column),z(column)-h);
        if zp(column) == zm(column)
            error('solveC172sNonlinearTrim:DegenerateDifference', ...
                'A finite-difference trim direction has zero width.');
        end
        rp = trimResidual(zp,model)./model.solver.residualScale;
        rm = trimResidual(zm,model)./model.solver.residualScale;
        jacobian(:,column) = (rp-rm)/(zp(column)-zm(column));
    end
    if rcond(jacobian) < 1e-12
        error('solveC172sNonlinearTrim:SingularJacobian', ...
            'The trim Jacobian is singular or ill-conditioned.');
    end
    step = -jacobian\scaled;
    accepted = false;
    for lineSearch = 0:model.solver.maximumLineSearchIterations
        factor = 0.5^lineSearch;
        candidate = min(model.solver.upper,max(model.solver.lower,z+factor*step));
        candidateResidual = trimResidual(candidate,model)./ ...
            model.solver.residualScale;
        if norm(candidateResidual,2) < norm(scaled,2)
            z = candidate;
            accepted = true;
            break
        end
    end
    if ~accepted || norm(factor*step,inf) < model.solver.minimumStepNorm
        break
    end
end
if ~converged
    error('solveC172sNonlinearTrim:NoConvergence', ...
        'No bounded trim solution met the residual gate.');
end

[residual,detail] = trimResidual(z,model);
solution.artifactRevision = model.artifactRevision;
solution.converged = true;
solution.iterations = iteration;
solution.unknowns = z;
solution.alpha_rad = z(1);
solution.elevator_rad = z(2);
solution.throttle = z(3);
solution.state = detail.state;
solution.controls = detail.controls;
solution.totalForces = detail.totalForces;
solution.residual = residual;
solution.scaledResidual = residual./model.solver.residualScale;
solution.theta_rad = z(1)+model.trim.flightPath_rad;
solution.flightPath_rad = model.trim.flightPath_rad;
solution.referenceAxisQualification = model.referenceAxis.definition;

end

function [residual,detail] = trimResidual(z,model)
alpha = z(1);
theta = alpha+model.trim.flightPath_rad;
V = model.trim.TAS_mps;
q = [cos(theta/2);0;sin(theta/2);0];
state = [V*cos(alpha);0;V*sin(alpha);zeros(3,1); ...
    0;0;-model.trim.altitude_m;q];
controls = [z(2);0;0;z(3)];
totalForces = c172sTrimTotalForces(state,controls,0, ...
    model.trim.steadyWindNED_mps,model.trim.gustNED_mps,model);
xdot = totalForces.stateDerivative;
residual = [xdot(1);xdot(3);xdot(5)];
detail.state = state;
detail.controls = controls;
detail.totalForces = totalForces;
end
