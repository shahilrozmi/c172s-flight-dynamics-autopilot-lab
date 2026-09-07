function [xDot,aux] = c172sRigidBody6DofEom( ...
    x,forceBody_N,momentBody_Nm,model)
%C172SRIGIDBODY6DOFEOM Nonlinear flat-Earth rigid-body equations of motion.
%
% State order:
%   x = [u v w p q r pN pE pD q0 q1 q2 q3].'
%
% Conventions:
%   * SI units
%   * right-handed body axes: x forward, y right, z down
%   * NED navigation frame
%   * scalar-first quaternion maps body vectors into NED
%   * forceBody_N excludes gravity
%   * momentBody_Nm is about the center of mass

validateattributes(x,{'double'},{'real','finite','column','numel',13}, ...
    mfilename,'x',1);
validateattributes(forceBody_N,{'double'}, ...
    {'real','finite','column','numel',3},mfilename,'forceBody_N',2);
validateattributes(momentBody_Nm,{'double'}, ...
    {'real','finite','column','numel',3},mfilename,'momentBody_Nm',3);

required = {'mass_kg','gravity_mps2','inertia','quaternion'};
for k = 1:numel(required)
    if ~isfield(model,required{k})
        error('c172sRigidBody6DofEom:MissingModelField', ...
            'model.%s is required.',required{k});
    end
end
validateattributes(model.mass_kg,{'double'}, ...
    {'real','finite','scalar','positive'},mfilename,'model.mass_kg');
validateattributes(model.gravity_mps2,{'double'}, ...
    {'real','finite','scalar','nonnegative'},mfilename,'model.gravity_mps2');

if ~isfield(model.inertia,'matrix_kgm2')
    error('c172sRigidBody6DofEom:MissingInertia', ...
        'model.inertia.matrix_kgm2 is required.');
end
inertia = model.inertia.matrix_kgm2;
validateattributes(inertia,{'double'}, ...
    {'real','finite','size',[3 3]},mfilename,'model.inertia.matrix_kgm2');
if norm(inertia-inertia.','fro') > 1.0e-10*max(1,norm(inertia,'fro'))
    error('c172sRigidBody6DofEom:NonSymmetricInertia', ...
        'The inertia tensor must be symmetric.');
end
[~,cholStatus] = chol(inertia);
if cholStatus ~= 0
    error('c172sRigidBody6DofEom:NonPositiveDefiniteInertia', ...
        'The inertia tensor must be positive definite.');
end

velocityBody_mps = x(1:3);
omegaBody_radps = x(4:6);
quaternion = x(10:13);
quaternionNorm = norm(quaternion);
if quaternionNorm < model.quaternion.minimumNorm
    error('c172sRigidBody6DofEom:InvalidQuaternion', ...
        'The quaternion norm is too small.');
end
quaternionNormError = abs(quaternionNorm-1.0);
if quaternionNormError > model.quaternion.maximumInputNormError
    error('c172sRigidBody6DofEom:QuaternionNotUnit', ...
        'Quaternion norm error %.3g exceeds %.3g.', ...
        quaternionNormError,model.quaternion.maximumInputNormError);
end
quaternion = quaternion/quaternionNorm;

bodyToNED = quaternionToDCM(quaternion);
gravityNED_mps2 = [0;0;model.gravity_mps2];
gravityBody_mps2 = bodyToNED.'*gravityNED_mps2;

velocityDotBody_mps2 = forceBody_N/model.mass_kg + ...
    gravityBody_mps2-cross(omegaBody_radps,velocityBody_mps);
angularMomentumBody = inertia*omegaBody_radps;
omegaDotBody_radps2 = inertia\(momentBody_Nm- ...
    cross(omegaBody_radps,angularMomentumBody));
positionDotNED_mps = bodyToNED*velocityBody_mps;

q0 = quaternion(1);
qVector = quaternion(2:4);
quaternionDot = 0.5*[ ...
    -qVector.'; ...
    q0*eye(3)+skew(qVector)]*omegaBody_radps;

xDot = [velocityDotBody_mps2;omegaDotBody_radps2; ...
    positionDotNED_mps;quaternionDot];

if nargout > 1
    aux.bodyToNED = bodyToNED;
    aux.gravityBody_mps2 = gravityBody_mps2;
    aux.angularMomentumBody_kgm2ps = angularMomentumBody;
    aux.translationalKineticEnergy_J = ...
        0.5*model.mass_kg*(velocityBody_mps.'*velocityBody_mps);
    aux.rotationalKineticEnergy_J = ...
        0.5*omegaBody_radps.'*angularMomentumBody;
    aux.quaternionNormError = quaternionNormError;
end

end

function matrix = quaternionToDCM(q)
q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
matrix = [ ...
    q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2); ...
    2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1); ...
    2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];
end

function matrix = skew(vector)
matrix = [0,-vector(3),vector(2); ...
    vector(3),0,-vector(1); ...
    -vector(2),vector(1),0];
end
