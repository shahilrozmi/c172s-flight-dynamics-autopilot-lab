function damper = c172sPitchRateDamperData()
%C172SPITCHRATEDAMPERDATA Pitch-rate damper definition and requirements.
%
%   Feedback law, using the project sign convention:
%
%       deltaE = deltaECommand + Kq*q
%
%   Positive q is nose-up. Positive deltaE is elevator trailing-edge down,
%   which produces a nose-down moment because Cm_deltaE is negative.
%   Therefore Kq > 0 supplies damping feedback.

damper.version = 'Pitch-Rate Damper V1';
damper.Kq_s = 0.110;
damper.feedbackLaw = 'deltaE = deltaECommand + Kq*q';
damper.qFeedbackRow = [0, 0, damper.Kq_s, 0];

damper.requirements.minimumShortPeriodZeta = 0.70;
damper.requirements.maximumShortPeriodZeta = 1.00;
damper.requirements.allCasesStable = true;
damper.requirements.allCasesOscillatory = true;

damper.design.gainSearchRange_s = [0.0, 0.14];
damper.design.gainSearchIncrement_s = 0.0005;
damper.design.description = [ ...
    'Smallest robust qualifying gain plus margin; evaluated across ', ...
    'the 36-case CG, Iy and CL_deltaE uncertainty grid.'];

end
