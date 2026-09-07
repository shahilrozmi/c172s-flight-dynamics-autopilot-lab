function session = START_HERE()
%START_HERE Verify and launch the C172S Flight Dynamics/Autopilot Lab.
%
%   SESSION = START_HERE performs a non-simulating installation preflight,
%   places this release folder first on the MATLAB path for the duration of
%   the session, and opens the public laboratory menu.

releaseRoot = fileparts(mfilename('fullpath'));
originalPath = path;
pathCleanup = onCleanup(@() path(originalPath)); %#ok<NASGU>
addpath(releaseRoot,'-begin');

verifyC172sFlightDynamicsAutopilotLabInstallation;
session = mainC172sFlightControlLab;
end
