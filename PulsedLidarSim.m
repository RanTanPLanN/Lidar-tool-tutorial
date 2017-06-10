%% Pulsed LIDAR simulator for MSc thesis
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% PURPOSE: Write a stand-alone pulsed LIDAR simulator tool to use in the
% MSc thesis. The tool must be as generic as possible.
%
% The user inputs should be the LIDAR position, the elevation angle of
% the LIDAR beam (theta), the azimuthial angle (phi) over which the LIDAR will
% scan, the exact location of interest, the probe length, and the range
% spacing (gap between the range gates).
%
% The output(s) should be the reconstructed velocity at that specific point
%
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Created: February 2nd, 2017
% Last edited: February 13, 2017
% Author: Nikolaos Frouzakis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%% User input section %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear all
clc

%% Operational mode selection

% operational mode: 
% Select 'p' for PPI or 's' for staring mode.
operationMode = 's';

%% Selection of weighting function shape

% choose type of weighting function:
% 't' for triangular-shaped function
% 'g' for gaussian-shaped function
weightingFuncType = 't';

%% Number of LIDARs

% choose number of LIDARs that participate in the campaign (2 or 3)
nLidars = 2;

%% Dummies - These variable are just for testing

% The user is asked to insert both rotor radius in order to normalize all 
% dimensions w.r.t. that.

% Wind turbine rotor radius [in metres]
R = 40;

% mean wind speed (m/s)
meanWindSpeed = 12;

% Dimensions of dummy domain. The dimensions are later normalized w.r.t. R.
xDimension = 700;
yDimension = 350;
zDimension = 200;

% Each dimension consists of 1 points per unit length (grid resolution)
pointsPerDimensionLength = 1;

xVector = linspace(-xDimension/2,xDimension/2,xDimension*pointsPerDimensionLength);
yVector = linspace(0,yDimension,yDimension*pointsPerDimensionLength);
zVector = linspace(0,zDimension,zDimension*pointsPerDimensionLength);

% Create dummy domain to try out the interpolation
[Domain.x, Domain.y, Domain.z] = meshgrid(xVector, yVector, zVector);

% universal probe length for all Lidars. This variable can be commented out
% if the user wants to choose different probe length for each Lidar. 
dummyprobelength = 20;

%% Focal point selection

% y-coordinate of focal points
yPointsMat1 = 51:10:250;
yPointsMat = repmat(yPointsMat1,1,length(yPointsMat1));
yPointVec = yPointsMat(:);

% x-coordinate of focal points
xPointVec = yPointVec;

% z-coordinate of focal points
zPointMat = linspace(10,170,length(yPointsMat1));
zPointVec = repelem(zPointMat,length(yPointsMat1))';

%% Wind field

umag = meanWindSpeed/meanWindSpeed;
vmag = 5/meanWindSpeed;
wmag = 0.0;

uComp = umag.*ones(size(Domain.x));
vComp = vmag.*ones(size(Domain.x));
wComp = wmag.*ones(size(Domain.x));


%% Position of 1st LIDAR

% Give the lidar position (we assume that this point is the origin of the
% beam). The values should be in METRES. Put the lidar in the middle of the
% x-dimension
Lidar(1).x = 100;
Lidar(1).y = 0;
Lidar(1).z = 0;

%% Information about the probe of the 1st LIDAR

% Gap between the Lidar and the first range gate. Number in METRES
probe(1).FirstGap = 40;

% probe length in METRES
probe(1).Length = dummyprobelength;

% points per unit length for the probe
probe(1).PointsPerLength = 1;

%% Information about additional LIDARs 

if (nLidars > 1)
    
    % position of second LIDAR in cartesian coordinates
    Lidar(2).x = 0;
    Lidar(2).y = 100;
    Lidar(2).z = 0; 

    % FirstGap, PointsPerLength and NRgates will remain the same.
    probe(2).FirstGap = probe(1).FirstGap;
    probe(2).PointsPerLength = probe(1).PointsPerLength;

    % change the probe length if needed 
    probe(2).Length = dummyprobelength;
    
%     % RangeGateGap is again twice the Length
%     probe(2).RangeGateGap = 2*probe(2).Length;
    
    % check if there is a 3rd LIDAR and assign the ncessary values
    if nLidars == 3
        % position of 3rd LIDAR in cartesian coordinates
        Lidar(3).x = -60;
        Lidar(3).y = yDimension;
        Lidar(3).z = 0; 

        % FirstGap, PointsPerLength and NRgates will remain the same.
        probe(3).FirstGap = probe(1).FirstGap;
        probe(3).PointsPerLength = probe(1).PointsPerLength;

        % change the probe length if needed 
        probe(3).Length = dummyprobelength;

%         % RangeGateGap is again twice the Length
%         probe(3).RangeGateGap = 2*probe(3).Length;  
    end
end

%% 
if (operationMode == 'p')
    
    focalPoint = [0 200 30];
    % Distance between range gates (should be at least equal to the
    % probeLength). Now I put it twice the probeLength. It is subject to change
    % in the future.
    probe(1).RangeGateGap = 2*probe(1).Length;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This part was used when the input was radial distance and elevation
% angle. After the update, the input is a focal point in cartesian
% coordinates so this part is not used anymore

    % Radial distance to the point where we want to measure. The distance
    % should be larger that FirstGap + Length/2
%     probe(1).RadialDist2MeasurePoint = 57;

    % Inclination angle theta. The value must be in DEGREES. This angle also
    % corresponds to the point we want to measure.
    % WARNING: If 2 or less LIDARs are used for the measurement, the
    % inclination angle should be as small as possible to ensure small
    % contribution of the w-component in the reconstructed velocity.
    thetaLidar = nan(1,nLidars);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % azimuthial angle phi. The value must be in DEGREES. The lidar will scan
    % from -phi to +phi.
    phiLidar = 25;
    phiStep = 5;

    % number of range gates we want to measure.
    % NOTE: The code will return 3 range gates per Lidar UNLESS the
    % measurement point is too far from the Lidar location. If this is true
    % and more range gates fit before the one that actually contain the
    % focal point, then more range gates will be returned. If that is the
    % case then the focal point is always included in the last range gate.
    probe(1).NRangeGates = 3;
    
    % include the number of range gates in the probe(2) structure as well.
    if (nLidars > 1)
        probe(nLidars).NRangeGates = probe(1).NRangeGates;
        probe(nLidars-1).NRangeGates = probe(1).NRangeGates;
        
        % RangeGateGap is again twice the Length
        probe(nLidars).RangeGateGap = 2*probe(nLidars).Length;
        probe(nLidars-1).RangeGateGap = 2*probe(nLidars-1).Length;

    end;
    
    % call 'PPImode' script
    PPImode
else
    % matrix with the input measurement points. The matrix is [n x 3]. 
    % Each row represents one point and each column represents the x,y,z 
    % coordinate of that point.    
    CartInputPoints = [xPointVec yPointVec zPointVec];

    % call 'StaringMode' script
    StaringMode
end     

%% Quiver plot
%
% The following plot creates a field of vectors representing the wind
% magnitude and direction.
% NOTE: The quiver plot needs a lot of memory so it is recommended to
% use only with a very small grid just for visualization of the wind field.
% 
% figure
% quiver3(Domain.x,Domain.y,Domain.z,uComp,vComp,wComp,0.2)
% axis([-1 6 4 7 0 zDimension])
% view([70 50])
% xlabel('x dimension')
% ylabel('y dimension')
% zlabel('Height')

%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%% End of User Input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% DONT CHANGE ANYTHING BELOW THIS LINE %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% call printLidarInfo script to display all the user choices
printLidarInfo


%% END of script


