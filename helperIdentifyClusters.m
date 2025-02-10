function [clusterIDs,detidx,numClusters,clusterer] = helperIdentifyClusters(XsnrdB,iDet,azgrid,rggrid,rrgrid)
% This function is for demo purposes only and may be removed in the future

%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.

% Compute the range, angle, and Doppler indices of the detection crossings
[iRg,iAng,iRr] = ind2sub(size(XsnrdB),iDet);
detidx = [iRg iAng iRr];

% Select the range, angle, and Doppler bin centers associated with each
% detection crossing
meas = [azgrid(iAng)' rggrid(iRg) rrgrid(iRr)];

% Estimate the epsilon to use for DBSCAN clustering.
minNumPoints = 2;
maxNumPoints = 10;
epsilon = clusterDBSCAN.estimateEpsilon(meas,minNumPoints,maxNumPoints);
% Range and Doppler ambiguity limits
ambLims = [ ...
    [min(rggrid) max(rggrid)]; ...
    [min(rrgrid) max(rrgrid)]];
clusterer = clusterDBSCAN( ...
    'MinNumPoints',2, ...
    'EnableDisambiguation',true, ...
    'AmbiguousDimension',[2 3], ...
    'Epsilon',epsilon);

% Identify the detections that should be clustered into a single detection.
% The threshold crossings are assigned IDs. Crossings that appear to come
% from the same target will be assigned the same ID.
[~,clusterIDs] = clusterer(meas,ambLims);
clusterIDs = clusterIDs(:)';

% Number of unique targets estimated after clustering the threshold crossings.
if isempty(clusterIDs)
    numClusters = 0;
else
    numClusters = max(clusterIDs);
end
end