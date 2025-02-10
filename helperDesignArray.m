function [txArray, rxArray, arrayMetrics, txPos, rxPos, ant] = helperDesignArray(lambda,Fc)
% This function is for demo purposes only and may be removed in the future

%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.

% Each array element is modeled as a 4-element series-fed patch antenna
% with a field of view of 85 degrees in azimuth and 25 degrees in elevation.
antFOV = [85 25]; % degrees
k = log(1/2)./(2*log(cosd(antFOV/2))); % See documentation for phased.CosineAntennaElement 
ant = phased.CosineAntennaElement('CosinePower',k(:)');

% Confirm that the directivity of the phased.CosineAntennaElement matches
% the directivity of 12 dBi reported in Table 1 of [3]
Gdbi = pattern(ant,Fc,0,0);

% Use phased.ConformalArray to create the linear receive array consisting
% of 16 elements and the 12 element transmit array with each array element
% modeled as a 4-element series-fed patch antenna.
rxPos = zeros(54,1);
rxPos([1:4 12:15 47:50 51:54],1) = 1;
[row,col] = ind2sub(size(rxPos),find(rxPos(:)));
rxArray = phased.ConformalArray('Element',ant,'ElementPosition',[zeros(numel(row),1) row-1 col-1]'*lambda/2);

% Confirm that the receive array gain matches the gain of 24 dBi modeled in
% the Radar Designer app.
Grx = pattern(rxArray,Fc,0,0);

% Compute the azimuth 3-dB beamwidth for the receive array. The 3-dB
% beamwidth should match the desired azimuth resolution of 1.4 degrees.
rxAzBw = beamwidth(rxArray,Fc,'Cut','Azimuth');

% Create the transmit array designed in [1] consisting of 12 array elements
% with each array element modeled as a 4-element series-fed patch antenna.
txPos = zeros(33,7);
txPos(1:4:end,1) = 1;
txPos(10,2) = 1;
txPos(11,5) = 1;
txPos(12,7) = 1;

[row,col] = ind2sub(size(txPos),find(txPos(:)));
txArray = phased.ConformalArray('Element',ant,'ElementPosition',[zeros(numel(row),1) row-1 col-1]'*lambda/2);

% Confirm that the transmit array gain matches the gain of 23 dBi modeled in the Radar Designer app.
Gtx = pattern(txArray,Fc,0,0);

% The transmit array beamwidth is larger than the receive array beamwidth.
% This is useful since the look directions will be spaced according to the
% radar's azimuth resolution. The larger transmit beamwidth ensures that
% all targets within a receive beamwidth will be illuminated by the
% transmit beam.
txAzBw = beamwidth(txArray,Fc,'Cut','Azimuth');

arrayMetrics.ElementDirectivity = Gdbi;
arrayMetrics.RxGain = Grx;
arrayMetrics.RxBeamwidth = rxAzBw;
arrayMetrics.TxGain = Gtx;
arrayMetrics.TxBeamwidth = txAzBw;
end
