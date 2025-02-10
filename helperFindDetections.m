function [iDet,estnoiselvldB,threshdB] = helperFindDetections(Xbmfrngdop,Xrngdop,xcvrLLR,Pd,Pf)
% This function is for demo purposes only and may be removed in the future

%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.

% Compare the SNR of the range, Doppler, and beamformed data cube to a
% detection threshold. Select the detection threshold to match the desired
% detection probability.
threshdB = detectability(Pd,Pf);

numFT = xcvrLLR.Waveform.SampleRate*xcvrLLR.Waveform.SweepTime;
numST = xcvrLLR.NumRepetitions;
numRxElmnts = getNumElements(xcvrLLR.ReceiveAntenna.Sensor);

% Estimate the noise floor of the processed data by collecting noise only data
noSignal = zeros(numFT,numRxElmnts,numST);
Ncube = xcvrLLR.Receiver(noSignal(:));
Ncube = reshape(Ncube,numFT,numRxElmnts,numST);

% Apply the same range, Doppler, and beamform processing to the noise data
% as was applied to the detection data.
Nrngdop = helperRangeDopplerProcessing(Ncube,xcvrLLR);
Nbmfrngdop = helperBeamProcessing(Nrngdop,xcvrLLR,0);

% Estimate noise level in dB
estnoiselvldB = pow2db(mean(abs(Nbmfrngdop(:)).^2));

% Compute the SNR of the processed radar data cube.
Xpow = abs(Xbmfrngdop).^2; % Power is amplitude squared
XsnrdB = pow2db(Xpow)-estnoiselvldB; % Normalize using the estimated noise floor power level to get SNR

isDet = XsnrdB(:)>threshdB;

% Many of the threshold crossings correspond to the sidelobes of the
% virtual array. Suppression of the sidelobes using a spatial taper alone
% is often not enough to avoid these spurious detections. Identify sidelobe
% detections by comparing the SNR of the detection locations for a single
% element to the SNR in the beams.

% Estimate sidelobe level from sidelobe detections
estnoiseslldB = pow2db(mean(abs(Nrngdop(:)).^2));
XslldB = squeeze(max(pow2db(abs(Xrngdop).^2),[],2)); % Max across receive elements
XslldB = XslldB-estnoiseslldB; % Convert to SNR
XslldB = permute(XslldB,[1 3 2]); % range x azimuth x Doppler
isLob = isDet & XsnrdB(:)<XslldB(:);

% Adjust the detection threshold according to the max sidelobe level to
% further suppress sidelobe detections.
maxSLLdB = max(XslldB(isLob)-XsnrdB(isLob));
threshdB = threshdB+maxSLLdB;
iDet = find(XsnrdB(:)>threshdB & ~isLob);
end
