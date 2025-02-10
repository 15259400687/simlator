function [meas,noise,snrdB,detRngAng] = helperMeasurementEstimation(Xpow,estnoiselvlpwr,xcvrLLR,azgrid,rggrid,rrgrid,clusterIDs,detidx,rgestimator,rrestimator)
% This function is for demo purposes only and may be removed in the future

%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.

if ~isreal(Xpow)
    % Convert I/Q signal to power
    Xpow = abs(Xpow).^2;
end


% swpBandwidth = min(xcvrLLR.Waveform.SweepBandwidth,xcvrLLR.Receiver.SampleRate);
% rgres = bw2rangeres(swpBandwidth,xcvrLLR.ReceiveAntenna.PropagationSpeed);
% rgestimator = phased.RangeEstimator(...
%     'ClusterInputPort',true, ...
%     'VarianceOutputPort',true,...
%     'NoisePowerSource','Input port', ...
%     'RMSResolution',rgres);
[rgest,rgvar] = rgestimator(squeeze(max(Xpow,[],2)),rggrid,detidx,estnoiselvlpwr,clusterIDs);

% rrestimator = phased.DopplerEstimator(...
%     'ClusterInputPort',true, ...
%     'VarianceOutputPort',true, ...
%     'NoisePowerSource','Input port', ...
%     'NumPulses',xcvrLLR.NumRepetitions);
[rrest,rrvar] = rrestimator(squeeze(max(Xpow,[],2)),rrgrid,detidx,estnoiselvlpwr,clusterIDs);

angres = beamwidth(xcvrLLR.ReceiveAntenna.Sensor, ...
    xcvrLLR.ReceiveAntenna.OperatingFrequency, ...
    'PropagationSpeed',xcvrLLR.ReceiveAntenna.PropagationSpeed);
[azest,azvar,snr,detRngAng] = helperAzimuthEstimate(Xpow,azgrid,detidx,clusterIDs, ...
    angres,estnoiselvlpwr);

snrdB = pow2db(snr);

tgtIndex = find(rgest>0);

meas = [azest(tgtIndex) rgest(tgtIndex ) rrest(tgtIndex ) ]';
noise = [azvar(tgtIndex ) rgvar(tgtIndex ) rrvar(tgtIndex )]';


end
