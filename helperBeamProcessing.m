function [Xbmfrngdop] = helperBeamProcessing(Xrngdop,rxSteer,txSteer,varargin)
% This function is for demo purposes only and may be removed in the future

%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.

if isa(rxSteer,'radarTransceiver')
    % Use phased.SteeringVector to compute steering vectors for the receive
    % array for each scan angle
    xcvrLLR = rxSteer;
    azgrid = txSteer;

    txSV = phased.SteeringVector( ...
    'SensorArray',xcvrLLR.TransmitAntenna.Sensor, ...
    'PropagationSpeed',xcvrLLR.TransmitAntenna.PropagationSpeed);
    txSteer = txSV(xcvrLLR.TransmitAntenna.OperatingFrequency,azgrid);

    rxSV = phased.SteeringVector( ...
        'SensorArray',xcvrLLR.ReceiveAntenna.Sensor, ...
        'PropagationSpeed',xcvrLLR.ReceiveAntenna.PropagationSpeed);
    rxSteer = rxSV(xcvrLLR.ReceiveAntenna.OperatingFrequency,azgrid);
end
[Xbmfrngdop]= helperVirtualBeamform(txSteer,rxSteer,Xrngdop,varargin{:});
end
