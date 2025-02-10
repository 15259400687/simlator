function [Xrngdop,rggrid,rrgrid] = helperRangeDopplerProcessing(Xframes,xcvrLLR,rngdopproc,varargin)
% This function is for demo purposes only and may be removed in the future

%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.

if nargin>3
    rngWinFcn = varargin{4};
end
if nargin<=3
    rngWinFcn = rangeDopplerProcessor();
end

[Xrngdop,rggrid,rrgrid] = helperRangeDopplerProcess(rngdopproc,Xframes,xcvrLLR,rngWinFcn);
end

function rngWinFcn = rangeDopplerProcessor()


rngWinFcn = @hanning;



% rngdopproc = phased.RangeDopplerResponse('RangeMethod','FFT', ...
%     'DopplerOutput','Speed','SweepSlope',xcvrLLR.Waveform.SweepBandwidth/xcvrLLR.Waveform.SweepTime);

% Configure Doppler properties

end
