function [rot,off,frm] = helperSensorFrame(sensor)
% This function is for demo purposes only and may be removed in the future

%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.

rot = matlabshared.tracking.internal.fusion.rotZYX(sensor.MountingAngles);
off = sensor.MountingLocation(:);

frm = matlabshared.tracking.internal.fusion.objectFrame();
frm.OriginPosition(:) = off;
frm.Orientation(:,:) = rot;
frm.IsParentToChild(:) = false;
end
