function [Xbmfrngdop] = helperVirtualBeamform(txSteer,vxSteer,Xrngdop,bmfWinFcn)
% This function is for demo purposes only and may be removed in the future

%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.

% Implements receive beamforming for the virtual array in the transmit
% beamforming (TXBF) mode where each scan angle is electronically scanned
% on transmit and then beamformed at the same angle on receive.

hasWinFcn = nargin>3 && ~isempty(bmfWinFcn);
if ~hasWinFcn
    bmfWinFcn = @rectwin;
end

Nr = size(Xrngdop,1); % Number of range bins
Nd = size(Xrngdop,3); % Number of Doppler bins



% Apply spatial taper
Ne = size(vxSteer,1); % Number of non-overlapping virtual array elements
if iscell(bmfWinFcn)
    win = feval(bmfWinFcn{1},Ne,bmfWinFcn{2:end});
else
    win = feval(bmfWinFcn,Ne);
end
vxSteer = bsxfun(@times,win,vxSteer);

% Allocate beamform datacube
Nb = size(vxSteer,2); % Number of scan angles
Xbmfrngdop = zeros(Nr,Nb,Nd,'like',Xrngdop);

% Beamform for each transmit and receive scan angle
if length(size(Xrngdop)) > 3
    for iBm = 1:Nb
        % Virtual array steering vector
        vxSteer0 = vxSteer(:,iBm);
    
        % Create vitual array elements
        Xrx = Xrngdop(:,:,:,iBm); % Nr x Ne x Nd
        Xrx = shiftdim(Xrx,1); % Ne x Nd x Nr
        Xrx = Xrx(:,:); % Ne x (Nd x Nr)
        Xvx = helperVirtualSteeringVector(txSteer(:,iBm),Xrx);
    
        % Beamform
        vxBeam = vxSteer0'*Xvx;
    
        % Save beamformed data
        vxBeam = reshape(vxBeam,Nd,Nr); % Nd x Nr
        vxBeam = shiftdim(vxBeam,1); % Nr x Nd
        Xbmfrngdop(:,iBm,:) = vxBeam;
    end
else
    Xrx = Xrngdop; % Nr x Ne x Nd
    Xrx = shiftdim(Xrx,1); % Ne x Nd x Nr
    Xrx = Xrx(:,:); % Ne x (Nd x Nr)
    for iBm = 1:Nb
        % Virtual array steering vector
        vxSteer0 = vxSteer(:,iBm);
    
        % Create vitual array elements

        Xvx = helperVirtualSteeringVector(txSteer(:,iBm),Xrx);
    
        % Beamform
        vxBeam = vxSteer0'*Xvx;
    
        % Save beamformed data
        vxBeam = reshape(vxBeam,Nd,Nr); % Nd x Nr
        vxBeam = shiftdim(vxBeam,1); % Nr x Nd
        Xbmfrngdop(:,iBm,:) = vxBeam;
    end
end


end
