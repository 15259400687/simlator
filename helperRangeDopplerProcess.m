function [Xrngdop,rggrid,rrgrid] = helperRangeDopplerProcess(rngdopproc,Xcube,transceiver,rngWinFcn)
% This function is for demo purposes only and may be removed in the future

%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.

hasRngWinFcn = nargin>3;
Ns = size(Xcube,1); % Number of samples

isDechirp = ~ismethod(transceiver.Waveform,'getMatchedFilter');
if isDechirp
    % Dechirp data cubes for CW waveforms
    refsig = transceiver.Waveform();
    refsig = refsig(:);
    %Xcube = helperDechirpCube(Xcube,refsig(:));
end

isCube = ~ismatrix(Xcube);
if isCube
    Nc = size(Xcube,2);
    % Leave any extra dimensions untouched
    if length(size(Xcube)) > 3
        xtraDims = ones(1,max(ndims(Xcube)-3,0));
        for iXtra = 1:numel(xtraDims)
            xtraDims(iXtra) = size(Xcube,iXtra+3);
        end
    end
else
    Nc = 1;
end

% Apply range window
if hasRngWinFcn
    if iscell(rngWinFcn)
        win = feval(rngWinFcn{1},Ns,rngWinFcn{2:end});
    else
        win = feval(rngWinFcn,Ns);
    end
end

% Apply range and Doppler processing
isFFT = strcmp(rngdopproc.RangeMethod,'FFT');
if isFFT
    args = {};
else
    mfcoeff = getMatchedFilter(transceiver.Waveform);
    args = {mfcoeff};
end

if isCube
    if length(size(Xcube)) > 3
        Xvals = Xcube(:,1,:,1);
    else
        Xvals = Xcube(:,1,:);
    end
    
    if isDechirp
        Xvals(:) = dechirp(Xvals(:,:),refsig);
    end
    if hasRngWinFcn
        Xvals(:,:) = win .* Xvals(:,:);
    end
    [Yrngdop,rggrid,rrgrid] = rngdopproc(Xvals(:,:),args{:});

    % Remove negative range bins
    % isGdRg = rggrid>=0;
    % isGdRg = rggrid>=min(rggrid);
    % rggrid = rggrid(isGdRg);

    Nd = numel(rrgrid);
    Nr = numel(rggrid);
    if length(size(Xcube)) > 3
        Xrngdop = zeros([Nr Nc Nd xtraDims],'like',Yrngdop);
    else
        Xrngdop = zeros([Nr Nc Nd],'like',Yrngdop);
    end
    
    for iCh = 1:Nc
        if length(size(Xcube)) > 3
            for iXtra = 1:prod(xtraDims)
                Xvals(:) = Xcube(:,iCh,:,iXtra);
                if isDechirp
                    Xvals(:) = dechirp(Xvals(:,:),refsig);
                end
                if hasRngWinFcn
                    Xvals(:,:) = win .* Xvals(:,:);
                end
                Yrngdop(:) = rngdopproc(Xvals(:,:),args{:});
                % Xrngdop(:,iCh,:,iXtra) = Yrngdop(isGdRg,:);
                Xrngdop(:,iCh,:,iXtra) = Yrngdop;
            end
        else
            Xvals(:) = Xcube(:,iCh,:);
            if isDechirp
                Xvals(:) = dechirp(Xvals(:,:),refsig);
            end
            if hasRngWinFcn
                Xvals(:,:) = win .* Xvals(:,:);
            end
            Yrngdop(:) = rngdopproc(Xvals(:,:),args{:});
            % Xrngdop(:,iCh,:) = Yrngdop(isGdRg,:);
            Xrngdop(:,iCh,:) = Yrngdop;
        end
    end
else
    Xvals = Xcube(:,:);
    if isDechirp
        Xvals(:) = dechirp(Xvals(:,:),refsig);
    end
    if hasRngWinFcn
        Xvals(:,:) = win .* Xvals(:,:);
    end
    [Yrngdop,rggrid,rrgrid] = rngdopproc(Xvals(:,:),args{:});

    % Remove negative range bins
    % isGdRg = rggrid>=0;
    % isGdRg = rggrid>=min(rggrid);
    % rggrid = rggrid(isGdRg);
    % 
    % Xrngdop = Yrngdop(isGdRg,:);
    Xrngdop = Yrngdop;
end

if isDechirp
    Xrngdop = conj(Xrngdop);
end
end
