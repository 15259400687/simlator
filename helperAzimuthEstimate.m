function [azest,azvar,snr,detRngAng] = helperAzimuthEstimate(Xpow,anggrid,detidx,clusterIDs,angres,noisepwr)
% This function is for demo purposes only and may be removed in the future

%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.
detRngAng = detidx;
uids = unique(clusterIDs);
azest = NaN(numel(uids),1,'like',anggrid);
snr = NaN(numel(uids),1,'like',anggrid);
hasLessThan3 = false(numel(uids),1);

[n1,n2,n3] = size(Xpow);

for m = 1:numel(uids)
    iClstr = find(clusterIDs==uids(m));
    a = permute(Xpow,[1 3 2]);
    b = a(detidx(1,iClstr),detidx(2,iClstr),:);
    [~,iMaxChannel] = max(b,[],3);
    detidx(3,iClstr) = diag(iMaxChannel);
    iDet = sub2ind([n1 n2 n3],detidx(1,iClstr), detidx(3,iClstr), detidx(2,iClstr));

    [mVal,iMax] = max(Xpow(iDet));
    
    snr(m) = mVal;
    
    % Select adjacent beams
    
    iBm0 = detidx(3,iClstr(iMax));
    iBm = iBm0+(-1:1);
    iBm = iBm(iBm>=1 & iBm<=numel(anggrid));
    if numel(iBm)>2
        % Perform beamsplit to estimate angle
        i1 = detidx(1,iClstr(iMax));
        i3 = detidx(2,iClstr(iMax));
        vals = log(Xpow);
        vals = vals(:,:,i3);
        vals = vals(i1,:);
        vals = vals(:,iBm,:);
        vals = vals-mean(vals);
        coeff = polyfit(anggrid(iBm),vals,2);
        azest(m) = -coeff(2)/(2*coeff(1));
    else
        azest(m) = anggrid(iBm0);
        hasLessThan3(m) = true;
    end

    detRngAng(2,iClstr) = iBm(randi(numel(iBm),size(iClstr)));
end
bw = abs(diff(anggrid(1:2)));
azest = min(azest,max(anggrid)+bw/2);
azest = max(azest,min(anggrid)-bw/2);

snr = snr/noisepwr(iClstr(iMax));

% Equation 6.37 in Skolnik, Merrill I. Introduction to Radar Systems. 2002.
k = 0.628;
fn = k./sqrt(2*snr);
fn(hasLessThan3) = 1/sqrt(12);
fn = min(fn,1/sqrt(12)); % Limit worst-case to uniformly distributed across resolution
azvar = (angres*fn).^2;
end
