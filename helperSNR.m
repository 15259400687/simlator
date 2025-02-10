function [snr] = helperSNR(rdfft_pow,detidx,clusterIDs,noisepwr)
%HELPERSNR 此处显示有关此函数的摘要
%   此处显示详细说明

    uids = unique(clusterIDs,'stable');
    pr = NaN(numel(uids),1);
    snr = NaN(numel(uids),1);
    for m = 1:numel(uids)
        iClstr = find(clusterIDs==uids(m));
        iDet = unique(detidx(iClstr,1));
        idxDetLinear = sub2ind(size(rdfft_pow),detidx(iClstr,1),detidx(iClstr,2));
        [~,iMax] = max(rdfft_pow(iDet,:),[],'all');
        iMaxcol = ceil(iMax/numel(iDet));
        iMaxrow = mod(iMax,numel(iDet));
        if iMaxrow == 0
            iMaxrow = numel(iDet);
        end
        
        [pr(m),~] = max(rdfft_pow(idxDetLinear));
    
        snr(m) = pr(m) ./ noisepwr(find(detidx==iDet(iMaxrow),1));
    end

end

