function [paths,svv,arrayEffect] = helperLandSnowPaths(tgt_pos_rel,tgt_rng_rel,tgt_ang_rel,tgt_vel_rel,lambda,rcsgain,tgt_rng_max,fc,...
    r_land,angle_land,rcs_land,R_max_land,rcsrate_snow_pow,R_max_snow,snowrate,snow_az,r_res,txArray,rxArray)
    

% This function is for demo purposes only and may be removed in the future

%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.


% ------------------------------------------------------------------------
% 
    Ntargets = size(tgt_rng_rel,2);
    

    % path loss
    tgtploss = fspl(tgt_rng_rel,lambda);
    tgtsnowloss = tgt_rng_rel / 1000 * 1.7 * (snowrate ^ 0.46);
    
    % doppler
    dop = speed2dop(radialspeed(tgt_pos_rel, tgt_vel_rel),lambda);

% ------------------------------------------------------------------------
% backscatter of snow
% rcs_signature_snow
    rcs_snow  = [];
    path_snow = [];
    if R_max_snow >= r_res
        snow_range_rel = tgt_rng_rel(abs(tgt_ang_rel(1,:)) <= snow_az);
        Ntargets_snow = size(snow_range_rel,2);
        if Ntargets_snow == 0
            for i = 1:ceil(R_max_snow / r_res)-1
                r_snow = i * r_res;
                mean_snow_rcs_db = pow2db(rcsrate_snow_pow * r_snow ^ 2);
                rcs_signature_snow = rcsSignature("Pattern",mean_snow_rcs_db,...
                    "Frequency",[fc fc],"FluctuationModel","Swerling1");
                rcs_snow = [rcs_snow rcs_signature_snow];
                path_snow = [path_snow i * r_res];
            end
        else 
            if max(snow_range_rel) < R_max_snow
                for i = 1:ceil(max(snow_range_rel) / r_res)-1
                    r_snow = i * r_res;
                    mean_snow_rcs_db = pow2db(rcsrate_snow_pow * r_snow ^ 2);
                    rcs_signature_snow = rcsSignature("Pattern",mean_snow_rcs_db,...
                        "Frequency",[fc fc],"FluctuationModel","Swerling1");
                    rcs_snow = [rcs_snow rcs_signature_snow];
                    path_snow = [path_snow i * r_res];
                end
            else
                for i = 1:ceil(R_max_snow / r_res)-1
                    r_snow = i * r_res;
                    mean_snow_rcs_db = pow2db(rcsrate_snow_pow * r_snow ^ 2);
                    rcs_signature_snow = rcsSignature("Pattern",mean_snow_rcs_db,...
                        "Frequency",[fc fc],"FluctuationModel","Swerling1");
                    rcs_snow = [rcs_snow rcs_signature_snow];
                    path_snow = [path_snow i * r_res];
                end
            end
        end
    end
    
% ------------------------------------------------------------------------
    
    Nsnow = size(rcs_snow,2);
    paths = repmat(struct(...
        'PathLength', zeros(1, 1), ...
        'PathLoss', zeros(1, 1), ...
        'ReflectionCoefficient', zeros(1,1), ...
        'AngleOfDeparture', zeros(2, 1), ...
        'AngleOfArrival', zeros(2, 1), ...
        'DopplerShift', zeros(1, 1)),...
        1,Ntargets);
        
    pathindex = 0;
    for m = 1:Ntargets
        
        % two way

        if tgt_rng_rel(:,m) > tgt_rng_max(:,m)
            paths(pathindex+1) = [];
            continue;
        else
            plength = 2 * tgt_rng_rel(:,m);
            pathindex = pathindex + 1;
        end


        ploss = 2 * tgtploss(:,m);
        psnowloss = 2 * tgtsnowloss(m,:);
        pdop = 2 * dop(m,:);
        
        paths(m).PathLength = plength;
        paths(m).PathLoss = ploss + psnowloss;
%         paths(m).PathLoss = ploss;
        paths(m).ReflectionCoefficient = db2mag(rcsgain(m,:));
        paths(m).AngleOfDeparture = tgt_ang_rel(:,m);
        paths(m).AngleOfArrival = tgt_ang_rel(:,m);
        paths(m).DopplerShift = pdop;
    end



    Npaths = size(paths,2);

    % path loss
    landploss = fspl(r_land,lambda);
    landsnowploss = r_land / 1000 * 1.7 * (snowrate ^ 0.46);
    
    % doppler
    dop = 0;


    if r_land <= R_max_land

        plength = 2 * r_land;
    

        ploss = 2 * landploss;
        psnowloss = 2 * landsnowploss;
        pdop = 2 * dop;
    
    
        paths(Npaths+1).PathLength = plength;
        paths(Npaths+1).PathLoss = ploss + psnowloss;
        paths(Npaths+1).ReflectionCoefficient = db2mag(rcs_land);
        paths(Npaths+1).AngleOfDeparture = angle_land;
        paths(Npaths+1).AngleOfArrival = angle_land;
        paths(Npaths+1).DopplerShift = pdop;
    end




    Npaths = size(paths,2);
    if Nsnow ~= 0

        snowploss = fspl(path_snow,lambda);
        snowloss = snowpl(path_snow,fc,snowrate);
        snowdop = 0;
    
        for m = 1:Nsnow
    
            rcs = db2pow(value(rcs_snow(m),0,0,fc));
            rgain = aperture2gain(rcs,lambda);
    
    
            % two way
            
            plength = 2 * path_snow(:,m);
            ploss = 2 * snowploss(:,m);
            psnowloss = 2 * snowloss(m,:);
            pdop = snowdop;
            
            paths(Npaths+m).PathLength = plength;
            paths(Npaths+m).PathLoss = ploss + psnowloss;
            paths(Npaths+m).ReflectionCoefficient = db2mag(rgain);
            paths(Npaths+m).AngleOfDeparture = [0;0];
            paths(Npaths+m).AngleOfArrival = [0;0];
            paths(Npaths+m).DopplerShift = pdop;
        end
    end


    sv_tgt = bsxfun(@times,1./sqrt(1),step(txArray,fc,tgt_ang_rel));
    arrayEffect_tgt = step(rxArray,fc,tgt_ang_rel);

    if Nsnow ~= 0
        snow_ang = zeros(2,Nsnow);
        sv_snow = bsxfun(@times,1./sqrt(1),step(txArray,fc,snow_ang));
        arrayEffect_snow = step(rxArray,fc,snow_ang);
    else
        sv_snow = [];
        arrayEffect_snow = [];
    end
    
    sv = [sv_tgt sv_snow];
    svv = sum(sv);
    arrayEffect = [arrayEffect_tgt arrayEffect_snow];


end
