function [paths,svv,arrayEffect] = helperLandRainPaths(tgt_pos_rel,tgt_rng_rel,tgt_ang_rel,tgt_vel_rel,lambda,rcsgain,tgt_rng_max,fc,...
    r_land,angle_land,rcs_land,R_max_land,rcsrate_rain_pow,R_max_rain,rainrate,rain_az,r_res,txArray,rxArray)
    

% This function is for demo purposes only and may be removed in the future

%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.


% ------------------------------------------------------------------------
% 
    Ntargets = size(tgt_rng_rel,2);
    

    % path loss
    tgtploss = fspl(tgt_rng_rel,lambda);
    tgtrainloss = rainpl(tgt_rng_rel,fc,rainrate);
    
    % doppler
    dop = speed2dop(radialspeed(tgt_pos_rel, tgt_vel_rel),lambda);

% ------------------------------------------------------------------------
% backscatter of rain
% rcs_signature_rain
    rcs_rain  = [];
    path_rain = [];
    if R_max_rain >= r_res
        rain_range_rel = tgt_rng_rel(abs(tgt_ang_rel(1,:)) <= rain_az);
        Ntargets_rain = size(rain_range_rel,2);
        if Ntargets_rain == 0
            for i = 1:ceil(R_max_rain / r_res)-1
                r_rain = i * r_res;
                mean_rain_rcs_db = pow2db(rcsrate_rain_pow * r_rain ^ 2);
                rcs_signature_rain = rcsSignature("Pattern",mean_rain_rcs_db,...
                    "Frequency",[fc fc],"FluctuationModel","Swerling1");
                rcs_rain = [rcs_rain rcs_signature_rain];
                path_rain = [path_rain i * r_res];
            end
        else 
            if max(rain_range_rel) < R_max_rain
                for i = 1:ceil(max(rain_range_rel) / r_res)-1
                    r_rain = i * r_res;
                    mean_rain_rcs_db = pow2db(rcsrate_rain_pow * r_rain ^ 2);
                    rcs_signature_rain = rcsSignature("Pattern",mean_rain_rcs_db,...
                        "Frequency",[fc fc],"FluctuationModel","Swerling1");
                    rcs_rain = [rcs_rain rcs_signature_rain];
                    path_rain = [path_rain i * r_res];
                end
            else
                for i = 1:ceil(R_max_rain / r_res)-1
                    r_rain = i * r_res;
                    mean_rain_rcs_db = pow2db(rcsrate_rain_pow * r_rain ^ 2);
                    rcs_signature_rain = rcsSignature("Pattern",mean_rain_rcs_db,...
                        "Frequency",[fc fc],"FluctuationModel","Swerling1");
                    rcs_rain = [rcs_rain rcs_signature_rain];
                    path_rain = [path_rain i * r_res];
                end
            end
        end
    end
    
% ------------------------------------------------------------------------
    
    Nrain = size(rcs_rain,2);
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
        prainloss = 2 * tgtrainloss(m,:);
        pdop = 2 * dop(m,:);
        
        paths(pathindex).PathLength = plength;
        paths(pathindex).PathLoss = ploss + prainloss;
%         paths(m).PathLoss = ploss;
        paths(pathindex).ReflectionCoefficient = db2mag(rcsgain(m,:));
        paths(pathindex).AngleOfDeparture = tgt_ang_rel(:,m);
        paths(pathindex).AngleOfArrival = tgt_ang_rel(:,m);
        paths(pathindex).DopplerShift = pdop;
    end



    Npaths = size(paths,2);

    % path loss
    landploss = fspl(r_land,lambda);
    landrainploss = rainpl(r_land,fc,rainrate);
    
    % doppler
    dop = 0;


    if r_land <= R_max_land

        plength = 2 * r_land;
    

        ploss = 2 * landploss;
        prainloss = 2 * landrainploss;
        pdop = 2 * dop;
    
    
        paths(Npaths+1).PathLength = plength;
        paths(Npaths+1).PathLoss = ploss + prainloss;
        paths(Npaths+1).ReflectionCoefficient = db2mag(rcs_land);
        paths(Npaths+1).AngleOfDeparture = angle_land;
        paths(Npaths+1).AngleOfArrival = angle_land;
        paths(Npaths+1).DopplerShift = pdop;
    end




    Npaths = size(paths,2);
    if Nrain ~= 0

        rainploss = fspl(path_rain,lambda);
        rainloss = rainpl(path_rain,fc,rainrate);
        raindop = 0;
    
        for m = 1:Nrain
    
            rcs = db2pow(value(rcs_rain(m),0,0,fc));
            rgain = aperture2gain(rcs,lambda);
    
    
            % two way
            
            plength = 2 * path_rain(:,m);
            ploss = 2 * rainploss(:,m);
            prainloss = 2 * rainloss(m,:);
            pdop = raindop;
            
            paths(Npaths+m).PathLength = plength;
            paths(Npaths+m).PathLoss = ploss + prainloss;
            paths(Npaths+m).ReflectionCoefficient = db2mag(rgain);
            paths(Npaths+m).AngleOfDeparture = [0;0];
            paths(Npaths+m).AngleOfArrival = [0;0];
            paths(Npaths+m).DopplerShift = pdop;
        end
    end


    sv_tgt = bsxfun(@times,1./sqrt(1),step(txArray,fc,tgt_ang_rel));
    arrayEffect_tgt = step(rxArray,fc,tgt_ang_rel);

    if Nrain ~= 0
        rain_ang = zeros(2,Nrain);
        sv_rain = bsxfun(@times,1./sqrt(1),step(txArray,fc,rain_ang));
        arrayEffect_rain = step(rxArray,fc,rain_ang);
    else
        sv_rain = [];
        arrayEffect_rain = [];
    end
    
    sv = [sv_tgt sv_rain];
    svv = sum(sv);
    arrayEffect = [arrayEffect_tgt arrayEffect_rain];


end
