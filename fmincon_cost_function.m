function [final_cost] = fmincon_cost_function(X_flat, params)
    ModelParams=bycicle_params();
    Horizon = params.Horizon;
    X = reshape(X_flat, [], Horizon)';
    X_x = X(:, 1:ModelParams.nx);
    X_u = X(:, ModelParams.nx+1: ModelParams.nx+ModelParams.nu);
    
    
    final_cost = 0;
    track = params.track;
    trackWidth = params.trackWidth;
    gamma = 0.5;
    last_idx = 0;
    for i=1:Horizon
       state_i = X_x(i,:)';
       input_i = X_u(i,:)';
       x_i = state_i(ModelParams.stateindex_x);
       y_i = state_i(ModelParams.stateindex_y);
       
       
       p = round(([x_i; y_i]-[track.xmin;track.ymin])/track.step); % get rows and cols for current position on distance mat
       [rows, cols] = size(track.D); % get num rows and cols of distance matrix
       p = max(p,1);
       p = min(p,[cols;rows-1]);
       idx = track.idx(rows-p(2), p(1)); % get index of [xc, yc] in track.center_rounded
       if(last_idx == 0)
           last_idx = idx;
       end
       xc_i = track.center_rounded(1,idx);
       yc_i = track.center_rounded(2,idx);
       if idx == length(track.center)
           phi_i = atan2(track.center_rounded(2,1)- yc_i, track.center_rounded(1,1)- xc_i);
       else
           phi_i = atan2(track.center_rounded(2,idx+1)- yc_i, track.center_rounded(1,idx+1)- xc_i);
       end
       
       el_i = -cos(phi_i)*(x_i - xc_i) - sin(phi_i)*(y_i-yc_i);
       ec_i = sin(phi_i)*(x_i - xc_i) - cos(phi_i)*(y_i-yc_i);

       progress = track.progress(last_idx, idx);

       final_cost = final_cost + 5*ec_i^2 + 5*el_i^2;% -progress; % mancalreg
       last_idx = idx;
    end
end