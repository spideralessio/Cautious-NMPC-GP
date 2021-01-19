function [final_cost] = cost_function(X,U,e,data,params)
% ModelParams.stateindex_x=1; %x position
% ModelParams.stateindex_y=2; %y position
% ModelParams.stateindex_phi=3; %orientation
% ModelParams.stateindex_vx=4; %longitudinal velocity
% ModelParams.stateindex_vy=5; %lateral velocity
% ModelParams.stateindex_omega=6; %yaw rate
% ModelParams.stateindex_theta=7; %virtual position
    ModelParams=bycicle_params();
    N = length(X);
    final_cost = 0;
    track = params.track;
    trackWidth = params.trackWidth;
    gamma = 0.5;
    
    for i=1:N
       state_i = X(i,:)';
       input_i = U(i,:)';
       final_cost = final_cost + input_i(1);
       x_i = state_i(ModelParams.stateindex_x);
       y_i = state_i(ModelParams.stateindex_y);
       
       
       p = round(([x_i; y_i]-[track.xmin;track.ymin])/track.step); % get rows and cols for current position on distance mat
       [rows, cols] = size(track.D); % get num rows and cols of distance matrix
       idx = track.idx(rows-p(2), p(1)); % get index of [xc, yc] in track.center_rounded
       xc_i = track.center_rounded(1,idx);
       yc_i = track.center_rounded(2,idx);
       if idx == length(track.center)
           phi_i = atan2(track.center_rounded(2,1)- yc_i, track.center_rounded(1,1)- xc_i);
       else
           phi_i = atan2(track.center_rounded(2,idx+1)- yc_i, track.center_rounded(1,idx+1)- xc_i);
       end
       
       if i>1
           el_i = -cos(phi_i)*(x_i - xc_i) - sin(phi_i)*(y_i-yc_i);
           ec_i = sin(phi_i)*(x_i - xc_i) - cos(phi_i)*(y_i-yc_i);

           lreg_i = norm(input_i - U(i-1,:)')^2;
           
           progress = track.progress(last_idx, idx);
           if idx > length(track.center)
                progress = -progress
                idx = mod(idx, track.points)
           end
           final_cost = final_cost + el_i^2 + ec_i^2 - gamma*progress + lreg_i;
       end
       last_idx = idx;
    end
end

