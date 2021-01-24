function [final_cost, cost_gradient] = fmincon_cost_function(X_flat, params)
    ModelParams=bycicle_params();
    Horizon = params.Horizon;
    X = reshape(X_flat, [], Horizon)';
    X_x = X(:, 1:ModelParams.nx);
    X_u = X(:, ModelParams.nx+1: ModelParams.nx+ModelParams.nu);
    
    
    final_cost = 0;
    track = params.track;
    trackWidth = params.trackWidth;
    gamma = 0.2;
    error_factor = 5;
    reg_factor = 10;
    last_idx = 0;
    
    cost_gradient = zeros(size(X_flat));
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
        
       del2_dxi = 2*cos(phi_i)*(cos(phi_i)*(x_i - xc_i) + sin(phi_i)*(y_i - yc_i));
       dec2_dxi = -2*sin(phi_i)*(cos(phi_i)*(y_i - yc_i) - sin(phi_i)*(x_i - xc_i));
       del2_dyi = 2*sin(phi_i)*(cos(phi_i)*(x_i - xc_i) + sin(phi_i)*(y_i - yc_i));
       dec2_dyi = 2*cos(phi_i)*(cos(phi_i)*(y_i - yc_i) - sin(phi_i)*(x_i - xc_i));
       
       cost_gradient((ModelParams.nx+ModelParams.nu)*(i-1) + 1) = cost_gradient((ModelParams.nx+ModelParams.nu)*(i-1) + 1) + error_factor*del2_dxi + error_factor*dec2_dxi;
       cost_gradient((ModelParams.nx+ModelParams.nu)*(i-1) + 2) = cost_gradient((ModelParams.nx+ModelParams.nu)*(i-1) + 2) + error_factor*del2_dyi + error_factor*dec2_dyi;
       cost_gradient((ModelParams.nx+ModelParams.nu)*(i-1) + ModelParams.nx + 1) = cost_gradient((ModelParams.nx+ModelParams.nu)*(i-1)  + ModelParams.nx + 1) - gamma;
       
       progress = input_i(1);
        
       if i > 1 && idx > 1
           lreg = (norm(input_i - X_u(i-1,:)'))^2;% + (norm(progress - track.progress(last_idx,idx-1)))^2; 
           D1 = X_u(i-1,1);
           D2 = X_u(i,1);
           delta1 = X_u(i-1,2);
           delta2 = X_u(i,2);
           dlreg_dD1 = 2*(D1-D2);
           dlreg_dD2 = 2*(D2-D1);
           dlreg_ddelta1 = 2*(delta1 - delta2);
           dlreg_ddelta2 = 2*(delta2 - delta1);
           
           cost_gradient((ModelParams.nx+ModelParams.nu)*(i-2) + ModelParams.nx+ 1) = cost_gradient((ModelParams.nx+ModelParams.nu)*(i-2) + ModelParams.nx+ 1) + reg_factor*dlreg_dD1;
           cost_gradient((ModelParams.nx+ModelParams.nu)*(i-1) + ModelParams.nx+ 1) = cost_gradient((ModelParams.nx+ModelParams.nu)*(i-1) + ModelParams.nx+ 1) + reg_factor*dlreg_dD2;
           cost_gradient((ModelParams.nx+ModelParams.nu)*(i-2) + ModelParams.nx+ 2) = cost_gradient((ModelParams.nx+ModelParams.nu)*(i-2) + ModelParams.nx+ 2) + reg_factor*dlreg_ddelta1;
           cost_gradient((ModelParams.nx+ModelParams.nu)*(i-1) + ModelParams.nx+ 2) = cost_gradient((ModelParams.nx+ModelParams.nu)*(i-1) + ModelParams.nx+ 2) + reg_factor*dlreg_ddelta2;
           
           final_cost = final_cost + error_factor*ec_i^2 + error_factor*el_i^2-gamma*progress + reg_factor*lreg; 
       else
           final_cost = final_cost + error_factor*ec_i^2 + error_factor*el_i^2-gamma*progress; % mancalreg
       end
       last_idx = idx;
    end
end
