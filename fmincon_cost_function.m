function [final_cost, cost_gradient] = fmincon_cost_function(X_flat, params)
    ModelParams=bycicle_params();
    Horizon = params.Horizon;
    X = reshape(X_flat, [], Horizon)';
    X_x = X(:, 1:ModelParams.nx);
    X_u = X(:, ModelParams.nx+1: ModelParams.nx+ModelParams.nu);
    
    
    final_cost = 0;
    track = params.track;
    trackWidth = params.trackWidth;
    gamma = 1;
    error_factor = 1;
    reg_factor = 1;
    
    cost_gradient = zeros(size(X_flat));
    for i=1:Horizon
      state_i = X_x(i,:)';
      input_i = X_u(i,:)';
      x_i = state_i(ModelParams.stateindex_x);
      y_i = state_i(ModelParams.stateindex_y);
      theta_i = state_i(ModelParams.stateindex_theta);
      progress = input_i(ModelParams.inputindex_vtheta);

      [~, idx] = min(abs(track.progress(1,:)-theta_i));

      if (idx > 1)
        tmp_idx = idx-1;
      else
        tmp_idx = idx+1;
      end
      xc_i = ((track.center_rounded(1,idx) - track.center_rounded(1,tmp_idx)) / (track.progress(1,idx) - track.progress(1,tmp_idx))) * (theta_i - track.progress(1,idx)) + track.center_rounded(1,idx); %linear interpolation for xc
      yc_i = ((track.center_rounded(2,idx) - track.center_rounded(2,tmp_idx)) / (track.progress(1,idx) - track.progress(1,tmp_idx))) * (theta_i - track.progress(1,idx)) + track.center_rounded(2,idx); %linear interpolation for yc

      phi_i = atan2(track.center_rounded(2,idx)- yc_i, track.center_rounded(1,idx)- xc_i);
       
      el_i = -cos(phi_i)*(x_i - xc_i) - sin(phi_i)*(y_i-yc_i);
      ec_i = sin(phi_i)*(x_i - xc_i) - cos(phi_i)*(y_i-yc_i);
        
      del2_dxi = 2*cos(phi_i)*(cos(phi_i)*(x_i - xc_i) + sin(phi_i)*(y_i - yc_i));
      dec2_dxi = -2*sin(phi_i)*(cos(phi_i)*(y_i - yc_i) - sin(phi_i)*(x_i - xc_i));
      del2_dyi = 2*sin(phi_i)*(cos(phi_i)*(x_i - xc_i) + sin(phi_i)*(y_i - yc_i));
      dec2_dyi = 2*cos(phi_i)*(cos(phi_i)*(y_i - yc_i) - sin(phi_i)*(x_i - xc_i));

      cost_gradient((ModelParams.nx+ModelParams.nu)*(i-1) + 1) = cost_gradient((ModelParams.nx+ModelParams.nu)*(i-1) + 1) + error_factor*del2_dxi + error_factor*dec2_dxi;
      cost_gradient((ModelParams.nx+ModelParams.nu)*(i-1) + 2) = cost_gradient((ModelParams.nx+ModelParams.nu)*(i-1) + 2) + error_factor*del2_dyi + error_factor*dec2_dyi;
      cost_gradient((ModelParams.nx+ModelParams.nu)*(i-1) + ModelParams.nx + 3) = cost_gradient((ModelParams.nx+ModelParams.nu)*(i-1)  + ModelParams.nx + 3) - gamma;
       
        
      if i > 1 && idx > 1
         lreg = (norm(input_i - X_u(i-1,:)'))^2;
         u1 = X_u(i-1,:);
         u2 = X_u(i,:);

         dlreg_du1 = 2*(u1 - u2)';
         dlreg_du2 = 2*(u2 - u1)';
         cost_gradient((ModelParams.nx+ModelParams.nu)*(i-2) + ModelParams.nx+ [1:ModelParams.nu]) = cost_gradient((ModelParams.nx+ModelParams.nu)*(i-2) + ModelParams.nx+ [1:ModelParams.nu])  + reg_factor*dlreg_du1;
         cost_gradient((ModelParams.nx+ModelParams.nu)*(i-1) + ModelParams.nx+ [1:ModelParams.nu]) = cost_gradient((ModelParams.nx+ModelParams.nu)*(i-1) + ModelParams.nx+ [1:ModelParams.nu])  + reg_factor*dlreg_du2;
         
         final_cost = final_cost + error_factor*ec_i^2 + error_factor*el_i^2-gamma*progress + reg_factor*lreg; 
      else
         final_cost = final_cost + error_factor*ec_i^2 + error_factor*el_i^2-gamma*progress; % mancalreg
      end
    end
end
