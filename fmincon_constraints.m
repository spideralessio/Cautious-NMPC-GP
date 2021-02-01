function [cineq, ceq, cineq_grad, ceq_grad] = fmincon_constraints(X_flat,params)
    if params.modified
        load('modified_bycicle_params.mat');
    else
        ModelParams = bycicle_params();
    end
    Horizon = params.Horizon;
    X = reshape(X_flat, [], Horizon)';
    X_x = X(:, 1:ModelParams.nx);
    X_u = X(:, ModelParams.nx+1: ModelParams.nx+ModelParams.nu);
    
    n = ModelParams.nx + ModelParams.nu;
    trackWidth = params.trackWidth;
    track = params.track;
    cineq = zeros(Horizon, 1);
    cineq_grad = zeros(Horizon*(ModelParams.nx+ModelParams.nu), Horizon);
    last_idx = 0;
    for i=1:Horizon
       state_i = X_x(i,:)';
       input_i = X_u(i,:)';

       x_i = state_i(ModelParams.stateindex_x);
       y_i = state_i(ModelParams.stateindex_y);
       theta_i = state_i(ModelParams.stateindex_theta);

      [~, idx] = min(abs(track.progress(1,:)-theta_i));

      if (idx > 1)
        tmp_idx = idx-1;
      else
        tmp_idx = idx+1;
      end
      xc_i = ((track.center_rounded(1,idx) - track.center_rounded(1,tmp_idx)) / (track.progress(1,idx) - track.progress(1,tmp_idx))) * (theta_i - track.progress(1,idx)) + track.center_rounded(1,idx); %linear interpolation for xc
      yc_i = ((track.center_rounded(2,idx) - track.center_rounded(2,tmp_idx)) / (track.progress(1,idx) - track.progress(1,tmp_idx))) * (theta_i - track.progress(1,idx)) + track.center_rounded(2,idx); %linear interpolation for yc
      c = [xc_i;yc_i];
       
       [c,idx] = get_c([x_i;y_i], track);
       err = norm([x_i;y_i]-c);
       
       derr_dx = (x_i - c(1))/err;
       derr_dy = (y_i - c(2))/err;
       cineq_grad((ModelParams.nx+ModelParams.nu)*(i-1) + 1, i) = cineq_grad((ModelParams.nx+ModelParams.nu)*(i-1) + 1, i) + derr_dx;
       cineq_grad((ModelParams.nx+ModelParams.nu)*(i-1) + 2, i) = cineq_grad((ModelParams.nx+ModelParams.nu)*(i-1) + 2, i) + derr_dy;
       cineq(i) = err - trackWidth/2;
       
    end
    
    Bd = [zeros(3); eye(3); zeros(1,3);
    ceq = zeros((Horizon-1)*ModelParams.nx,1);
    ceq_grad = zeros(Horizon*(ModelParams.nx+ModelParams.nu), (Horizon-1)*ModelParams.nx);
    for i=1:Horizon-1
        state_i = X_x(i,:)';
        input_i = X_u(i,:)';
        state_next = state_i + params.Ts*bycicle_model(state_i, input_i, params) + Bd*evaluate_gp(state_i, input_i);
        constraint_indexes = [1:ModelParams.nx] + (i-1)*ModelParams.nx;
        ceq(constraint_indexes,1) = X_x(i+1,:)' - state_next;
        
        f_grad = - params.Ts*bycicle_model_grad(state_i, input_i, params);
                
        ceq_grad((ModelParams.nx+ModelParams.nu)*(i-1) + [1:ModelParams.nx+ModelParams.nu], constraint_indexes) = ceq_grad((ModelParams.nx+ModelParams.nu)*(i-1) + [1:ModelParams.nx+ModelParams.nu], constraint_indexes) + f_grad; % derivative wrt cur state&inp
       
        ceq_grad((ModelParams.nx+ModelParams.nu)*(i-1) + [1:ModelParams.nx], constraint_indexes) = ceq_grad((ModelParams.nx+ModelParams.nu)*(i-1) + [1:ModelParams.nx], constraint_indexes) - eye(ModelParams.nx); % derivative wrt cur state summed to 1
        
        ceq_grad((ModelParams.nx+ModelParams.nu)*(i) + [1:ModelParams.nx], constraint_indexes) = ceq_grad((ModelParams.nx+ModelParams.nu)*(i) + [1:ModelParams.nx], constraint_indexes) + eye(ModelParams.nx); % derivative wrt next state is -1
%     
    end
    
    
    
end