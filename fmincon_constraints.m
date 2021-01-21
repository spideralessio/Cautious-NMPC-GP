function [cineq, ceq] = constraints(X_flat,params)
    ModelParams=bycicle_params();
    Horizon = params.Horizon;
    X = reshape(X_flat, Horizon, []);
    X_x = X(:, 1:ModelParams.nx);
    X_u = X(:, ModelParams.nx+1: ModelParams.nx+ModelParams.nu);
    
    n = ModelParams.nx + ModelParams.nu;
    trackWidth = params.trackWidth;
    track = params.track;
    cineq = zeros(Horizon, 1);
    for i=1:Horizon
       state_i = X_x(i,:)';
       input_i = X_u(i,:)';

       x_i = state_i(ModelParams.stateindex_x);
       y_i = state_i(ModelParams.stateindex_y);
       
       p = round(([x_i; y_i]-[track.xmin;track.ymin])/track.step); % get rows and cols for current position on distance mat
       [rows, cols] = size(track.D); % get num rows and cols of distance matrix
       p = max(p,1);
       p = min(p,[cols;rows-1]);
       err = track.D(rows-p(2), p(1)); % evaluate distance from [xc, yc]
       err = double(err);  
       
%        c = get_c([x_i;y_i], track);
%        err = norm([x_i;y_i]-c);
       
       cineq(i) = err - trackWidth/2;
       
    end
    
    
    ceq = zeros((Horizon-1)*ModelParams.nx,1);
    for i=1:Horizon-1
        state_i = X_x(i,:)';
        input_i = X_u(i,:)';
        state_next = state_i + params.Ts*bycicle_model(state_i, input_i);
        ceq(1 + (i-1)*ModelParams.nx:ModelParams.nx + (i-1)*ModelParams.nx,1) = X_x(i+1,:)' - state_next;
    end
    
    
    
end