function [cineq] = constraints(X,U,e,data,params)
%CONSTRAINTS Summary of this function goes here
%   Detailed explanation goes here
    ModelParams=bycicle_params();
    N = length(X);
    traj = params.traj;
    trackWidth = params.trackWidth;
    track = params.track;
    cineq = [];
    for i=1:N
       state_i = X(i,:)';
       input_i = U(i,:)';

       x_i = state_i(ModelParams.stateindex_x);
       y_i = state_i(ModelParams.stateindex_y);
       theta_i = state_i(ModelParams.stateindex_theta);
       xc_i = ppval(traj.ppx,theta_i);
       yc_i = ppval(traj.ppy,theta_i);
        
       err = norm([x_i;y_i] - [xc_i;yc_i]);
       
       cineq = [cineq;err - trackWidth/2;input_i(3)-0.8];
       
    end
    
end

