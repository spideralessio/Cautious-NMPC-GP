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
    traj = params.traj;
    track = params.track;
    trackWidth = params.trackWidth;
    gamma = 0.5;
    
    for i=2:N
       state_i = X(i,:)';
       input_i = U(i,:)';
       final_cost = final_cost + input_i(1);
       x_i = state_i(ModelParams.stateindex_x);
       y_i = state_i(ModelParams.stateindex_y);
       theta_i = state_i(ModelParams.stateindex_theta);
       xc_i = ppval(traj.ppx,theta_i);
       yc_i = ppval(traj.ppy,theta_i);
       phi_i = atan2(ppval(traj.dppy,theta_i),ppval(traj.dppx,theta_i));
       
       el_i = -cos(phi_i)*(x_i - xc_i) - sin(phi_i)*(y_i-yc_i);
       ec_i = sin(phi_i)*(x_i - xc_i) - cos(phi_i)*(y_i-yc_i);
       
       lreg_i = norm(input_i - U(i-1,:)')^2;
       
       final_cost = final_cost + el_i^2 + ec_i^2 - gamma*input_i(3) + lreg_i;
    end
end

