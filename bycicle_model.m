function xdot=bycicle_model(x,u,params)
    if params.modified
        load('modified_bycicle_params.mat');
    else
        ModelParams = bycicle_params();
    end
    Cm1=ModelParams.Cm1;
    Cm2=ModelParams.Cm2;
    Cr0=ModelParams.Cr0;
    Cr2=ModelParams.Cr2;
    
            
    B_r = ModelParams.Br;
    C_r = ModelParams.Cr;
    D_r = ModelParams.Dr;
    B_f = ModelParams.Bf;
    C_f = ModelParams.Cf;
    D_f = ModelParams.Df;
    
    m = ModelParams.m;
    Iz = ModelParams.Iz;
    l_f = ModelParams.lf;
    l_r = ModelParams.lr;

    
    phi   =x(ModelParams.stateindex_phi);
    v_x     =x(ModelParams.stateindex_vx);
    v_y     =x(ModelParams.stateindex_vy);
    omega   =x(ModelParams.stateindex_omega);
    D     =u(ModelParams.inputindex_D);
    delta =u(ModelParams.inputindex_delta);
    vtheta =u(ModelParams.inputindex_vtheta);
    
    alpha_f = -atan2(l_f*omega + v_y,abs(v_x))+delta;
    alpha_r =  atan2(l_r*omega - v_y,abs(v_x));

    F_fy = D_f*sin(C_f*atan(B_f*alpha_f));
    F_ry = D_r*sin(C_r*atan(B_r*alpha_r));

    F_rx = (Cm1*D-Cm2*D*v_x-Cr0-Cr2*v_x^2);
    
    xdot=[v_x*cos(phi) - v_y*sin(phi);
       v_y*cos(phi) + v_x*sin(phi);
       omega;
       1/m*(F_rx - F_fy*sin(delta) + m*v_y*omega);
       1/m*(F_ry + F_fy*cos(delta) - m*v_x*omega);
       1/Iz*(F_fy*l_f*cos(delta)- F_ry*l_r);
       vtheta];

    
return