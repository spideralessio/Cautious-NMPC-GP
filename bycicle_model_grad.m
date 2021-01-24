function [grad]=DiscretizedLinearizedModel(Xbar_k,Ubar_k)
  ModelParams = bycicle_params();
  sx=ModelParams.sx-1;
  su=ModelParams.su-1;
  

  Cm1=ModelParams.Cm1;
  Cm2=ModelParams.Cm2;
  Cr0=ModelParams.Cr0;
  Cr2=ModelParams.Cr2;
  B_r=ModelParams.Br;
  C_r=ModelParams.Cr;
  D_r=ModelParams.Dr;
  B_f=ModelParams.Bf;
  C_f=ModelParams.Cf;
  D_f=ModelParams.Df;
  m  =ModelParams.m;
  Iz=ModelParams.Iz;
  l_f=ModelParams.lf;
  l_r=ModelParams.lr;

  
  phi   =Xbar_k(ModelParams.stateindex_phi);
  v_x     =Xbar_k(ModelParams.stateindex_vx);
  v_y     =Xbar_k(ModelParams.stateindex_vy);
  omega   =Xbar_k(ModelParams.stateindex_omega);
  
  
  D     =Ubar_k(ModelParams.inputindex_D);
  delta =Ubar_k(ModelParams.inputindex_delta);
  
  if(v_x < 0.5)
      v_x = v_x;
      v_y = 0;
      omega = 0;
      delta = 0;
  	if(v_x < 0.3)
          v_x = 0.3;
      end
  end
  
  alpha_f = -atan2(l_f*omega + v_y,v_x)+delta;
  alpha_r =  atan2(l_r*omega - v_y,v_x);

  F_fy = D_f*sin(C_f*atan(B_f*alpha_f));
  F_ry = D_r*sin(C_r*atan(B_r*alpha_r));

  F_rx = (Cm1*D-Cm2*D*v_x-Cr0-Cr2*v_x^2);

  f=[v_x*cos(phi) - v_y*sin(phi);
     v_y*cos(phi) + v_x*sin(phi);
     omega;
     1/m*(F_rx - F_fy*sin(delta) + m*v_y*omega);
     1/m*(F_ry + F_fy*cos(delta) - m*v_x*omega);
     1/Iz*(F_fy*l_f*cos(delta)- F_ry*l_r)];
 %% Derivatives of the force laws
 % F_rx
 dFrx_dvx = -Cm2*D - 2*Cr2*v_x;
 dFrx_dD  = Cm1 - Cm2*v_x;
 % F_ry
 dFry_dvx = ((B_r*C_r*D_r*cos(C_r*atan(B_r*alpha_r)))/(1+B_r^2*alpha_r^2))...
           *(-(l_r*omega - v_y)/((-l_r*omega + v_y)^2+v_x^2));
       
 dFry_dvy = ((B_r*C_r*D_r*cos(C_r*atan(B_r*alpha_r)))/(1+B_r^2*alpha_r^2))...
           *((-v_x)/((-l_r*omega + v_y)^2+v_x^2));
       
 dFry_domega = ((B_r*C_r*D_r*cos(C_r*atan(B_r*alpha_r)))/(1+B_r^2*alpha_r^2))...
           *((l_r*v_x)/((-l_r*omega + v_y)^2+v_x^2));
 % F_fy 
 
 dFfy_dvx =     (B_f*C_f*D_f*cos(C_f*atan(B_f*alpha_f)))/(1+B_f^2*alpha_f^2)...
               *((l_f*omega + v_y)/((l_f*omega + v_y)^2+v_x^2));
           
 dFfy_dvy =     (B_f*C_f*D_f*cos(C_f*atan(B_f*alpha_f)))/(1+B_f^2*alpha_f^2)...
               *(-v_x/((l_f*omega + v_y)^2+v_x^2));
           
 dFfy_domega =    (B_f*C_f*D_f*cos(C_f*atan(B_f*alpha_f)))/(1+B_f^2*alpha_f^2)...
               *((-l_f*v_x)/((l_f*omega + v_y)^2+v_x^2));
           
 dFfy_ddelta =  (B_f*C_f*D_f*cos(C_f*atan(B_f*alpha_f)))/(1+B_f^2*alpha_f^2); 
 %% 
 % f1 = v_x*cos(phi) - v_y*sin(phi);
 df1_dphi = -v_x*sin(phi) - v_y*cos(phi);
 df1_dvx  = cos(phi);
 df1_dvy  = -sin(phi);
 
 % f2 = v_y*cos(phi) + v_x*sin(phi);
 df2_dphi = -v_y*sin(phi) + v_x*cos(phi);
 df2_dvx  = sin(phi);
 df2_dvy  = cos(phi);
 
 % f3 = omega;
 df3_domega = 1;
 
 % f4 = 1/m*(F_rx - F_fy*sin(delta) + m*v_y*omega);
 df4_dvx     = 1/m*(dFrx_dvx - dFfy_dvx*sin(delta));
 df4_dvy     = 1/m*(           - dFfy_dvy*sin(delta)     + m*omega);
 df4_domega = 1/m*(           - dFfy_domega*sin(delta) + m*v_y);
 df4_dD      = 1/m*     dFrx_dD;
 df4_ddelta  = 1/m*(           - dFfy_ddelta*sin(delta)  - F_fy*cos(delta));
 
 % f5 = 1/m*(F_ry + F_fy*cos(delta) - m*v_x*omega);
 df5_dvx     = 1/m*(dFry_dvx     + dFfy_dvx*cos(delta)     - m*omega);
 df5_dvy     = 1/m*(dFry_dvy     + dFfy_dvy*cos(delta));   
 df5_domega = 1/m*(dFry_domega + dFfy_domega*cos(delta) - m*v_x);
 df5_ddelta  = 1/m*(               dFfy_ddelta*cos(delta)  - F_fy*sin(delta));
 
 % f6 = 1/Iz*(F_fy*l_f*cos(delta)- F_ry*l_r)
 df6_dvx     = 1/Iz*(dFfy_dvx*l_f*cos(delta)     - dFry_dvx*l_r);
 df6_dvy     = 1/Iz*(dFfy_dvy*l_f*cos(delta)     - dFry_dvy*l_r);
 df6_domega = 1/Iz*(dFfy_domega*l_f*cos(delta) - dFry_domega*l_r);
 df6_ddelta  = 1/Iz*(dFfy_ddelta*l_f*cos(delta)  - F_fy*l_f*sin(delta));
 
  % Jacobians
  
  Ac=[0 0  df1_dphi        df1_dvx         df1_dvy        0           0;
      0 0  df2_dphi        df2_dvx         df2_dvy        0           0;
      0 0  0               0               0              df3_domega  0;
      0 0  0               df4_dvx         df4_dvy        df4_domega  0;
      0 0  0               df5_dvx         df5_dvy        df5_domega  0;
      0 0  0               df6_dvx         df6_dvy        df6_domega  0;
      0 0  0               0               0              0           0];
  
  Bc=[0         0            0;
      0         0            0;
      0         0            0;
      df4_dD    df4_ddelta   0;
      0         df5_ddelta   0;
      0         df6_ddelta   0;
      0         0            1];

    
  grad = [Ac Bc]';