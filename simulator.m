close all
clear
clc
load track.mat

Ts = 0.1;

startIdx = 1 % index of starting point on track
last_closestIdx = startIdx;

ModelParams = bycicle_params();

vx0 = 1;

trackWidth = norm(track.inner(:,1)-track.outer(:,1));

x0 = [track.center(1,startIdx),track.center(2,startIdx),... % point on centerline
      pi+atan2(track.center(1,startIdx+1) - track.center(1,startIdx), track.center(2,startIdx+1) - track.center(2,startIdx)),... % aligned with centerline
      vx0 ,0, 0]';
  
  
figure(1);
plot(track.outer(1,:),track.outer(2,:),'r')
hold on
plot(track.inner(1,:),track.inner(2,:),'r')
plot(track.center(1,:),track.center(2,:),'--r')
carBox(x0,ModelParams.W/2,ModelParams.L/2)

x = x0;
u = [0;0];




nlobj = nlmpc(ModelParams.nx,ModelParams.nx,ModelParams.nu);
nlobj.Ts = Ts;
nlobj.Model.StateFcn = @bycicle_model;
nlobj.MV(1).Min = ModelParams.Dmin;
nlobj.MV(1).Max = ModelParams.Dmax;
nlobj.MV(2).Min = ModelParams.deltamin;
nlobj.MV(2).Max = ModelParams.deltamax;
nlobj.Optimization.UseSuboptimalSolution = true;
nlobj.Model.NumberOfParameters = 1;
nloptions = nlmpcmoveopt;
params = struct;
params.track = track;
params.trackWidth = trackWidth;
nloptions.Parameters = {params};
nlobj.Optimization.CustomCostFcn = @cost_function;
nlobj.Optimization.CustomIneqConFcn = @constraints;
nlobj.Optimization.SolverOptions.MaxIter = 10;
nlobj.PredictionHorizon = 10;

for i=1:100
    [u, opt, info] = nlmpcmove(nlobj,x,u,[],[],nloptions);
    u = info.MVopt(1,:)';
    x = bycicle_step(x, u, Ts)
    p = round((x(1:2)-[track.xmin;track.ymin])/track.step); % get rows and cols for current position on distance mat
    [rows, cols] = size(track.D); % get num rows and cols of distance matrix
    idx = track.idx(rows-p(2), p(1)); % get index of [xc, yc] in track.center_rounded
    xc_i = track.center_rounded(1,idx);
    yc_i = track.center_rounded(2,idx);
    hold off
    figure(1);
    plot(track.outer(1,:),track.outer(2,:),'r')
    hold on
    scatter(xc_i, yc_i);
    plot(info.Xopt(:,1), info.Xopt(:,2), '--b');
    plot(track.inner(1,:),track.inner(2,:),'r')
    plot(track.center(1,:),track.center(2,:),'--r')
    carBox(x,ModelParams.W/2,ModelParams.L/2)
end
