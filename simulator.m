close all
clear
clc
load track2.mat
addpath('splines');
track = track2;

Ts = 0.1;

startIdx = 1 % index of starting point on track
last_closestIdx = startIdx;

ModelParams = bycicle_params();
[traj, borders] =splinify(track);
tl = traj.ppy.breaks(end);

vx0 = 1;

trackWidth = norm(track.inner(:,1)-track.outer(:,1));
[theta, ~] = findTheta([track.center(1,startIdx),track.center(2,startIdx)],track.center,traj.ppx.breaks,trackWidth,startIdx);

x0 = [track.center(1,startIdx),track.center(2,startIdx),... % point on centerline
      atan2(ppval(traj.dppy,theta),ppval(traj.dppx,theta)),... % aligned with centerline
      vx0 ,0,0, theta]';
  
  
figure(1);
plot(track.outer(1,:),track.outer(2,:),'r')
hold on
plot(track.inner(1,:),track.inner(2,:),'r')
plot(track.center(1,:),track.center(2,:),'--r')
carBox(x0,ModelParams.W/2,ModelParams.L/2)

x = x0;
u = [0;0;0];




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
params.traj = traj;
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
%     x = info.Xopt(1,:)'
    [ theta, last_closestIdx] = findTheta(x,track.center,traj.ppx.breaks,trackWidth,last_closestIdx);
    x(ModelParams.stateindex_theta) = theta;
    xcc = ppval(traj.ppx, info.Xopt(:,7));
    ycc = ppval(traj.ppy, info.Xopt(:,7));
    hold off
    figure(1);
    plot(track.outer(1,:),track.outer(2,:),'r')
    hold on
    plot(xcc,ycc, 'b');
    plot(info.Xopt(:,1), info.Xopt(:,2), '--b');
    plot(track.inner(1,:),track.inner(2,:),'r')
    plot(track.center(1,:),track.center(2,:),'--r')
    carBox(x,ModelParams.W/2,ModelParams.L/2)
end
