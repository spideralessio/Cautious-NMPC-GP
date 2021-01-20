clear
clc
load track.mat

Ts = 0.02;

startIdx = 1 % index of starting point on track
last_closestIdx = startIdx;

ModelParams = bycicle_params();

vx0 = 1;

trackWidth = norm(track.inner(:,1)-track.outer(:,1));

x0 = [track.center(1,startIdx),track.center(2,startIdx),... % point on centerline
      -pi+atan2(track.center(1,startIdx+1) - track.center(1,startIdx), track.center(2,startIdx+1) - track.center(2,startIdx)),... % aligned with centerline
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
nlobj.PredictionHorizon = 40;
nlobj.ControlHorizon = 40;
[u, opt, info] = nlmpcmove(nlobj,x,u,[],[],nloptions);


for i=1:100
    [u, opt, info] = nlmpcmove(nlobj,x,u,[],[],nloptions)
    u = info.MVopt(1,:)';
    x = bycicle_step(x, u, Ts)
    
    hold off
    figure(1);
    plot(track.outer(1,:),track.outer(2,:),'r')
    hold on
    plot(info.Xopt(:,1), info.Xopt(:,2), 'b');
    c = [];
    for j=1:length(info.Xopt)
       c = [c, get_c(info.Xopt(j,:)', track)];
    end
    
    scatter(c(1,:), c(2,:))
    plot(track.inner(1,:),track.inner(2,:),'r')
    plot(track.center(1,:),track.center(2,:),'--r')
    carBox(x,ModelParams.W/2,ModelParams.L/2)
end