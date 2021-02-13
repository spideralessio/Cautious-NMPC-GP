clear all
clc
close all


load('gp_data.mat');

Bd = [zeros(3); eye(3); zeros(1,3)];
Bd_pinv = pinv(Bd);

X = [data.x(:,1:length(data.x)-1);data.u(:,1:length(data.x)-1)]';

y = data.x(:,2:length(data.x));
size(y)

f = zeros(size(y));

Ts = 0.03
params = struct;
params.modified = true;
prova = y;
for i=1:length(f)
    f(:,i) = data.x(:,i) + Ts*bycicle_model(data.x(:,i), data.u(:,i), params);
    %f(:,i) = bycicle_step(data.x(:,i), data.u(:,i), Ts);

end

y = Bd_pinv*(y - f);
 
y = y';

X_tilde = [X,ones(length(X),1)];

% y = wx + b
% y = wX
% X = [x 1]

%3x11
%522x11

W = pinv(X_tilde)*y;

w = W(1:length(W)-1,:)
b = W(length(W),:)





