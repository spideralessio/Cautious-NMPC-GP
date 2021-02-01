random_num = @()(rand - 0.5)*2*0.15;
edit_params = @(x) x+x*random_num();
ModelParams = bycicle_params();

ModelParams.m = edit_params(ModelParams.m)
ModelParams.Iz = edit_params(ModelParams.Iz)
ModelParams.lf = edit_params(ModelParams.lf)
ModelParams.lr = edit_params(ModelParams.lr)
ModelParams.Cm1 = edit_params(ModelParams.Cm1)
ModelParams.Cm2 = edit_params(ModelParams.Cm2)
ModelParams.Cr0 = edit_params(ModelParams.Cr0)
ModelParams.Cr2 = edit_params(ModelParams.Cr2)
ModelParams.Br = edit_params(ModelParams.Br)
ModelParams.Cr = edit_params(ModelParams.Cr)
ModelParams.Dr = edit_params(ModelParams.Dr)
ModelParams.Bf = edit_params(ModelParams.Bf)
ModelParams.Cf = edit_params(ModelParams.Cf)
ModelParams.Df = edit_params(ModelParams.Df)
ModelParams.L = edit_params(ModelParams.L)
ModelParams.W = edit_params(ModelParams.W)

