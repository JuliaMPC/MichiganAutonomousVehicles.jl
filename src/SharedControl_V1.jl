module SharedControl

using NLOptControl
using VehicleModels
using JuMP
using Ipopt
using DataFrames
using Parameters
using KNITRO
import MathProgBase

include("CaseModule.jl")
using .CaseModule

export
      initializeSharedControl,
      sharedControl,
      feasible,
      OA

"""
mdl,n,r,params = initializeSharedControl(c);
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 1/27/2017, Last Modified: 3/28/2017 \n
--------------------------------------------------------------------------------------\n
"""
function initializeSharedControl(c)
  pa = Vpara(x_min=-200.,x_max=50.,y_min=-100.,y_max=800.,Fz_min=1.0);
  n = NLOpt();
  XF = [NaN, NaN,  NaN,  NaN, NaN];
  @unpack_Vpara pa
  XL = [x_min, y_min, NaN, NaN, psi_min];
  XU = [x_max, y_max, NaN, NaN, psi_max];
  CL = [sa_min]; CU = [sa_max];
  n = define(n,stateEquations=ThreeDOFv1,numStates=5,numControls=1,X0=c.m.X0,XF=XF,XL=XL,XU=XU,CL=CL,CU=CU)

  # variable names
  names = [:x,:y,:v,:r,:psi];
  descriptions = ["X (m)","Y (m)","Lateral Velocity (m/s)","Yaw Rate (rad/s)","Yaw Angle (rad)"];
  stateNames(n,names,descriptions)
  names = [:sa];
  descriptions = ["Steering Angle (rad)"];
  controlNames(n,names,descriptions);

  # configure problem
  n=configure(n,Ni=2,Nck=[10,10];(:integrationMethod => :ps),(:integrationScheme => :lgrExplicit),(:finalTimeDV => false),(:tf => c.m.tp))

  # define optimization problem
  mdl=defineSolver(n,c);

  # add parameters
  @NLparameter(mdl, ux_param==c.m.UX); # inital vehicle speed
  @NLparameter(mdl, sa_param==0.0);  # initial driver steering command

  # obstacles
  Q = size(c.o.A)[1]; # number of obstacles
  @NLparameter(mdl, a[i=1:Q] == c.o.A[i]);
  @NLparameter(mdl, b[i=1:Q] == c.o.B[i]);
  @NLparameter(mdl, X_0[i=1:Q] == c.o.X0[i]);
  @NLparameter(mdl, Y_0[i=1:Q] == c.o.Y0[i]);
  params = [pa,ux_param,sa_param,a,b,X_0,Y_0];

  # define ocp
  s=Settings(;save=false,MPC=true);
  n,r=OCPdef(mdl,n,s,params);
  obj = integrate(mdl,n,r.u[:,1];D=sa_param,(:variable=>:control),(:integrand=>:squared),(:integrandAlgebra=>:subtract));
  @NLobjective(mdl, Min, obj)

  # obstacle postion after the intial postion
  @NLexpression(mdl, X_obs[j=1:Q,i=1:n.numStatePoints], X_0[j])
  @NLexpression(mdl, Y_obs[j=1:Q,i=1:n.numStatePoints], Y_0[j])

  # constraint position
  obs_con=@NLconstraint(mdl, obs_con[j=1:Q,i=1:n.numStatePoints-1], 1 <= ((r.x[(i+1),1]-X_obs[j,i])^2)/((a[j]+c.m.sm)^2) + ((r.x[(i+1),2]-Y_obs[j,i])^2)/((b[j]+c.m.sm)^2));
  newConstraint(r,obs_con,:obs_con);

  # intial optimization
  optimize(mdl,n,r,s);

  # set up constraint data
  evalConstraints(n,r);

  # initialize feasibility checking functionality
  d=JuMP.NLPEvaluator(mdl);
  MathProgBase.initialize(d,[:Grad]);
  feasible(mdl,d,n,r,s,params,c.m.X0,0.0,c.m.UX);
  return mdl,d,n,r,params
end


"""
t_opt, sa_opt, t_sample, sa_sample, status = sharedControl(mdl,n,r,s,params,X0,SA,UX;Iter);
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 1/27/2017, Last Modified: 3/28/2017 \n
--------------------------------------------------------------------------------------\n
"""
function sharedControl(mdl,n,r,s,params,X0,SA,UX;Iter::Int64=0)

  setvalue(params[2],UX)   # update speed
  setvalue(params[3],SA)   # update desired steering angle
  updateX0(n,r,X0;(:userUpdate=>true))    # user defined update of X0

  updateStates(n,r);
  status=optimize(mdl,n,r,s;Iter=Iter);

  # sample solution
  sp_SA=Linear_Spline(r.t_ctr,r.U[:,1]);
  t_sample = Vector(0:0.01:n.mpc.tex);
  sa_sample  = sp_SA[t_sample];
  if status!=:Infeasible; status=1.; else status=0.; end
  return r.t_ctr[1:end-1], r.U[:,1], t_sample, sa_sample, status
end

"""
OA(s)
--------------------------------------------------------------------------------------\n
Authors: Yingshi and Huckleberry Febbo, Graduate Students, University of Michigan
Date Create: 1/15/2017, Last Modified: 3/28/2017 \n
--------------------------------------------------------------------------------------\n
"""
function OA(s,s1,s2)
  # Parameters
  Iter=1;RunJulia=1;NumObs=3;status=1.0;

  if s.save
    global AAA=[];
    global PP=[];
    global R_obs=[];
    global X0_obs=[];
    global Y0_obs=[];
    global X0_all=[];
  end

  while(RunJulia>0.0)
    print("Iteration:"); println(Iter);
    MsgString = recv(s1);
    MsgIn = zeros(19);
    allidx = find(MsgString->MsgString == 0x20,MsgString);
    allidx = [0;allidx];
    for i in 1:19  #length(allidx)-1
      MsgIn[i] = parse(Float64,String(copy(MsgString[allidx[i]+1:allidx[i+1]-1])));
    end

    tic();
    SA = MsgIn[1];                # Vehicle steering angle
    UX = MsgIn[2];                # Longitudinal speed
    X0 = MsgIn[3:7];              # global X, global y, lateral speed v, yaw rate r, yaw angle psi
    X_0obs       = MsgIn[8:8+NumObs-1];             # ObsX
    Y_0obs       = MsgIn[8+NumObs:8+2*NumObs-1];    # ObsY
    A            = MsgIn[8+2*NumObs:8+3*NumObs-1];  # ObsR
    B            = A;
    RunJulia     = MsgIn[8+3*NumObs+2];             # whether to update obstacles info.

    # update obstacle feild
    for i in 1:length(A)
      setvalue(params[3][1][i],A[i]);setvalue(params[3][1][i],B[i]);
      setvalue(params[3][1][i],X_0obs[i]); setvalue(params[3][1][i],Y_0obs[i]);
    end

    if s.save
      A2dfs(Settings(;reset=false),AAA,AL,AU);
      PP=append!(PP,Bool(pass));
      R_obs=append!(R_obs,[A]);
      X0_obs=append!(X0_obs,[X_0obs]);
      Y0_obs=append!(Y0_obs,[Y_0obs]);
      X0_all=append!(X0_all,[X0]);
    end

    t_opt, sa_opt, t_sample, sa_sample, status = sharedControl(mdl,n,r,s,params,X0,SA,UX;Iter=Iter) # SAMPLE OUTPUT
    Comp_time = toc();
    if status == 0.0 # if infeasible -> let user control
      MsgOut = [SA*ones(convert(Int64, floor(t_ocp/0.01))+1 );Comp_time;3;0]
    else
      MsgOut = [sa_sample;Comp_time;2;0];
    end

    # send UDP packets to client side
    MsgOut = [MsgOut;Float64(Iter)];
    MsgOutString = ' ';
    for j in 1:length(MsgOut)
        MsgOutString = string(MsgOutString,' ',MsgOut[j]);
    end
    MsgOutString = string(MsgOutString," \n");
    send(s2,ip"141.212.141.245",36881,MsgOutString);  # change this to the ip where you are running Simulink!
    Iter = Iter+1;
  end # while
end

"""
pass = feasible(mdl,n,d,r,s,params,X0,SA,UX;feas_tol=0.005);
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/13/2017, Last Modified: 2/21/2017 \n
--------------------------------------------------------------------------------------\n
"""
function feasible(mdl,d,n,r,s,params,X0,SA,UX;feas_tol::Float64=0.005)
  updateStates(n,r,params,X0,SA,UX)

  # simulate vehicle response
  SA=SA*ones(n.numStatePoints,); UX=UX*ones(n.numStatePoints,);
  sol=ThreeDOFv1(params[1],X0,r.t_st,SA,UX,r.t_st[1],r.t_st[end]);

  if s.save; u=[SA,UX]; plant2dfs(n,r,s,u,sol); end

  # pass values to constraint equations
  num = n.numStatePoints*n.numStates + n.numControlPoints*n.numControls + 1;  # TODO find other variable!!! 125 vs 126  IT IS TIME!!!! the intial time...
  values=zeros(num);
  for st in 1:n.numStates
    for j in 1:n.numStatePoints
     values[linearindex(r.x[j,st])] = sol(r.t_st[j])[st];
    end
  end
  idx=1;
  for j in linearindex(r.u[1,1]):linearindex(r.u[end,1]);  # NOT u?? -> changed from r.x to r.u..
    values[linearindex(r.u[idx,1])] = SA[idx]; idx+=1;
  end
  g=zeros(MathProgBase.numconstr(d.m)); # TODO fix -->number of constraints is different from num TODO remove that if we are not doing mpc??
  MathProgBase.eval_g(d,g,values);  # is this broken"
  b=JuMP.constraintbounds(mdl);
  AL=g-b[1]-feas_tol; AU=b[2]+feas_tol-g; # give tolerance

  # check obstacle avoidance constraints
  L=0;U=0;
  for i in 1:length(r.constraint.name)
    if r.constraint.name[i]==:obs_con
      L=r.constraint.nums[i][end][1];
      U=r.constraint.nums[i][end][2];
    end
  end
  AL=AL[L:U]; AU=AU[L:U];
  if minimum(sign([AL;AU]))==-1;pass=0.0;else;pass=1.0; end
  return pass, AL, AU
end

"""
A2dfs(s,AAA,AL,AU)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/17/2017, Last Modified: 2/17/2017 \n
--------------------------------------------------------------------------------------\n
"""
function A2dfs(s,AAA,AL,AU)
  dfs=DataFrame();
  dfs[:AL]=AL;
  dfs[:AU]=AU;
  if s.reset
    AAA=[dfs];
  else
    push!(AAA,dfs);
  end
end
"""
evalNum()
# to extract data from a particular iteration number
# will not work unless the data was saved; i.e., s.save = true
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/21/2017, Last Modified: 2/21/2017 \n
--------------------------------------------------------------------------------------\n
"""
function evalNum(Idx)
  eval_num=0;
  for i in 1:length(r.dfs_opt)
    if r.dfs_opt[i][:iter_num][1]==Idx
      eval_num=i;
      break
    end
  end
  print(eval_num);
  s=Settings(;format=:png,MPC=false);
  cd("results/test1"); allPlots(n,r,s,eval_num); cd(main_dir)
end


end # module
