module AutonomousControl

using NLOptControl
using VehicleModels
using JuMP
using DataFrames
using Parameters

include("CaseModule.jl")
using .CaseModule

export
      initializeAutonomousControl,
      autonomousControl,
      updateAutoParams!

"""
mdl,n,r,params = initializeAutonomousControl(c);
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/1/2017, Last Modified: 3/28/2017 \n
--------------------------------------------------------------------------------------\n
"""
function initializeAutonomousControl(c)
 pa=Vpara(x_min=c.m.Xlims[1],x_max=c.m.Xlims[2],y_min=c.m.Ylims[1],y_max=c.m.Ylims[2]);
 n=NLOpt(); @unpack_Vpara pa
 XF=[c.g.x_ref, c.g.y_ref, NaN, NaN, NaN, NaN, NaN, NaN];
 XL=[x_min, y_min, NaN, NaN, psi_min, sa_min, u_min, NaN];
 XU=[x_max, y_max, NaN, NaN, psi_max, sa_max, u_max, NaN];
 #XL=[NaN,NaN, NaN, NaN, psi_min, sa_min, u_min, NaN];
 #XU=[NaN,NaN, NaN, NaN, psi_max, sa_max, u_max, NaN];
 CL = [sr_min, jx_min]; CU = [sr_max, jx_max];
 define!(n,stateEquations=ThreeDOFv2,numStates=8,numControls=2,X0=copy(c.m.X0),XF=XF,XL=XL,XU=XU,CL=CL,CU=CU)
         # 1  2  3  4  5    6   7   8
 names = [:x,:y,:v,:r,:psi,:sa,:ux,:ax];
 descriptions = ["X (m)","Y (m)","Lateral Velocity (m/s)", "Yaw Rate (rad/s)","Yaw Angle (rad)", "Steering Angle (rad)", "Longitudinal Velocity (m/s)", "Longitudinal Acceleration (m/s^2)"];
 stateNames!(n,names,descriptions)
         # 1    2
 names = [:sr,:jx];
 descriptions = ["Steering Rate (rad/s)","Longitudinal Jerk (m/s^3)"];
 controlNames!(n,names,descriptions)
 params = [pa];   # vehicle parameters

 # configure problem
 configure!(n,Ni=c.m.Ni,Nck=c.m.Nck;(:integrationMethod => :ps),(:integrationScheme => :lgrExplicit),(:finalTimeDV => true))
 mdl=defineSolver!(n,c);

 # define tolerances
 XF_tol=[5.,5.,NaN,NaN,NaN,NaN,NaN,NaN];
 X0_tol=[0.05,0.05,0.05,0.05,0.01,0.001,0.05,0.05];
 defineTolerances!(n;X0_tol=X0_tol,XF_tol=XF_tol);

 # add parameters
 Q = size(c.o.A)[1]; # number of obstacles
 @NLparameter(mdl, a[i=1:Q] == copy(c.o.A[i]));
 @NLparameter(mdl, b[i=1:Q] == copy(c.o.B[i]));
 @NLparameter(mdl, X_0[i=1:Q] == copy(c.o.X0[i]));
 @NLparameter(mdl, Y_0[i=1:Q] == copy(c.o.Y0[i]));
 @NLparameter(mdl, speed_x[i=1:Q] == copy(c.o.s_x[i]));
 @NLparameter(mdl, speed_y[i=1:Q] == copy(c.o.s_y[i]));
 obs_params=[a,b,X_0,Y_0,speed_x,speed_y];

 # define ocp
 s=Settings(;save=false,MPC=true);
 r=OCPdef!(mdl,n,s,[pa]); #TODO add rest of terms in obj function

 # define objective function
 sr_obj=integrate!(mdl,n,r.u[:,1];C=c.w.sr,(:variable=>:control),(:integrand=>:squared))
 @NLobjective(mdl, Min, n.tf + sr_obj)

 # obstacle postion after the intial postion
 X_obs=@NLexpression(mdl, [j=1:Q,i=1:n.numStatePoints], X_0[j] + speed_x[j]*n.tV[i]);
 Y_obs=@NLexpression(mdl, [j=1:Q,i=1:n.numStatePoints], Y_0[j] + speed_y[j]*n.tV[i]);

 # constraint on position
 obs_con=@NLconstraint(mdl, [j=1:Q,i=1:n.numStatePoints-1], 1 <= ((r.x[(i+1),1]-X_obs[j,i])^2)/((a[j]+c.m.sm)^2) + ((r.x[(i+1),2]-Y_obs[j,i])^2)/((b[j]+c.m.sm)^2));
 newConstraint!(r,obs_con,:obs_con);

 # position parameters
 @NLparameter(mdl, X0_params[j=1:2]==n.X0[j]);

 # LiDAR constraint: ensure that all of the points are in the LiDAR range TODO LiDAR range constrait with x y constraints are messing it up
#  LiDAR_con=@NLconstraint(mdl, [i=1:n.numStatePoints-1], ((r.x[(i+1),1]-X0_params[1])^2+(r.x[(i+1),2]-X0_params[2])^2) <= (c.m.Lr + c.m.L_rd)^2); # not constraining the first state
#  newConstraint!(r,LiDAR_con,:LiDAR_con);

#  sm=createParallelModels(n,c,pa)

# intial optimization
 optimize!(mdl,n,r,s);

 # set mpc parameters
 initializeMPC!(n,r;FixedTp=c.m.FixedTp,PredictX0=c.m.PredictX0,tp=c.m.tp,tex=copy(c.m.tex),max_iter=c.m.mpc_max_iter);
 n.mpc.X0=[copy(c.m.X0)];

        #  1    2          3
 params=[pa,obs_params,X0_params];

 # save case data TODO    case_data, obs_data = case2dfs(c);
 return mdl, n, r, params
end

"""


--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/20/2017, Last Modified: 4/11/2017 \n
--------------------------------------------------------------------------------------\n
"""
function updateAutoParams!(n,r,c,params)

  # vehicle position for LiDAR
  setvalue(params[3][1],n.X0[1])
  setvalue(params[3][2],n.X0[2])

  # obstacle information



  nothing
end



end # module
