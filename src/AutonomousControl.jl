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
      autonomousControl

"""
mdl,n,r,params = initializeAutonomousControl(c);
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/1/2017, Last Modified: 3/28/2017 \n
--------------------------------------------------------------------------------------\n
"""
function initializeAutonomousControl(c)
 pa=Vpara(x_min=0.0,x_max=400.,y_min=-0.01,y_max=400.);
 n=NLOpt(); @unpack_Vpara pa
 X0=[  x0_,           y0_, v0_, r0_,   psi0_,   sa0_,  u0_, ax0_];
 XF=[c.g.x_ref, c.g.y_ref, NaN, NaN,     NaN,    NaN,  NaN,  NaN];
 XL=[x_min,         y_min, NaN, NaN, psi_min, sa_min, u_min, NaN];
 XU=[x_max,         y_max, NaN, NaN, psi_max, sa_max, u_max, NaN];
 CL = [sr_min, jx_min]; CU = [sr_max, jx_max];
 n = define(n,stateEquations=ThreeDOFv2,numStates=8,numControls=2,X0=X0,XF=XF,XL=XL,XU=XU,CL=CL,CU=CU,tf_max=40.0)
 n = configure(n,Ni=c.m.Ni,Nck=c.m.Nck;(:integrationMethod => :ps),(:integrationScheme => :lgrExplicit),(:finalTimeDV => true))
 mpcParams(n,c);
 mdl=defineSolver(n,c);

         # 1  2  3  4  5    6   7   8
 names = [:x,:y,:v,:r,:psi,:sa,:ux,:ax];
 descriptions = ["X (m)","Y (m)","Lateral Velocity (m/s)", "Yaw Rate (rad/s)","Yaw Angle (rad)", "Steering Angle (rad)", "Longitudinal Velocity (m/s)", "Longitudinal Acceleration (m/s^2)"];
 stateNames(n,names,descriptions)
         # 1    2
 names = [:sr,:jx];
 descriptions = ["Steering Rate (rad/s)","Longitudinal Jerk (m/s^3)"];
 controlNames(n,names,descriptions)
 params = [pa];   # vehicle parameters

 # define tolerances
 XF_tol=[0.5,0.5,NaN,NaN,NaN,NaN,0.5,NaN];
 X0_tol=[0.05,0.05,0.05,0.05,0.01,0.001,0.05,0.05];
 defineTolerances(n;X0_tol=X0_tol,XF_tol=XF_tol);
 s=Settings(;save=false,MPC=true);
 n,r=OCPdef(mdl,n,s,params);
 sr_obj=integrate(mdl,n,r.u[:,1];C=c.w.sr,(:variable=>:control),(:integrand=>:squared))
 @NLobjective(mdl, Min, n.tf + sr_obj)

 # obstacles
 Q = size(c.o.A)[1]; # number of obstacles
 @NLparameter(mdl, a[i=1:Q] == c.o.A[i]);
 @NLparameter(mdl, b[i=1:Q] == c.o.B[i]);
 @NLparameter(mdl, X_0[i=1:Q] == c.o.X0[i]);
 @NLparameter(mdl, Y_0[i=1:Q] == c.o.Y0[i]);
 @NLparameter(mdl, speed_x[i=1:Q] == c.o.s_x[i]);
 @NLparameter(mdl, speed_y[i=1:Q] == c.o.s_y[i]);

 # obstacle postion after the intial postion
 X_obs=@NLexpression(mdl, [j=1:Q,i=1:n.numStatePoints], X_0[j] + speed_x[j]*n.tV[i]);
 Y_obs=@NLexpression(mdl, [j=1:Q,i=1:n.numStatePoints], Y_0[j] + speed_y[j]*n.tV[i]);

 # constraint on position
 obs_con=@NLconstraint(mdl, [j=1:Q,i=1:n.numStatePoints-1], 1 <= ((r.x[(i+1),1]-X_obs[j,i])^2)/((a[j]+c.m.sm)^2) + ((r.x[(i+1),2]-Y_obs[j,i])^2)/((b[j]+c.m.sm)^2));
 newConstraint(r,obs_con,:obs_con);

 # ensure that all of the points are in the LiDAR range TODO update variables here  -> also add back Lrd
 #LiDAR_con=@NLconstraint(mdl, [i=1:n.numStatePoints-1], ((r.x[(i+1),1]-x0_)^2+(r.x[(i+1),2]-y0_)^2) <= (c.m.Lr + L_Rd)^2); # not constraining the first state
 #newConstraint(r,LiDAR_con,:LiDAR_con);

#  sm=createParallelModels(n,c,pa)

 optimize(mdl,n,r,s);

 # save case data TODO    case_data, obs_data = case2dfs(c);
 return mdl, n, r, params
end


end # module
