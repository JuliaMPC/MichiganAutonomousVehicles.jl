module AutonomousControl

using NLOptControl
using VehicleModels

include("CaseModule.jl")
using .CaseModule

export
      initializeAutonomousControl,
      autonomousControl,
      updateAutoParams!

"""
n=initializeAutonomousControl(c);
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/1/2017, Last Modified: 6/30/2017 \n
--------------------------------------------------------------------------------------\n
"""
function initializeAutonomousControl(c)
 pa=Vpara(x_min=c.m.Xlims[1],x_max=c.m.Xlims[2],y_min=c.m.Ylims[1],y_max=c.m.Ylims[2],sr_min=-0.18,sr_max=0.18);
 @unpack_Vpara pa
 XF=[c.g.x_ref, c.g.y_ref, NaN, NaN, NaN, NaN, NaN, NaN];
 XL=[x_min, y_min, NaN, NaN, psi_min, sa_min, u_min, NaN];
 XU=[x_max, y_max, NaN, NaN, psi_max, sa_max, u_max, NaN];
 #XL=[NaN,NaN, NaN, NaN, psi_min, sa_min, u_min, NaN]; #NOTE this creates problems
 #XU=[NaN,NaN, NaN, NaN, psi_max, sa_max, u_max, NaN];
 CL = [sr_min, jx_min]; CU = [sr_max, jx_max];
 n=define(numStates=8,numControls=2,X0=copy(c.m.X0),XF=XF,XL=XL,XU=XU,CL=CL,CU=CU)
 n.params = [pa];   # vehicle parameters

 # set mpc parameters
 initializeMPC!(n;FixedTp=c.m.FixedTp,PredictX0=c.m.PredictX0,tp=c.m.tp,tex=copy(c.m.tex),max_iter=c.m.mpc_max_iter);
 n.mpc.X0=[copy(c.m.X0)];
 n.mpc.plantEquations=ThreeDOFv2;
 n.mpc.modelEquations=ThreeDOFv2;

 # define tolerances
 XF_tol=[5.,5.,NaN,NaN,NaN,NaN,NaN,NaN];
 X0_tol=[0.05,0.05,0.05,0.05,0.01,0.001,0.05,0.05];
 defineTolerances!(n;X0_tol=X0_tol,XF_tol=XF_tol);

         # 1  2  3  4  5    6   7   8
names = [:x,:y,:v,:r,:psi,:sa,:ux,:ax];
descriptions = ["X (m)","Y (m)","Lateral Velocity (m/s)", "Yaw Rate (rad/s)","Yaw Angle (rad)", "Steering Angle (rad)", "Longitudinal Velocity (m/s)", "Longitudinal Acceleration (m/s^2)"];
states!(n,names,descriptions=descriptions)

        # 1    2
 names = [:sr,:jx];
 descriptions = ["Steering Rate (rad/s)","Longitudinal Jerk (m/s^3)"];
 controls!(n,names,descriptions=descriptions)

 # dynamic constrains
 dx,FZ_RL_expr,FZ_RR_expr,Fyf_linear,Fyr_linear=ThreeDOFv2_expr(n)
 dynamics!(n,dx)

 # solver settings
 SS=((:name=>:Ipopt),(:mpc_defaults=>true),(:max_cpu_time=>c.m.max_cpu_time))

 # configure problem
 configure!(n,Nck=c.m.Nck;(:integrationScheme=>:lgrExplicit),(:finalTimeDV=>true),(:solverSettings=>SS))

 # vertical tire load
 #=
 FZ_RL=NLExpr(n,FZ_RL_expr)
 FZ_RR=NLExpr(n,FZ_RR_expr)
 FZ_rl_con=@NLconstraint(n.mdl, [j=1:n.numStatePoints], 0 <= FZ_RL[j] - Fz_min)
 FZ_rr_con=@NLconstraint(n.mdl, [j=1:n.numStatePoints], 0 <= FZ_RR[j] - Fz_min)
 newConstraint!(n,FZ_rl_con,:FZ_rl_con);
 newConstraint!(n,FZ_rr_con,:FZ_rr_con);
=#
 # linear tire constraint limits
#=
 Fyf_linear=NLExpr(n,Fyf_linear)
 Fyr_linear=NLExpr(n,Fyr_linear)
 Fyf_con=@NLconstraint(n.mdl, [j=1:n.numStatePoints], Fyf_min <= Fyf_linear[j] <= Fyf_max)
 Fyr_con=@NLconstraint(n.mdl, [j=1:n.numStatePoints], Fyf_min <= Fyr_linear[j] <= Fyf_max)
 newConstraint!(n,Fyf_con,:Fyf_con);
 newConstraint!(n,Fyr_con,:Fyr_con);
=#

 # add parameters
 Q = size(c.o.A)[1]; # number of obstacles
 @NLparameter(n.mdl, a[i=1:Q] == copy(c.o.A[i]));
 @NLparameter(n.mdl, b[i=1:Q] == copy(c.o.B[i]));
 @NLparameter(n.mdl, X_0[i=1:Q] == copy(c.o.X0[i]));
 @NLparameter(n.mdl, Y_0[i=1:Q] == copy(c.o.Y0[i]));
 @NLparameter(n.mdl, speed_x[i=1:Q] == copy(c.o.s_x[i]));
 @NLparameter(n.mdl, speed_y[i=1:Q] == copy(c.o.s_y[i]));
 obs_params=[a,b,X_0,Y_0,speed_x,speed_y];

 #TODO add rest of terms in obj function
#=
#----------------------
# OBJECTIVE FUNCTION #
#----------------------
# penalize distance to goal
@NLparameter(mdl, w_goal_param == w_goal)  # temp values
@NLexpression(mdl, goal_obj, w_goal_param*((x[1] - x_ref)^2 + (y[1] - y_ref)^2)/((x[N+1] - x_ref)^2 + (y[N+1] - y_ref)^2+EP))

# penalize difference between final heading angle and angle relative to the goal
@NLparameter(mdl, w_psi_param == w_psi)  # temp values NOTE THESE ARE SO we can set them to zero!  for contraints on getting to goal we can relax those parameters as well
@NLexpression(mdl, psi_obj, w_psi_param*atan((y_ref-y[end])/(x_ref-x[end]+EP))*(atan(sin(psi[end]-psi_ref)/(cos(psi[end]-psi_ref)+EP)))^2 )

# minimizing the integral over the entire prediction horizon of the line that passes through the goal
@NLexpression(mdl, haf_obj, w_haf*sum{((sin(psi_ref)*(x[i]-x_ref)-cos(psi_ref)*(y[i]-y_ref))^2)*dt[i],i=1:N})

# define objective function
#@NLobjective(mdl, Min, aux_time[N+1] + goal_obj + psi_obj + haf_obj + ce_obj + Fz_obj )
@NLobjective(mdl, Min, aux_time[N+1] + haf_obj + ce_obj + Fz_obj )
=#
  #----------------------
  # OBJECTIVE FUNCTION #
  #----------------------
 # prevent vehicle from operating at its vertical tire load limit
# @NLexpression(n.mdl, Fz_obj, c.w.Fz*sum((tanh(-(0.5*(FzR0 + KZX*(ax[i] - v[i]*r[i])) - KZYR*((FYF[i] + FYR[i])/m) -a_t)/b_t)+tanh(-(0.5*(FzR0 + KZX*(ax[i] - v[i]*r[i])) + KZYR*((FYF[i] + FYR[i])/m)-a_t)/b_t))*dt[i],i=1:N));


#(tanh(-(0.5*(FzR0 + KZX*(ax[i] - v[i]*r[i])) - KZYR*((FYF[i] + FYR[i])/m) -a_t)/b_t)

 # penalize control effort
 ce_obj=integrate!(n,:($c.w.ce*($c.w.sa*(sa[j]^2)+$c.w.sr*(sr[j]^2)+$c.w.jx*(jx[j]^2))) )

 @NLobjective(n.mdl, Min, c.w.time*n.tf + ce_obj )

 # obstacle postion after the intial postion
 X_obs=@NLexpression(n.mdl, [j=1:Q,i=1:n.numStatePoints], X_0[j] + speed_x[j]*n.tV[i]);
 Y_obs=@NLexpression(n.mdl, [j=1:Q,i=1:n.numStatePoints], Y_0[j] + speed_y[j]*n.tV[i]);

 # constraint on position
 obs_con=@NLconstraint(n.mdl, [j=1:Q,i=1:n.numStatePoints-1], 1 <= ((n.r.x[(i+1),1]-X_obs[j,i])^2)/((a[j]+c.m.sm)^2) + ((n.r.x[(i+1),2]-Y_obs[j,i])^2)/((b[j]+c.m.sm)^2));
 newConstraint!(n,obs_con,:obs_con);

 # position parameters
 @NLparameter(n.mdl, X0_params[j=1:2]==n.X0[j]);

 # LiDAR constraint: ensure that all of the points are in the LiDAR range TODO LiDAR range constrait with x y constraints are messing it up
#  LiDAR_con=@NLconstraint(mdl, [i=1:n.numStatePoints-1], ((r.x[(i+1),1]-X0_params[1])^2+(r.x[(i+1),2]-X0_params[2])^2) <= (c.m.Lr + c.m.L_rd)^2); # not constraining the first state
#  newConstraint!(r,LiDAR_con,:LiDAR_con);

#  sm=createParallelModels(n,c,pa)

# intial optimization
 n.s.save=false; n.s.MPC=false; n.s.evalConstraints=false;
 optimize!(n);

         #  1    2          3
 n.params=[pa,obs_params,X0_params];

 # settings
 n.s.save=true; n.s.evalConstraints=true;

 # save case data TODO    case_data, obs_data = case2dfs(c);
 return n
end

"""

--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/20/2017, Last Modified: 4/11/2017 \n
--------------------------------------------------------------------------------------\n
"""
function updateAutoParams!(n)

  # vehicle position for LiDAR
  setvalue(n.params[3][1],n.X0[1])
  setvalue(n.params[3][2],n.X0[2])

  # obstacle information
 return nothing
end


end # module
