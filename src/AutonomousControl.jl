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
Date Create: 2/1/2017, Last Modified: 7/05/2017 \n
--------------------------------------------------------------------------------------\n
"""
function initializeAutonomousControl(c)
 pa=Vpara(x_min=c.m.Xlims[1],x_max=c.m.Xlims[2],y_min=c.m.Ylims[1],y_max=c.m.Ylims[2],sr_min=-0.18,sr_max=0.18);
 @unpack_Vpara pa
 XF=[c.g.x_ref, c.g.y_ref, NaN, NaN, NaN, NaN, NaN, NaN];
 XL=[x_min, y_min, NaN, NaN, psi_min, sa_min, u_min, NaN];
 XU=[x_max, y_max, NaN, NaN, psi_max, sa_max, u_max, NaN];
 CL = [sr_min, jx_min]; CU = [sr_max, jx_max];
 n=define(numStates=8,numControls=2,X0=copy(c.m.X0),XF=XF,XL=XL,XU=XU,CL=CL,CU=CU)
 n.s.tf_max=10.0;
 n.params=[pa];   # vehicle parameters

 # set mpc parameters
 initializeMPC!(n;FixedTp=c.m.FixedTp,PredictX0=c.m.PredictX0,tp=c.m.tp,tex=copy(c.m.tex),max_iter=c.m.mpc_max_iter);
 n.mpc.X0=[copy(c.m.X0)];
 n.mpc.plantEquations=ThreeDOFv2;
 n.mpc.modelEquations=ThreeDOFv2;

 # define tolerances
 X0_tol=[0.05,0.05,0.05,0.05,0.01,0.001,0.05,0.05];
 XF_tol=[c.m.sigma,c.m.sigma,NaN,NaN,NaN,NaN,NaN,NaN];
 defineTolerances!(n;X0_tol=X0_tol,XF_tol=XF_tol);

         # 1  2  3  4  5    6   7   8
 names = [:x,:y,:v,:r,:psi,:sa,:ux,:ax];
 descriptions = ["X (m)","Y (m)","Lateral Velocity (m/s)", "Yaw Rate (rad/s)","Yaw Angle (rad)", "Steering Angle (rad)", "Longitudinal Velocity (m/s)", "Longitudinal Acceleration (m/s^2)"];
 states!(n,names,descriptions=descriptions)

          # 1   2
 names = [:sr,:jx];
 descriptions = ["Steering Rate (rad/s)","Longitudinal Jerk (m/s^3)"];
 controls!(n,names,descriptions=descriptions)

 # dynamic constraints and additional constraints
 dx,con,tire_expr=ThreeDOFv2_expr(n)
 dynamics!(n,dx)
 constraints!(n,con)

 # solver settings
 solver_time = (c.m.solver==:Ipopt) ? :max_cpu_time : :maxtime_real
 SS=((:name=>c.m.solver),(:mpc_defaults=>true),(solver_time=>100.)) # let the solver have more time for the initalization

 # configure problem
 configure!(n;(Nck=c.m.Nck),(:integrationScheme=>c.m.integrationScheme),(:finalTimeDV=>true),(:solverSettings=>SS))
 x=n.r.x[:,1];y=n.r.x[:,2];psi=n.r.x[:,5]; # pointers to JuMP variables

 #################################
 # obstacle aviodance constraints
 ################################
 Q = size(c.o.A)[1]; # number of obstacles
 @NLparameter(n.mdl, a[i=1:Q] == copy(c.o.A[i]));
 @NLparameter(n.mdl, b[i=1:Q] == copy(c.o.B[i]));
 @NLparameter(n.mdl, X_0[i=1:Q] == copy(c.o.X0[i]));
 @NLparameter(n.mdl, Y_0[i=1:Q] == copy(c.o.Y0[i]));
 @NLparameter(n.mdl, speed_x[i=1:Q] == copy(c.o.s_x[i]));
 @NLparameter(n.mdl, speed_y[i=1:Q] == copy(c.o.s_y[i]));
 obs_params=[a,b,X_0,Y_0,speed_x,speed_y,Q];

 # obstacle postion after the intial postion
 X_obs=@NLexpression(n.mdl, [j=1:Q,i=1:n.numStatePoints], X_0[j] + speed_x[j]*n.tV[i]);
 Y_obs=@NLexpression(n.mdl, [j=1:Q,i=1:n.numStatePoints], Y_0[j] + speed_y[j]*n.tV[i]);

 # constraint on position
 obs_con=@NLconstraint(n.mdl, [j=1:Q,i=1:n.numStatePoints-1], 1 <= ((x[(i+1)]-X_obs[j,i])^2)/((a[j]+c.m.sm)^2) + ((y[(i+1)]-Y_obs[j,i])^2)/((b[j]+c.m.sm)^2));
 newConstraint!(n,obs_con,:obs_con);

 ####################
 # LiDAR constraints
 ###################
 # ensure that the final x and y states are near the LiDAR boundary
 @NLparameter(n.mdl, LiDAR_param_1==(c.m.Lr + c.m.L_rd)^2);
 @NLparameter(n.mdl, LiDAR_param_2==(c.m.Lr - c.m.L_rd)^2);
 LiDAR_edge_high = @NLconstraint(n.mdl,[j=1], (x[end]-x[1])^2+(y[end]-y[1])^2  <= LiDAR_param_1);
 LiDAR_edge_low = @NLconstraint(n.mdl,[j=1], (x[end]-x[1])^2+(y[end]-y[1])^2  >= LiDAR_param_2);
 newConstraint!(n,LiDAR_edge_high,:LiDAR_edge_high);
 newConstraint!(n,LiDAR_edge_low,:LiDAR_edge_low);

# constrain all state points to be within LiDAR boundary
 LiDAR_range = @NLconstraint(n.mdl, [j=1:n.numStatePoints-1], (x[j+1]-x[1])^2+(y[j+1]-y[1])^2 <= (c.m.Lr + c.m.L_rd)^2 );
 newConstraint!(n,LiDAR_range,:LiDAR_range);

 if goalRange!(n,c)   # relax LiDAR boundary constraints
    setvalue(LiDAR_param_1, 1e6)
    setvalue(LiDAR_param_2,-1e6)
else                  # relax constraints on the final x and y position
    for st=1:2;for k in 1:2; setRHS(n.r.xf_con[st,k], 1e6); end; end
end
 LiDAR_params=[LiDAR_param_1,LiDAR_param_2]
 #####################
 # objective function
 ####################
 # parameters
 if goalRange!(n,c)
  println("\n goal in range")
  @NLparameter(n.mdl, w_goal_param == 0.0)
  @NLparameter(n.mdl, w_psi_param == 0.0)
 else
  @NLparameter(n.mdl, w_goal_param == c.w.goal)
  @NLparameter(n.mdl, w_psi_param == c.w.psi)
 end
 obj_params=[w_goal_param,w_psi_param]

 # penalize distance to goal
 goal_obj=@NLexpression(n.mdl,w_goal_param*((x[end] - c.g.x_ref)^2 + (y[end] - c.g.y_ref)^2)/((x[1] - c.g.x_ref)^2 + (y[1] - c.g.y_ref)^2 + EP))

 # penalize difference between final heading angle and angle relative to the goal NOTE currently this is broken becasue atan2() is not available
 #psi_frg=@NLexpression(n.mdl,asin(c.g.y_ref-y[end])/(acos(c.g.x_ref-x[end]) + EP) )
 #psi_obj=@NLexpression(n.mdl,w_psi_param*(asin(sin(psi[end] - psi_frg))/(acos(cos(psi[end] - psi_frg)) + EP) )^2 )
 psi_obj=0;
 # psi_obj=@NLexpression(n.mdl,w_psi_param*(asin(sin(psi[end] - asin(c.g.y_ref-y[end])/acos(c.g.x_ref-x[end])))/(acos(cos(psi[end] - asin(c.g.y_ref-y[end])/acos(c.g.x_ref-x[end]))) + EP) )^2 )

 # soft constraints on vertical tire load
 tire_obj=integrate!(n,tire_expr);

 # minimizing the integral over the entire prediction horizon of the line that passes through the goal
 #haf_obj=integrate!(n,:( $c.w.haf*( sin($c.g.psi_ref)*(x[j]-$c.g.x_ref) - cos($c.g.psi_ref)*(y[j]-$c.g.y_ref) )^2 ) )
 haf_obj=0;
 # penalize control effort
 ce_obj=integrate!(n,:($c.w.ce*($c.w.sa*(sa[j]^2)+$c.w.sr*(sr[j]^2)+$c.w.jx*(jx[j]^2))) )

 @NLobjective(n.mdl, Min, goal_obj + psi_obj + c.w.Fz*tire_obj + haf_obj + c.w.time*n.tf + ce_obj )

 #########################
 # intial optimization (s)
 ########################
 n.s.save=false; n.s.MPC=false; n.s.evalConstraints=false;
 if n.s.save
  warn("saving initial optimization results where functions where cashed!")
 end
 for k in 1:1
  optimize!(n);
  if n.r.status==:Optimal; break; end
 end

 # modifify the maximum solver time
 SS=Dict((:name=>c.m.solver),(:mpc_defaults=>true),(solver_time=>c.m.max_cpu_time))
 defineSolver!(n,SS)

         #  1      2          3          4
 n.params=[pa,obs_params,LiDAR_params,obj_params];

 n.s.save=true; # settings

 #n.s.evalConstraints=true

 return n
end

"""

--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/20/2017, Last Modified: 7/05/2017 \n
--------------------------------------------------------------------------------------\n
"""
function updateAutoParams!(n,c)

  # obstacle information-> only show if it is in range at the start TODO
  goal_in_range = goalRange!(n,c)
  if goal_in_range # TODO make a flag that indicates this switch has been flipped
    println("goal is in range")

    # enforce final state constraints on x and y position
    for st=1:2;
      setRHS(n.r.xf_con[st,1], +(n.XF[st]+n.XF_tol[st]));
      setRHS(n.r.xf_con[st,2], -(n.XF[st]-n.XF_tol[st]));
    end

    # relax LiDAR constraints
    setvalue(n.params[3][1], 1e6)
    setvalue(n.params[3][2],-1e6)

    # remove terms in cost function
    setvalue(n.params[4][1],0.0)
    setvalue(n.params[4][2],0.0)
  end
  #NOTE assuming it is not going in and out of range

 return goal_in_range
end

"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 7/04/2017, Last Modified: 7/04/2017 \n
--------------------------------------------------------------------------------------\n
"""
function goalRange!(n,c)
 return ( (n.mpc.X0[end][1] - c.g.x_ref)^2 + (n.mpc.X0[end][2] - c.g.y_ref)^2 )^0.5 < c.m.Lr
end


end # module
