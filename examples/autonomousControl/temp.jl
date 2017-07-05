using NLOptControl
using VehicleModels
using Parameters
using DataFrames
using MAVs

c=defineCase(;(:mode=>:autoARC));
setMisc!(c;mpc_max_iter=50,tex=0.5,max_cpu_time=0.466,Nck=[10,8,6]);
pa=Vpara(x_min=c.m.Xlims[1],x_max=c.m.Xlims[2],y_min=c.m.Ylims[1],y_max=c.m.Ylims[2],sr_min=-0.18,sr_max=0.18);
@unpack_Vpara pa
XF=[c.g.x_ref, c.g.y_ref, NaN, NaN, NaN, NaN, NaN, NaN];
XL=[x_min, y_min, NaN, NaN, psi_min, sa_min, u_min, NaN];
XU=[x_max, y_max, NaN, NaN, psi_max, sa_max, u_max, NaN];
#XL=[NaN,NaN, NaN, NaN, psi_min, sa_min, u_min, NaN]; #NOTE this creates problems
#XU=[NaN,NaN, NaN, NaN, psi_max, sa_max, u_max, NaN];
CL = [sr_min, jx_min]; CU = [sr_max, jx_max];
n=define(numStates=8,numControls=2,X0=copy(c.m.X0),XF=XF,XL=XL,XU=XU,CL=CL,CU=CU)
n.s.tf_max=15.0;
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

         # 1   2
names = [:sr,:jx];
descriptions = ["Steering Rate (rad/s)","Longitudinal Jerk (m/s^3)"];
controls!(n,names,descriptions=descriptions)

# dynamic constraints and additional constraints
dx,con,tire_expr=ThreeDOFv2_expr(n)
dynamics!(n,dx)
constraints!(n,con)

# solver settings
#SS=((:name=>:Ipopt),(:mpc_defaults=>true),(:max_cpu_time=>c.m.max_cpu_time))
SS=((:name=>:Ipopt),(:mpc_defaults=>true),(:max_cpu_time=>100.)) # for the initalization let the solver have as much time as it wants

# configure problem
configure!(n,Nck=c.m.Nck;(:integrationScheme=>:lgrExplicit),(:finalTimeDV=>true),(:solverSettings=>SS))

# obstacle aviodance constraints
Q = size(c.o.A)[1]; # number of obstacles
@NLparameter(n.mdl, a[i=1:Q] == copy(c.o.A[i]));
@NLparameter(n.mdl, b[i=1:Q] == copy(c.o.B[i]));
@NLparameter(n.mdl, X_0[i=1:Q] == copy(c.o.X0[i]));
@NLparameter(n.mdl, Y_0[i=1:Q] == copy(c.o.Y0[i]));
@NLparameter(n.mdl, speed_x[i=1:Q] == copy(c.o.s_x[i]));
@NLparameter(n.mdl, speed_y[i=1:Q] == copy(c.o.s_y[i]));
obs_params=[a,b,X_0,Y_0,speed_x,speed_y];

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

#----------------------
# OBJECTIVE FUNCTION #
#----------------------
# parameters
if ( (n.mpc.X0[end][1] - c.g.x_ref)^2 + (n.mpc.X0[end][2] - c.g.y_ref)^2 )^0.5 < c.m.Lr
 println("\n goal in range")
 @NLparameter(n.mdl, w_goal_param == 0.0)
 @NLparameter(n.mdl, w_psi_param == 0.0)
else
 @NLparameter(n.mdl, w_goal_param == c.w.goal)
 @NLparameter(n.mdl, w_psi_param == c.w.psi)
end
obj_params=[w_goal_param,w_psi_param]

# pointers
x=n.r.x[:,1];y=n.r.x[:,2];psi=n.r.x[:,5];

# penalize distance to goal
@NLexpression(n.mdl, goal_obj, w_goal_param*((x[1] - c.g.x_ref)^2 + (y[1] - c.g.y_ref)^2)/((x[end] - c.g.x_ref)^2 + (y[end] - c.g.y_ref)^2+EP))

# penalize difference between final heading angle and angle relative to the goal
@NLexpression(n.mdl, psi_obj, w_psi_param*atan((c.g.y_ref-y[end])/(c.g.x_ref-x[end]+EP))*(atan(sin(psi[end]-c.g.psi_ref)/(cos(psi[end]-c.g.psi_ref)+EP)))^2 )

# soft constraints on vertical tire load
tire_obj=integrate!(n,tire_expr);

# minimizing the integral over the entire prediction horizon of the line that passes through the goal
haf_obj=integrate!(n,:( ( sin($c.g.psi_ref)*(x[j]-$c.g.x_ref) - cos($c.g.psi_ref)*(y[j]-$c.g.y_ref) )^2 ) )

# penalize control effort
ce_obj=integrate!(n,:($c.w.ce*($c.w.sa*(sa[j]^2)+$c.w.sr*(sr[j]^2)+$c.w.jx*(jx[j]^2))) )

@NLobjective(n.mdl, Min, c.w.time*n.tf + ce_obj + c.w.Fz*tire_obj)

#  sm=createParallelModels(n,c,pa)

# intial optimization
n.s.save=false; n.s.MPC=false; n.s.evalConstraints=false;
for i in 1:5
 optimize!(n);
 if n.r.status==:Optimal; break; end
end

# modifify the maximum solver time
SS=Dict((:name=>:Ipopt),(:mpc_defaults=>true),(:max_cpu_time=>c.m.max_cpu_time))
defineSolver!(n,SS)

        #  1    2          3          4
n.params=[pa,obs_params,X0_params,obj_params];

# settings
n.s.save=true;
