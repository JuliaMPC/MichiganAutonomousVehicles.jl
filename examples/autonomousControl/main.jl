using NLOptControl
using VehicleModels
using JuMP
using DataFrames
using Parameters

using MAVs

c=defineCase(;(:mode=>:auto));

mdl,n,r,params=initializeAutonomousControl(c);  #TODO add driveStraight here to get a better intialization
global pa=params[1];
global s=Settings(;reset=false,save=true,simulate=true,MPC=true,format=:png);

# only happens once -> no other data for X0 currenly
#function driveStraight()
@unpack_Vpara pa; X0=[x0_,y0_,v0_,r0_,psi0_,sa0_,u0_,ax0_];
r.eval_num=1;                         # update the evaluation number
updateX0(n,r,X0;(:userUpdate=>true));
mpcUpdate(n,r);                        # initialize mpc parameters before running optimization -> first execution horizon is driving straight
# assuming that it takes n.mpc.tex to solve the OCP problem -> initialize the vehicle at this point
# initially assume the vehicle is driving straight
SR=0*ones(size(r.t_ctr));JX=SR;

# 1) simulate the "actual vehicle" response and simultaneously start solving the optimization problem starting at the predicted X0
println("Passing Initial Commands to 3DOF Vehicle Model");
sol = ThreeDOFv2(pa,n.X0,r.t_ctr+n.mpc.t0,SR,JX,n.mpc.t0,n.mpc.tf);

# 2) predict X0 (X0p) using 3DOF vehicle model (eventually the "actual model" will be different)
sol = ThreeDOFv2(pa,n.X0,r.t_ctr+n.mpc.t0,SR,JX,n.mpc.t0,n.mpc.tf); n.mpc.X0p=sol(n.mpc.tf)[:];

# 3) update variables so everything is in sync
u=[SR,JX]; plant2dfs(n,r,s,u,sol); updateX0(n,r);
postProcess(n,r,s;(:Init=>true))

for ii=2:n.mpc.max_iter
    if ((r.dfs_plant[end][:x][end]-c.g.x_ref)^2 + (r.dfs_plant[end][:y][end]-c.g.y_ref)^2)^0.5 < n.mpc.tex*r.dfs_plant[1][:ux][1] # || sum(getvalue(dt)) < 0.0001
      println("Goal Attained! \n"); n.mpc.goal_reached=true; break;
    end
    r.eval_num=ii;                         # update the evaluation number
    println("Running model for the: ",r.eval_num," time");
    mpcUpdate(n,r);                        # update mpc parameters before running optimization

    autonomousControl(mdl,n,r,s,params);   # rerun optimization
    if r.status[r.eval_num]==:Optimal || r.status[r.eval_num] ==:Suboptimal || r.status[r.eval_num] ==:UserLimit
      println("Passing Optimized Signals to 3DOF Vehicle Model");
      # 1) simulate the "actual vehicle" response and simultaneously start solving the optimization problem starting at the predicted X0
      sol = ThreeDOFv2(pa,n.X0,r.t_ctr+n.mpc.t0,r.U[:,1],r.U[:,2],n.mpc.t0,n.mpc.tf);

      # 2) "predict" X0 (X0p) using 3DOF vehicle model (eventually the "actual model" will be different)
      # this should be done as the vehicle is simulated/actually controlled
      sol = ThreeDOFv2(pa,n.X0,r.t_ctr+n.mpc.t0,r.U[:,1],r.U[:,2],n.mpc.t0,n.mpc.tf);
      n.mpc.X0p=sol(n.mpc.tf)[:];   # consider turning this into a function -> predict X0

      # 3) now we can update the actual X0 --> important not to do this before the prediction
      u=[r.U[:,1],r.U[:,2]]; plant2dfs(n,r,s,u,sol); updateX0(n,r);
    elseif r.status[r.eval_num]==:Infeasible
      println("FINISH:Passing PREVIOUS Optimized Signals to 3DOF Vehicle Model"); break;
    else
      println("There status is nor Optimal or Infeaible -> create logic for this case!"); break;
    end
  if r.eval_num==n.mpc.max_iter
    warn("The maximum number of iterations has been reached while closing the loop; consider increasing (max_iteration)")
  end

end

if s.simulate
  include("postProcess.jl")
end
