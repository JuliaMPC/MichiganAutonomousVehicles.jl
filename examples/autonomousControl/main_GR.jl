using NLOptControl
using VehicleModels
using Parameters
using DataFrames
using MAVs
#TODO write some code to run this in parallel! boom.
c=defineCase(;(:mode=>:auto));
error("update this")

mdl,n,r,params=initializeAutonomousControl(c);
global pa=params[1];
global s=Settings(;reset=false,save=true,simulate=true,MPC=true,format=:png);

initializeMPC(n,r,c.m.X0;(:PredictX0=>c.m.PredictX0))

# 1) predict X0 using 3DOF vehicle model
predictX0(n,pa,r;(:fixedTp=>true))

# 2) simulate the "actual vehicle" response
println("Passing Initial Commands to 3DOF Vehicle Model");
r.U=0*Matrix{Float64}(n.numControlPoints,n.numControls); # assume the vehicle is driving straight
simPlant(n,r,s,pa,n.X0,r.t_ctr+n.mpc.t0,r.U,n.mpc.t0,n.mpc.tf)

postProcess(n,r,s;(:Init=>true));

for ii=2:n.mpc.max_iter
    if ((r.dfs_plant[end][:x][end]-c.g.x_ref)^2 + (r.dfs_plant[end][:y][end]-c.g.y_ref)^2)^0.5 < n.mpc.tex*r.dfs_plant[1][:ux][1] # || sum(getvalue(dt)) < 0.0001
      println("Goal Attained! \n"); n.mpc.goal_reached=true; break;
    end
    mpcUpdate(n,r);                       # update mpc parameters before running optimization
    println("Running model for the: ",r.eval_num+1," time");

    #TODO update this -> LiDAR
    updatePathParams(n,r,c,params);          # update model parameters

    autonomousControl(mdl,n,r,s,params);     # rerun optimization
    if r.status[r.eval_num]==:Optimal || r.status[r.eval_num]==:Suboptimal || r.status[r.eval_num]==:UserLimit
      # 1) "predict" X0 using 3DOF vehicle model
      predictX0(n,pa,r)

      # 2) simulate the "actual vehicle" response and "simultaneously" start solving the optimization problem starting at the predicted X0
      println("Passing Optimized Signals to 3DOF Vehicle Model");
      simPlant(n,r,s,pa,n.X0,r.t_ctr+n.mpc.t0,r.U,n.mpc.t0,n.mpc.tf)

    elseif r.status[r.eval_num]==:Infeasible
      println("\n FINISH:Passing PREVIOUS Optimized Signals to 3DOF Vehicle Model \n"); break;
    else
      println("\n There status is nor Optimal or Infeaible -> create logic for this case!\n"); break;
    end
  if r.eval_num==n.mpc.max_iter
    warn(" \n The maximum number of iterations has been reached while closing the loop; consider increasing (max_iteration) \n")
  end

end

#using PrettyPlots
#s=Settings(;save=true,MPC=false,simulate=false,format=:png);
#pSimPath(n,r,s,c,r.eval_num)
if s.simulate; include("postProcess.jl"); end
