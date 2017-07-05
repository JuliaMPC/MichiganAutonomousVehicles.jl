using NLOptControl
using VehicleModels
using Parameters
using DataFrames
using MAVs

# TODO
#1) could make a check if the vehicle passes  the goal
#2) add in LiDAR
#4) switch between getting to edge of lidar range and goal
# 5) detect a crash
# 6) it is gettting a KNITRO error the first time, but not after that. maybe it is setup differently
# 7) looking at the second iteration it seems like vehicle position did not getr updated, but the vehicle did turn
c=defineCase(;(:mode=>:auto));
setMisc!(c;mpc_max_iter=30,tex=0.5,max_cpu_time=0.466,Nck=[10,8,6]);

n=initializeAutonomousControl(c);
n.s.evalConstraints=true
driveStraight!(n)
for ii=2:n.mpc.max_iter
   if ((n.r.dfs_plant[end][:x][end]-c.g.x_ref)^2 + (n.r.dfs_plant[end][:y][end]-c.g.y_ref)^2)^0.5 < 1.8*n.XF_tol[1]
      println("Goal Attained! \n"); n.mpc.goal_reached=true; break;
    end
    println("Running model for the: ",n.r.eval_num," time");
    updateAutoParams!(n,c);             # update model parameters
    status=autonomousControl!(n);     # rerun optimization
    n.mpc.t0_actual=(n.r.eval_num-1)*n.mpc.tex;  # external so that it can be updated easily in PathFollowing
    simPlant!(n) #TODO make sure we are passing the correct control signals
    if n.r.status==:Optimal || n.r.status==:Suboptimal || n.r.status==:UserLimit
      println("Passing Optimized Signals to 3DOF Vehicle Model");
    elseif n.r.status==:Infeasible
      println("\n FINISH:Pass PREVIOUSLY Optimized Signals to 3DOF Vehicle Model \n"); break;
    else
      println("\n There status is nor Optimal or Infeaible -> create logic for this case!\n"); break;
    end
  if n.r.eval_num==n.mpc.max_iter
    warn(" \n The maximum number of iterations has been reached while closing the loop; consider increasing (max_iteration) \n")
  end
end

include("postProcess.jl");
