using NLOptControl
using VehicleModels
using Parameters
using DataFrames
using MAVs
# TODO
#1) could make a check if the vehicle passes  the goal
#2) add in LiDAR
#3) add in terms in obj fun
#4) switch between getting to edge of lidar range and goal
# 5) detect a crash
# 6) it is gettting a KNITRO error the first time, but not after that. maybe it is setup differently
# 7) looking at the second iteration it seems like vehicle position did not getr updated, but the vehicle did turn
c=defineCase(;(:mode=>:autoARC));
setMisc!(c;mpc_max_iter=300,tex=0.5,max_cpu_time=0.47,Ni=3,Nck=[10,8,6]);

mdl,n,r,params=initializeAutonomousControl(c);

global pa=params[1];
global s=Settings(;reset=false,save=true,MPC=true);

driveStraight!(n,pa,r,s)
for ii=2:n.mpc.max_iter
   #if ((r.dfs_plant[end][:x][end]-c.g.x_ref)^2 + (r.dfs_plant[end][:y][end]-c.g.y_ref)^2)^0.5 < 2*n.mpc.tex*r.dfs_plant[1][:ux][end] #TODO this should be tolerance in PRettyPlots || sum(getvalue(dt)) < 0.0001
   if ((r.dfs_plant[end][:x][end]-c.g.x_ref)^2 + (r.dfs_plant[end][:y][end]-c.g.y_ref)^2)^0.5 < 0.4*n.mpc.tex*r.dfs_plant[1][:ux][end] #TODO this should be tolerance in PRettyPlots || sum(getvalue(dt)) < 0.0001

      println("Goal Attained! \n"); n.mpc.goal_reached=true; break;
    end
    println("Running model for the: ",r.eval_num," time");

    updateAutoParams!(n,r,c,params);          # update model parameters

    status=autonomousControl!(mdl,n,r,s,pa);     # rerun optimization
    if r.status==:Optimal || r.status==:Suboptimal || r.status==:UserLimit
      println("Passing Optimized Signals to 3DOF Vehicle Model");
      n.mpc.t0_actual=(r.eval_num-1)*n.mpc.tex;  # external so that it can be updated easily in PathFollowing
      simPlant!(n,r,s,pa,n.X0,r.t_ctr+n.mpc.t0,r.U,n.mpc.t0_actual,r.eval_num*n.mpc.tex)
    elseif r.status==:Infeasible
      println("\n FINISH:Passing PREVIOUS Optimized Signals to 3DOF Vehicle Model \n"); break;
    else
      println("\n There status is nor Optimal or Infeaible -> create logic for this case!\n"); break;
    end
  if r.eval_num==n.mpc.max_iter
    warn(" \n The maximum number of iterations has been reached while closing the loop; consider increasing (max_iteration) \n")
  end
end

include("postProcess.jl"); 
