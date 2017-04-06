using NLOptControl
using VehicleModels
using Parameters
using DataFrames
using MAVs

c=defineCase(;(:mode=>:path));
#c=defineCase(;(:mode=>:caseStudy));

mdl,n,r,params=initializePathFollowing(c);  #TODO add driveStraight here to get a better intialization
global pa=params[1];
global s=Settings(;reset=false,save=true,simulate=true,MPC=true,format=:png);

# only happens once -> no other data for X0 currenly
#function driveStraight()
X0=c.m.X0;
r.eval_num=1;                         # update the evaluation number
updateX0(n,r,X0;(:userUpdate=>true));
mpcUpdate(n,r);                        # initialize mpc parameters before running optimization -> first execution horizon is driving straight
# assuming that it takes n.mpc.tex to solve the OCP problem -> initialize the vehicle at this point
# initially assume the vehicle is driving straight
if c.m.model==:ThreeDOFv1
  SA=0*ones(size(r.t_ctr));
  UX=c.m.UX*ones(size(r.t_ctr)); # constant speed, but this can easily be changed, just update ux_param
elseif c.m.model==:ThreeDOFv2
  SR=0*ones(size(r.t_ctr));JX=SR;
end

# 1) simulate the "actual vehicle" response and simultaneously start solving the optimization problem starting at the predicted X0
println("Passing Initial Commands to 3DOF Vehicle Model");
if c.m.model==:ThreeDOFv1
  sol=ThreeDOFv1(pa,n.X0,r.t_ctr+n.mpc.t0,SA,UX,n.mpc.t0,n.mpc.tf);
elseif c.m.model==:ThreeDOFv2
  sol = ThreeDOFv2(pa,n.X0,r.t_ctr+n.mpc.t0,SR,JX,n.mpc.t0,n.mpc.tf);
end

# 2) predict X0 (X0p) using 3DOF vehicle model (eventually the "actual model" will be different)
if c.m.model==:ThreeDOFv1
  sol=ThreeDOFv1(pa,n.X0,r.t_ctr+n.mpc.t0,SA,UX,n.mpc.t0,n.mpc.tf); n.mpc.X0p=sol(n.mpc.tf)[:];
  u=[SA];
elseif c.m.model==:ThreeDOFv2
  sol=ThreeDOFv2(pa,n.X0,r.t_ctr+n.mpc.t0,SR,JX,n.mpc.t0,n.mpc.tf); n.mpc.X0p=sol(n.mpc.tf)[:];
  u=[SR,JX];
end

# 3) update variables so everything is in sync
plant2dfs(n,r,s,u,sol); updateX0(n,r);
postProcess(n,r,s;(:Init=>true))

for ii=2:n.mpc.max_iter
    if ((r.dfs_plant[end][:x][end]-c.g.x_ref)^2 + (r.dfs_plant[end][:y][end]-c.g.y_ref)^2)^0.5 < n.mpc.tex*r.dfs_plant[1][:ux][1] # || sum(getvalue(dt)) < 0.0001
      println("Track Complete! \n"); n.mpc.goal_reached=true; break;
    end
    r.eval_num=ii;                         # update the evaluation number
    println("Running model for the: ",r.eval_num," time");
    mpcUpdate(n,r);                        # update mpc parameters before running optimization

    updatePathParams(n,r,c,params);          # update model parameters

    autonomousControl(mdl,n,r,s,params);   # rerun optimization
    if r.status[r.eval_num]==:Optimal || r.status[r.eval_num]==:Suboptimal || r.status[r.eval_num]==:UserLimit
      println("Passing Optimized Signals to 3DOF Vehicle Model");
      # 1) simulate the "actual vehicle" response and simultaneously start solving the optimization problem starting at the predicted X0
      if c.m.model==:ThreeDOFv1
        sol=ThreeDOFv1(pa,n.X0,r.t_ctr+n.mpc.t0,r.U[:,1],UX,n.mpc.t0,n.mpc.tf);
      elseif c.m.model==:ThreeDOFv2
        print("a")
        sol=ThreeDOFv2(pa,n.X0,r.t_ctr+n.mpc.t0,r.U[:,1],r.U[:,2],n.mpc.t0,n.mpc.tf);
      end
      print("b")

      if r.eval_num == 19
        cd(r.results_dir)
          dfs_X0=DataFrame();
          dfs_X0[:X0]=n.X0;
          writetable("X0.csv",dfs_X0);
          dfs_t=DataFrame();
          dfs_t[:t]=r.t_ctr+n.mpc.t0;
          writetable("t.csv",dfs_t);
          dfs_U=DataFrame();
          dfs_U[:U1]=r.U[:,1];
          dfs_U[:U2]=r.U[:,2];
          writetable("U.csv",dfs_U);
          dfs_not=DataFrame();
          dfs_not[:not_save]=sol.interp.notsaveat_idxs;
          writetable("not_save.csv",dfs_not);
        cd(r.main_dir)
      end
      # 2) "predict" X0 (X0p) using 3DOF vehicle model (eventually the "actual model" will be different)
      # this should be done as the vehicle is simulated/actually controlled
      if c.m.model==:ThreeDOFv1
        sol=ThreeDOFv1(pa,n.X0,r.t_ctr+n.mpc.t0,r.U[:,1],UX,n.mpc.t0,n.mpc.tf);
      elseif c.m.model==:ThreeDOFv2
        sol=ThreeDOFv2(pa,n.X0,r.t_ctr+n.mpc.t0,r.U[:,1],r.U[:,2],n.mpc.t0,n.mpc.tf);
      end
      print("c")

      if !(sol.t[end] == n.mpc.tf)
          print("NOT EQUAL")
      end
      print(sol.t[end])
      print(n.mpc.tf)
      if r.eval_num == 19
        cd(r.results_dir)
          dfs_sol=DataFrame();
          dfs_sol[:sol_t]=sol.t;
          writetable("sol_t.csv",dfs_sol);
          dfs_tf=DataFrame();
          dfs_tf[:tf]=n.mpc.tf;
          dfs_tf[:t0]=n.mpc.t0;
          writetable("mpc_tf.csv",dfs_tf);
          dfs_u=DataFrame();
          dfs_u[:u]=sol.u;
          writetable("sol_u.csv",dfs_u);
          dfs_not=DataFrame();
          dfs_not[:not_save]=sol.interp.notsaveat_idxs;
          writetable("not_save.csv",dfs_not);
        cd(r.main_dir)
      end

      n.mpc.X0p=sol(n.mpc.tf)[:];   # consider turning this into a function -> predict X0
      print("d")

      # 3) now we can update the actual X0 --> important not to do this before the prediction
      if c.m.model==:ThreeDOFv1
        u=[r.U[:,1]];
      elseif c.m.model==:ThreeDOFv2
        u=[r.U[:,1],r.U[:,2]];
      end
      plant2dfs(n,r,s,u,sol); updateX0(n,r);
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
