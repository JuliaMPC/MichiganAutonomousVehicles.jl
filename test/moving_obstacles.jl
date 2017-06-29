

# closed loop moving obstacle avoidance when goal is within lidar range
function goal_in_lidar()
  c=defineCase(;(:mode=>:autoARC));
  setMisc!(c;mpc_max_iter=300,tex=0.5,max_cpu_time=0.46,Ni=3,Nck=[10,8,6]);

  n=initializeAutonomousControl(c);
  n.s.evalConstraints=false;
  driveStraight!(n)
  for ii=2:n.mpc.max_iter
     if ((n.r.dfs_plant[end][:x][end]-c.g.x_ref)^2 + (n.r.dfs_plant[end][:y][end]-c.g.y_ref)^2)^0.5 < 1.8*n.XF_tol[1]
        n.mpc.goal_reached=true; break;
      end
      updateAutoParams!(n);                        # update model parameters
      status=autonomousControl!(n);                # rerun optimization
      n.mpc.t0_actual=(n.r.eval_num-1)*n.mpc.tex;  # external so that it can be updated easily in PathFollowing
      simPlant!(n)
  end
  return n.mpc.goal_reached
end
