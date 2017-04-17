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

c=defineCase(;(:mode=>:auto));
c.o=defineObstacles(:auto2)

setMisc!(c;sm=2.6,Xlims=[100.,250.],Ylims=[-1., 140.],tex=0.5,max_cpu_time=.44,Ni=4,Nck=[12,10,8,6],mpc_max_iter=200,PredictX0=true,FixedTp=true);
c.m.X0=[200.0, 0.0, 0.0, 0.0, pi/2,0.0,20.0,0.0];

# TEST # 1
#c.o.A=[5.,10.,2.];
#c.o.B=[5.,10.,2.];
#c.o.s_x=[-3.5,2.,0.];
#c.o.s_y=[0.,4.,8.];
#c.o.X0=[205.,180.,200.];
#c.o.Y0=[50.,75.,30.];

A=[2.5 1.5];      # small car
B=[3.3 1.9];      # HUMMVEEs
C=[9.8/2 3.65/2]; # tanks
random = [0 0.6 1 0.25 0.6 0.5495 0.4852 0.8905 0.7990 1];

c.o.X0=[];
c.o.Y0=[];
c.o.A=[];
c.o.B=[];
c.o.s_x=[];
c.o.s_y=[];

# obstacles moving HUMMVEE
#=
for k = 1:3
   push!(c.o.X0,225-k*65);
   push!(c.o.Y0,30+k*15);
   push!(c.o.A,B[1]);
   push!(c.o.B,B[2]);
   push!(c.o.s_x,8+4*random[k]);
   push!(c.o.s_y,0);
end
# obstacles moving small car 1
for k = 1:3
   push!(c.o.X0,175+k*48);
   push!(c.o.Y0,38+k*10);
   push!(c.o.A,A[1]);
   push!(c.o.B,A[2]);
   push!(c.o.s_x,8+4*random[k]);
   push!(c.o.s_y,0);
end
=#

push!(c.o.X0,225);
push!(c.o.Y0,25);
push!(c.o.A,B[1]);
push!(c.o.B,B[2]);
push!(c.o.s_x,-5);
push!(c.o.s_y,0);

push!(c.o.X0,225);
push!(c.o.Y0,30);
push!(c.o.A,B[1]);
push!(c.o.B,B[2]);
push!(c.o.s_x,-5.5);
push!(c.o.s_y,0);

push!(c.o.X0,225);
push!(c.o.Y0,45);
push!(c.o.A,B[1]);
push!(c.o.B,B[2]);
push!(c.o.s_x,-6);
push!(c.o.s_y,0);

push!(c.o.X0,170);
push!(c.o.Y0,60);
push!(c.o.A,C[1]);
push!(c.o.B,C[2]);
push!(c.o.s_x,6);
push!(c.o.s_y,0);

#push!(c.o.X0,170);
#push!(c.o.Y0,75);
#push!(c.o.A,A[1]);
#push!(c.o.B,A[2]);
#push!(c.o.s_x,4);
#push!(c.o.s_y,0);

push!(c.o.X0,225);
push!(c.o.Y0,75);
push!(c.o.A,A[1]);
push!(c.o.B,A[2]);
push!(c.o.s_x,-5);
push!(c.o.s_y,0);

push!(c.o.X0,165);
push!(c.o.Y0,95);
push!(c.o.A,C[1]);
push!(c.o.B,C[2]);
push!(c.o.s_x,4);
push!(c.o.s_y,0);


push!(c.o.X0,200);
push!(c.o.Y0,50);
push!(c.o.A,C[2]);
push!(c.o.B,C[1]);
push!(c.o.s_x,0);
push!(c.o.s_y,7);

#=
%
%     % obstacles moving small car 1
%     for k = 1:3
%         obstacle_position{end+1} = [175+k*28;38];
%         obstacle_size{end+1} = A;
%         obstacle_movement{end+1} = 'x_line';
%         obstacle_speed{end+1} = -9-4*random(k);
%         obstacle_r{end+1} = 0;
%         obstacle_N{end+1} = 2;
%     end
%
%     % obstacles moving small car 2
%     for k = 1:3
%         obstacle_position{end+1} = [225-k*29;52];
%         obstacle_size{end+1} = A;
%         obstacle_movement{end+1} = 'x_line';
%         obstacle_speed{end+1} = 10+3*random(k);
%         obstacle_r{end+1} = 0;
%         obstacle_N{end+1} = 2;
%     end
%
%     % obstacles moving TANKS
%     for k = 1:4
%         obstacle_position{end+1} = [175+k*36;64];
%         obstacle_size{end+1} = C;
%         obstacle_movement{end+1} = 'x_line';
%         obstacle_speed{end+1} = -4 - 2*random(k);
%         obstacle_r{end+1} = 0;
%         obstacle_N{end+1} = 2;
%     end
%
%     % obstacles moving TANKS
%     for k = 1:4
%         obstacle_position{end+1} = [225-k*38;75];
%         obstacle_size{end+1} = C;
%         obstacle_movement{end+1} = 'x_line';
%         obstacle_speed{end+1} = 4 + 3.5*random(k);
%         obstacle_r{end+1} = 0;
%         obstacle_N{endt+1} = 2;
%     end
=#

mdl,n,r,params=initializeAutonomousControl(c);

global pa=params[1];
global s=Settings(;reset=false,save=true,simulate=true,MPC=true,format=:png);

driveStraight!(n,pa,r,s)
for ii=2:n.mpc.max_iter
   if ((r.dfs_plant[end][:x][end]-c.g.x_ref)^2 + (r.dfs_plant[end][:y][end]-c.g.y_ref)^2)^0.5 < 2*n.mpc.tex*r.dfs_plant[1][:ux][end] #TODO this should be tollerance in PRettyPlots || sum(getvalue(dt)) < 0.0001
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
#if s.simulate; include("postProcess.jl"); end
