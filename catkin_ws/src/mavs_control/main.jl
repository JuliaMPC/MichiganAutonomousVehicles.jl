#!/usr/bin/env julia
using RobotOS
@rosimport geometry_msgs.msg: Point, Pose, Pose2D, PoseStamped, Vector3, Twist
@rosimport std_srvs.srv: Empty, SetBool
@rosimport nav_msgs.srv.GetPlan
@rosimport gazebo_msgs.msg: ModelState, ModelStates
@rosimport gazebo_msgs.srv: SetModelState, GetModelState, GetWorldProperties

rostypegen()
using geometry_msgs.msg
using std_srvs.srv
using nav_msgs.srv
using nav_msgs.srv.GetPlan
using gazebo_msgs.srv
using gazebo_msgs.msg

using NLOptControl
using VehicleModels
using DataFrames
using MAVs
using KNITRO


type ExternalModel  # communication
  s1
  s2
  status   # to pass result of optimization to Matlab
  sa_sample
  runJulia # a Bool (in Int form) to indicate to run julia comming from Matlab
  numObs   # number of obstacles
  SA       # last two drivers steering angle
  SA_first # first order approximation of drivers commands
  UX       # vehicle speed
  X0       # vehicle states
  Xobs
  Yobs
  A
  B
  t0  # current time
  t_opt
  wsa
  feasible # a bool for the feasibility of the current driver commands
  infeasible_counter
  infeasible_counter_max
  AL
  AU
  dt  # this is the time step in simulink
end

function ExternalModel()
  ExternalModel(Any,
                Any,
                1.0,
                [],
                1,
                3,
                [0.0,0.0],
                [],
                10.0,
                [],
                [],
                [],
                [],
                [],
                0.0,
                0.0,
                1.0,
                false,
                100,
                10,
                [],
                [],
                0.01)
end

"""

--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 4/6/2017, Last Modified: 7/31/2017 \n
--------------------------------------------------------------------------------------\n
"""
function getPlantData!(n,params,x,c,r)
  MsgString = recv(x.s1);L=19;
  MsgIn = zeros(L);
  allidx = find(MsgString->MsgString == 0x20,MsgString);
  allidx = [0;allidx];
  for i in 1:L  #length(allidx)-1
    MsgIn[i] = parse(Float64,String(copy(MsgString[allidx[i]+1:allidx[i+1]-1])));
  end
  x.X0=zeros(n.numStates);
  x.X0[1:7]=MsgIn[1:7];              # global X, global y, lateral speed v, yaw rate r, yaw angle psi
  x.SA=zeros(2);
  x.SA=[copy(x.X0[6]),MsgIn[8]];     # Vehicle steering angle and previous state of steering angle
  x.UX=copy(x.X0[7]);

  N=n.numStates+1;
  x.Xobs=MsgIn[N:N+x.numObs-1];               # ObsX
  x.Yobs=MsgIn[N+x.numObs:N+2*x.numObs-1];    # ObsY
  x.A= MsgIn[N+2*x.numObs:N+3*x.numObs-1];    # ObsR
  x.B=x.A;
  x.runJulia=MsgIn[N+3*x.numObs];
  x.t0=MsgIn[N+3*x.numObs+1];
  n.mpc.t0_actual=copy(x.t0);

  # update drivers steering rate
  SR=copy((x.SA[1]-x.SA[2])/x.dt);
  if SR>0.086;SR=0.086;elseif SR<-0.086;SR=0.086;end # saturate SR
  setvalue(params[2][3],SR)

  # update obstacle feild
#  for i in 1:length(x.A)
  i=1;
  setvalue(params[3][1][i],x.A[i]);
  setvalue(params[3][2][i],x.B[i]);
  setvalue(params[3][3][i],x.Xobs[i]);
  setvalue(params[3][4][i],x.Yobs[i]);
#  end

  # update case for plotting
  if r.eval_num==1; c.o.A=[];c.o.B=[];c.o.X0=[];c.o.Y0=[]end # reset

  if isempty(c.o.X0) # update obstacle data
    push!(c.o.A,copy(x.A[1]));
    push!(c.o.B,copy(x.B[1]));
    push!(c.o.X0,copy(x.Xobs[1]));
    push!(c.o.Y0,copy(x.Yobs[1]));
    push!(c.o.s_y,0.0);
    push!(c.o.s_x,0.0);
  elseif  (x.Xobs!=c.o.X0[end]) && (x.Yobs!=c.o.Y0[end])
    push!(c.o.A,copy(x.A[1]));
    push!(c.o.B,copy(x.B[1]));
    push!(c.o.X0,copy(x.Xobs[1]));
    push!(c.o.Y0,copy(x.Yobs[1]));
    push!(c.o.s_y,0.0);
    push!(c.o.s_x,0.0);
  end

  x.SA_first=zeros(n.numStatePoints,);
  for jj in 1:n.numStatePoints
    x.SA_first[jj]=copy(x.SA[1]) + jj*copy(x.SA[1]-x.SA[2]);  # first order approximation of SA
  end

  if c.m.followDriver # first order
    for jj in 1:n.numStatePoints
      setvalue(params[2][2][jj], x.SA_first[jj])
    end
  end
  return nothing
end


# TODO
#1) could make a check if the vehicle passes  the goal
# 5) detect a crash
# 7) looking at the second iteration it seems like vehicle position did not get updated, but the vehicle did turn
c=defineCase(;(:mode=>:autoGazebo));
setMisc!(c;mpc_max_iter=60,tex=0.5,max_cpu_time=0.466,Nck=[10,8,6],solver=:KNITRO);

n=initializeAutonomousControl(c);
n.s.evalConstraints=true
driveStraight!(n)
for ii=2:n.mpc.max_iter
   if ((n.r.dfs_plant[end][:x][end]-c.g.x_ref)^2 + (n.r.dfs_plant[end][:y][end]-c.g.y_ref)^2)^0.5 < 2*n.XF_tol[1]
      println("Goal Attained! \n"); n.mpc.goal_reached=true; break;
    end
    println("Running model for the: ",n.r.eval_num," time");
    updateAutoParams!(n,c);             # update model parameters
    status=autonomousControl!(n);      # rerun optimization
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
