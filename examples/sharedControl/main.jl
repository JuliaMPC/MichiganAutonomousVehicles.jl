using NLOptControl
using VehicleModels
using Parameters
using DataFrames
using MAVs

# THINGS TO THINK ABOUT TODO
# 1) what if we sent a torque signal?'
# (a) have a PID controller follow the steering angle sent by the automation and record the torque -> maybe characterize the system somehow
  # different settings for different drivers?
  # basically fight a lot, medium and not so much and look at the torque that is needed to overcome
  # set up some sort of mapping then and different "levels" of automation based off of a higher level controller

# 2) the integration thing may not make sense anymore!!
# 4) think about settings, careful!
# 5)  if r.dfs_opt[r.eval_num][:status][end]!==:Infeasible # if infeasible -> let user control TODO what is this YINgshi?  sendOptData
# 6) intialize obstacle field in intial solve
# 7) get the vehicle states from MATLAB so that I can plot things as well

c=defineCase(;(:mode=>:caseStudy));

e=ExternalModel(); # setup data with Matlab
# setup UDP port and UDP communication
e.s1 = UDPSocket();bind(e.s1,ip"35.2.169.128",36880); # change this ip to the server (or julia code)
e.s2 = UDPSocket();bind(e.s2,ip"35.2.169.128",12000); # change this ip to the server (or julia code)

mdl,n,r,params=initializePathFollowing(c);
global pa=params[1];
global s=Settings(;reset=false,save=true,simulate=true,MPC=true,format=:png);

while(Bool(e.runJulia))
  println("Running model for the: ",r.eval_num," time");
  println("Getting Vehicle Position From MATLAB");
  getPlantData(n,params,e);

  # update X0
  updateX0(n,r,e.X0;(:userUpdate=>true));
  if n.mpc.predictX0 # predict where X0 will be when optimized signal is actually sent to the vehicle
    predictX0(n,pa,r;(:fixedTp=>false))
  else
    n.mpc.X0p=e.X0;  # autonomousControl (inside sharedControl()) uses n.mpc.X0p
  end

  println("Running Optimization");
  sharedControl(mdl,n,r,s,params,e);   # rerun optimization

  println("Sending Optimized Steering Angle Commands to MATLAB");
  sendOptData(r,e);
end
