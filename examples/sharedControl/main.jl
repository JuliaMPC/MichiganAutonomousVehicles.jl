using NLOptControl
using VehicleModels
using Parameters
using DataFrames
using MAVs

#s1 = UDPSocket();bind(s1,ip"35.2.235.173",36880);
#s1 = UDPSocket();bind(s1,ip"141.212.140.176",36880); # change this ip to the server (or julia code)
try
  close(e.s1)
  close(e.s2)
end
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
# 8) it was infeasible so you should not track that!!!!
# 9) make the vehicle drive strIght for a bit?
# 10 ) julia does not start for a while
# 11) fix time -> update each time based off of simulink

  e=ExternalModel(); # setup data with Matlab
  # setup UDP port and UDP communication
  #e.s1 = UDPSocket();bind(e.s1,ip"141.212.140.176",36880); # change this ip to the server (or julia code)
  #e.s2 = UDPSocket();bind(e.s2,ip"141.212.140.176",12000); # change this ip to the server (or julia code)
  e.s1 = UDPSocket();bind(e.s1,ip"141.212.140.176",36880); # change this ip to the server (or julia code)
  e.s2 = UDPSocket();bind(e.s2,ip"141.212.140.176",12000); # change this ip to the server (or julia code)

  c=defineCase(;(:mode=>:caseStudy));

  mdl,n,r,params=initializePathFollowing(c);

  global pa=params[1];
  #global s=Settings(;reset=false,save=true,simulate=true,MPC=true,format=:png);
  global s=Settings(;reset=false,save=true,simulate=true,MPC=true,format=:png,evalConstraints=true);

  r.eval_num=1;
  # assume vehicle is driving straight... -0.0002495371

  r.U=0*Matrix{Float64}(n.numControlPoints,n.numControls);
  r.t_ctr=Vector(Ranges.linspace(0,5,n.numControlPoints)); # gives a bunch of points
#try

  while(Bool(e.runJulia))
    println("Running model for the: ",r.eval_num," time");
    println("Getting Vehicle Position From MATLAB");
    getPlantData!(n,params,e);

    # update X0; NOTE: in the other examples this is done automatically) and for the first one it was already done
    updateX0!(n,r,e.X0;(:userUpdate=>true));

    println("Running Optimization");
    sharedControl!(mdl,n,r,s,params,e);   # rerun optimization

    println("Sending Optimized Steering Angle Commands to MATLAB");
    sendOptData!(r,e);
  end
#finally
  close(e.s1)
  close(e.s2)
