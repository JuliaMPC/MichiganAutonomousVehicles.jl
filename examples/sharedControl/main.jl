using NLOptControl
using VehicleModels
using Parameters
using DataFrames
using MAVs

try
  close(e.s1)
  close(e.s2)
end
# THINGS TO THINK ABOUT TODO

# 4) think about settings, careful!
# 6) intialize obstacle field in intial solve
# 7) get the vehicle states from MATLAB so that I can plot things as well
# 8) it was infeasible so you should not track that!!!!
# 11) fix time -> update each time based off of simulink

x=ExternalModel(); # setup data with Matlab
# setup UDP port and UDP communication
x.s1 = UDPSocket();bind(x.s1,ip"141.212.140.176",36880); # change this ip to the server (or julia code)
x.s2 = UDPSocket();bind(x.s2,ip"141.212.140.176",12000); # change this ip to the server (or julia code)

c=defineCase(;(:mode=>:caseStudy));
setMisc!(c;UX=10.0,tp=9.0,tex=0.5,max_cpu_time=.45,Ni=3,Nck=[10,10,10],mpc_max_iter=200,PredictX0=false,FixedTp=true);
mdl,n,r,params=initializePathFollowing(c);
r.results_dir = string(r.main_dir,"/results/",c.m.name,"_",c.m.solver,"_C/")
resultsDir(r.results_dir);
global pa=params[1];
global s=Settings(;reset=false,save=true,simulate=true,MPC=true,format=:png);
#global s=Settings(;reset=false,save=true,simulate=true,MPC=true,format=:png,evalConstraints=true);

r.eval_num=1;
# assume vehicle is driving straight... -0.0002495371
r.U=0*Matrix{Float64}(n.numControlPoints,n.numControls);
r.t_ctr=Vector(Ranges.linspace(0,5,n.numControlPoints)); # gives a bunch of points

while(Bool(x.runJulia))
  println("Running model for the: ",r.eval_num," time");
  println("Getting Vehicle Position From MATLAB");
  getPlantData!(n,params,x);

  # update X0; NOTE: in the other examples this is done automatically) and for the first one it was already done
  updateX0!(n,r,x.X0;(:userUpdate=>true));

  println("Running Optimization");
  sharedControl!(mdl,n,r,s,params,x);   # rerun optimization

  println("Sending Optimized Steering Angle Commands to MATLAB");
  sendOptData!(n,r,x);
end
close(x.s1)
close(x.s2)
include("postProcess.jl")
