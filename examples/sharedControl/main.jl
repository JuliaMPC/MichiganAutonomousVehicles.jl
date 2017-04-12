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
# 6) intialize obstacle field in intial solve
# 7) get the vehicle states from MATLAB so that I can plot things as well

c=defineCase(;(:mode=>:caseStudy));
setMisc!(c;activeSafety=true,followPath=true,followDriver=false,PredictX0=false,FixedTp=false,tp=6.0,tex=0.3,max_cpu_time=0.26,Ni=3,Nck=[10,8,6]);
#setMisc!(c;activeSafety=false,followPath=true,followDriver=false,PredictX0=false,FixedTp=false,tp=9.0,tex=0.3,max_cpu_time=0.26,Ni=3,Nck=[10,8,6]);
mdl,n,r,params,x,d=initializeSharedControl!(c)

# setup UDP port and UDP communication
x.s1 = UDPSocket();bind(x.s1,ip"141.212.140.176",36880); # change this ip to the server (or julia code)
x.s2 = UDPSocket();bind(x.s2,ip"141.212.140.176",12000); # change this ip to the server (or julia code)

global pa=params[1];
global s=Settings(;reset=false,save=true,simulate=true,MPC=true,format=:png);

r.eval_num=1;
# assume vehicle is driving straight... -0.0002495371
#r.U=0*Matrix{Float64}(n.numControlPoints,n.numControls); # for initial predictX0 [SR and JX]
#r.t_ctr=Vector(Ranges.linspace(0,copy(c.m.tp),n.numControlPoints)); # gives a bunch of points

while(Bool(x.runJulia))
  println("Running model for the: ",r.eval_num," time");
  println("Getting Vehicle Position From MATLAB");
  getPlantData!(n,params,x,c);

  updateX0!(n,r,x.X0;(:userUpdate=>true));

  if c.m.activeSafety
    checkFeasibility!(mdl,d,n,c,x,params[1],r;feas_tol=0.005);
  end

  if !c.m.activeSafety || !x.feasible
    println("Running Optimization");
    sharedControl!(mdl,n,r,s,params,x);   # rerun optimization
  end
  sendOptData!(n,r,x)
end
close(x.s1)
close(x.s2)
#include("postProcess.jl")
