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

#Ok version
#setMisc!(c;activeSafety=true,followPath=true,followDriver=false,PredictX0=false,FixedTp=false,tp=3.5,tex=0.35,max_cpu_time=0.3,Ni=3,Nck=[10,8,6]);
#setWeights!(c;sr=0.5,path=0.1,driver=0.1)
setMisc!(c;activeSafety=true,followPath=false,followDriver=false,PredictX0=false,FixedTp=false,NF=0,tp=3.0,tex=0.5,max_cpu_time=0.45,Ni=3,Nck=[12,10,8]);
setWeights!(c;sr=0.05,path=5.0,driver=0.0)

mdl,n,r,params,x,d=initializeSharedControl!(c)

# setup UDP port and UDP communication
x.s1 = UDPSocket();bind(x.s1,ip"141.212.140.176",36880); # change this ip to the server (or julia code)
x.s2 = UDPSocket();bind(x.s2,ip"141.212.140.176",12000); # change this ip to the server (or julia code)

global pa=params[1];
global s=Settings(;reset=false,save=true,simulate=true,MPC=true,format=:png);
r.eval_num=1;count=1;
# set infeasible_counter for active safety
x.infeasible_counter=100; x.infeasible_counter_max=2;  # initially the condition will be false
println("Getting Vehicle Position From MATLAB");
while(Bool(x.runJulia))
  if count!=r.eval_num
    println("Running model for the: ",r.eval_num," time");
  end
  count=r.eval_num;

  getPlantData!(n,params,x,c);

  updateX0!(n,r,x.X0;(:userUpdate=>true));

  if c.m.activeSafety
    checkFeasibility!(mdl,d,n,c,x,params[1],r;feas_tol=0.005);
    if !x.feasible
      x.infeasible_counter=1;
    else
      x.infeasible_counter+=1;
    end
  end

#  if !c.m.activeSafety || !x.feasible || (x.infeasible_counter < x.infeasible_counter_max)
  println("Running Optimization");
  sharedControl!(mdl,n,r,s,params,x);   # rerun optimization
#  end
  sendOptData!(n,r,x)
end
close(x.s1)
close(x.s2)

#include("postProcess.jl")