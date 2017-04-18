using NLOptControl
using VehicleModels
using Parameters
using DataFrames
using MAVs

"""
main(:activeSafety)
main(:sharedControl)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 4/6/2017, Last Modified: 4/7/2017 \n
--------------------------------------------------------------------------------------\n
"""
function main(matlabMode)

  try
    close(e.s1)
    close(e.s2)
  end
  # THINGS TO THINK ABOUT TODO
  # 6) intialize obstacle field in intial solve
  # 7) get the vehicle states from MATLAB so that I can plot things as well

  c=defineCase(;(:mode=>:caseStudy));

  if matlabMode==:activeSafety
    setMisc!(c;activeSafety=true,followPath=true,followDriver=true,PredictX0=false,FixedTp=false,tp=4.0,tex=0.2,max_cpu_time=0.18,Ni=2,Nck=[10,10]);
    setWeights!(c;sr=0.08,path=8.0,driver=0.0)
  elseif matlabMode==:sharedControl
    #setMisc!(c;activeSafety=false,followPath=true,followDriver=false,PredictX0=false,FixedTp=false,tp=4.0,tex=0.2,max_cpu_time=0.18,Ni=2,Nck=[10,10]);
    #setWeights!(c;sr=0.08,path=10.0,driver=0.0)
    setMisc!(c;activeSafety=false,followPath=true,followDriver=false,PredictX0=false,FixedTp=false,tp=4.0,tex=0.2,max_cpu_time=0.18,Ni=2,Nck=[10,10]);
    setWeights!(c;sr=0.08,path=10.0,driver=0.0)

    # fixed updates
    #setMisc!(c;activeSafety=false,followPath=true,followDriver=false,PredictX0=false,FixedTp=true,tp=9.0,tex=0.5,max_cpu_time=0.45,Ni=4,Nck=[12,10,8,6]);
  end

  mdl,n,r,params,x,d=initializeSharedControl!(c)

  # setup UDP port and UDP communication
  x.s1 = UDPSocket();bind(x.s1,ip"141.212.140.176",36880); # change this ip to the server (or julia code)
  x.s2 = UDPSocket();bind(x.s2,ip"141.212.140.176",12000); # change this ip to the server (or julia code)

  global pa=params[1];
  global s=Settings(;reset=false,save=true,simulate=true,MPC=true,format=:png);
  r.eval_num=1;
  # set infeasible_counter for active safety
  x.infeasible_counter=100; x.infeasible_counter_max=10;  # initially the condition will be false
  while(Bool(x.runJulia))
    println("Running model for the: ",r.eval_num," time");
    println("Getting Vehicle Position From MATLAB");
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

    if !c.m.activeSafety || !x.feasible || (x.infeasible_counter < x.infeasible_counter_max)
      println("Running Optimization");
      sharedControl!(mdl,n,r,s,params,x);   # rerun optimization
    end
    sendOptData!(n,r,x)
  end
  close(x.s1)
  close(x.s2)
end
#include("postProcess.jl")
