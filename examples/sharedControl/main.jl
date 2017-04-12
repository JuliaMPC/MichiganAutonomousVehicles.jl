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
# 11) fix time -> update each time based off of simulink

# setup UDP port and UDP communication
x.s1 = UDPSocket();bind(x.s1,ip"141.212.140.176",36880); # change this ip to the server (or julia code)
x.s2 = UDPSocket();bind(x.s2,ip"141.212.140.176",12000); # change this ip to the server (or julia code)

c=defineCase(;(:mode=>:caseStudy));
setMisc!(c;activeSafety=true,followPath=false,followDriver=false,predictX0=false,fixedTp=false,tp=5.0,tex=0.3,max_cpu_time=0.26,Ni=3,Nck=[10,8,6]);
mdl,n,r,params,x,d=initializeSharedControl!(c)

global pa=params[1];
global s=Settings(;reset=false,save=true,simulate=true,MPC=true,format=:png);

r.eval_num=1;
# assume vehicle is driving straight... -0.0002495371
r.U=0*Matrix{Float64}(n.numControlPoints,n.numControls); # for initial predictX0 [SR and JX]
r.t_ctr=Vector(Ranges.linspace(0,copy(c.m.tp),n.numControlPoints)); # gives a bunch of points

while(Bool(x.runJulia))
  println("Running model for the: ",r.eval_num," time");
  println("Getting Vehicle Position From MATLAB");
  getPlantData!(n,params,x);

  updateX0!(n,r,x.X0;(:userUpdate=>true));

  if c.m.activeSafety
    checkFeasibility!(mdl,d,n,c,e,params[1],r;feas_tol=0.005);
  end

  if !c.m.activeSafety || !e.feasible
    println("Running Optimization");
    sharedControl!(mdl,n,r,s,params,x);   # rerun optimization
    println("Sending Optimized Steering Angle Commands to MATLAB");
    sendOptData!(n,r,x);
  else # problem is feasible
    sendOptData!(n,r,x;(:feasible=>true))
  end

end
close(x.s1)
close(x.s2)
include("postProcess.jl")
