using MichiganAutonomousVehicles
using NLOptControl
using Plots

case_name = "s12"
c = load(open(string(Pkg.dir("MichiganAutonomousVehicles"),"/config/planner/","RTPP",".yaml")))
c["vehicle"] = load(open(string(Pkg.dir("MichiganAutonomousVehicles"),"/config/vehicle/","hmmwv",".yaml")))
c["goal"] = load(open(string(Pkg.dir("MichiganAutonomousVehicles"),"/config/case/",case_name,".yaml")))["goal"]
c["X0"] = load(open(string(Pkg.dir("MichiganAutonomousVehicles"),"/config/case/",case_name,".yaml")))["X0"]
c["obstacle"] = load(open(string(Pkg.dir("MichiganAutonomousVehicles"),"/config/case/",case_name,".yaml")))["obstacle"]
setConfig(c,"misc";(:N=>40),(:model=>:ThreeDOFv2),(:solver=>:Ipopt),(:integrationScheme=>:trapezoidal))
setConfig(c,"X0";(:ux=>0.1))# the type needs to be a float
# otherwise, this will pop up: WARNING: Ipopt finished with status Invalid_Number_Detected
fixYAML(c)   # fix messed up data types
n = initializeAutonomousControl(c);
simMPC!(n;updateFunction=updateAutoParams!,checkFunction=checkCrash)

include("postProcess.jl")
