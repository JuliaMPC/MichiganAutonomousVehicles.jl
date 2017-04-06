using NLOptControl
using VehicleModels
using MAVs

s=Settings(;format=:png,MPC=false,save=true);

# initialize problem\
c=defineCase(;(:mode=>:path));
mdl,n,r,params=initializePathFollowing(c);

optimize(mdl,n,r,s); # first run

# postProcess
using PrettyPlots, Plots
pyplot()
r.results_dir = string(r.main_dir,"/results/","TMP/");
resultsDir(r.results_dir);
allPlots(n,r,s,1);
pSimPath(n,r,s,c,1)
