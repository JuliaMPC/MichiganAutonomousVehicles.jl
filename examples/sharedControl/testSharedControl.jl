using NLOptControl
using VehicleModels
using JuMP
using Ipopt
using Parameters
using KNITRO

using OCP

# initializatin data
c=defineCase(;track=:NA,obstacles=:o3);
c.m=setMisc(;sm=2.0,tex=0.2,tp=3.0,X0=[0.0, 0.0, 0.0, 0.0, pi/2],UX=15.0)
mdl,d,n,r,params = initializeSharedControl(c); # initialize problem

s=Settings(;format=:png,MPC=false,reset=true,save=true);
X0 = [0.0, 0.0, 0.0, 0.0, pi/2]; SA=0.0; UX=15.0;
t_opt, sa_opt, t_sample, sa_sample = sharedControl(mdl,n,r,s,params,X0,SA,UX;Iter=22)

# test 1
X0 = [0.0, 0.0, 0.0, 0.0, pi/2]; SA=0.3; UX=15.0;
pass, AL, AU = feasible(mdl,d,n,r,s,params,X0,SA,UX)

# postProcess
using Plots, PrettyPlots
pyplot()
include("plots.jl");
r.results_dir = string(r.main_dir,"/results/","test_1/");
resultsDir(r.results_dir);
allPlots(n,r,s,1); # TODO should it be 1 or 2?
