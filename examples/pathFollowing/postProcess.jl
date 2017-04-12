using PrettyPlots, Plots, DataFrames
gr();
#pgfplots();
#pyplot();

description = string(
"In this test: \n",c.m.name,"\n
* m.Ni=",c.m.Ni," \n
* m.Nck=",c.m.Nck,"\n
* m.tp=",c.m.tp," \n
* m.tex=",c.m.tex,"\n
* m.max_cpu_time=",c.m.max_cpu_time," \n
* m.Nck=",c.m.Nck,"\n
* m.PredictX0=",c.m.PredictX0," \n
* m.FixedTp=",c.m.FixedTp,"\n
")
results_dir=string(r.main_dir,"/results","/testA_",c.m.name,"_",c.m.solver,"/")
resultsDir!(r,results_dir;description=description);


# save data TODO push this to NLOptControl.jl
function savePlantData(r)
  dfs=DataFrame();
  temp = [r.dfs_plant[jj][:t][1:end-1,:] for jj in 1:length(r.dfs_plant)]; # time
  U=[idx for tempM in temp for idx=tempM]; dfs[:t]=U;

  for st in 1:n.numStates # state
    temp = [r.dfs_plant[jj][n.state.name[st]][1:end-1,:] for jj in 1:length(r.dfs_plant)];
    U=[idx for tempM in temp for idx=tempM];
    dfs[n.state.name[st]]=U;
  end

  for ctr in 1:n.numControls # control
    temp = [r.dfs_plant[jj][n.control.name[ctr]][1:end-1,:] for jj in 1:length(r.dfs_plant)];
    U=[idx for tempM in temp for idx=tempM];
    dfs[n.control.name[ctr]]=U;
  end
  cd(r.results_dir)
    writetable("plant_data.csv",dfs);
  cd(r.main_dir)
  return nothing
end

cd(r.results_dir)
  write("description.txt", description);
  savePlantData(r)
cd(r.main_dir)

println("Plotting the Final Results!")
s=Settings(;reset=false,save=true,simulate=true,MPC=true,format=:png);

#s=Settings(;evalConstraints=true,save=true,MPC=false,simulate=false,format=:png);
#mainSimPath(n,r,s,c,pa,r.eval_num-1);

if r.eval_num>2;

   anim = @animate for ii in 1:length(r.dfs_plant)
    mainSimPath(n,r,s,c,pa,ii);
  end
  gif(anim, string(r.results_dir,"mainSimPath.gif"), fps = 5);
  cd(r.results_dir)
    run(`ffmpeg -f gif -i mainSimPath.gif RESULT.mp4`)
  cd(r.main_dir)

#  pgfplots();
#  s=Settings(;evalConstraints=true,save=true,MPC=false,simulate=false,format=:png);
#  mainSimPath(n,r,s,c,pa,length(r.dfs_plant));
else
  s=Settings(;save=true,MPC=false,simulate=false,format=:png);
  pSimPath(n,r,s,c,2)
  allPlots(n,r,s,2)
end

if r.dfs_opt[r.eval_num][:status]==:Infeasible
  s=Settings(;evalConstraints=true,save=true,MPC=false,simulate=false,format=:png);
  postProcess(n,r,s)
  # trouble getting a feasible solution? -> look at the constraints
  print(r.constraint.value)
end # then consider relaxing tolerances etc. to make it work
# then looking at the output
# These are the dual infeasibilities
#=
│ Row │ step │ x0_con   │
├─────┼──────┼──────────┤
│ 1   │ 1    │ -267.777 │
│ 2   │ 2    │ 64.6356  │
│ 3   │ 3    │ 271.738  │
│ 4   │ 4    │ 2.44213  │
│ 5   │ 5    │ 3496.5   │
=#
# These are the dual infeasibilities

# we cam try to relax the constraint on the

#=
# define tolerances
XF_tol=[NaN,NaN,NaN,NaN,NaN];
X0_tol=[0.05,0.05,0.05,0.05,0.01];
defineTolerances(n;X0_tol=X0_tol,XF_tol=XF_tol);
=#

#using PrettyPlots
#s=Settings(;save=true,MPC=false,simulate=false,format=:png);
#pSimPath(n,r,s,c,r.eval_num)
