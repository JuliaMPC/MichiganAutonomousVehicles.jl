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

function pmain(n,r,s,c)
  if r.eval_num>2;
     anim = @animate for ii in 1:length(r.dfs)
      mainSimPath(n,r,s,c,pa,ii);
    end
    gif(anim, string(r.results_dir,"mainSimPath.gif"), fps = Int(ceil(1/c.m.tex)));
    cd(r.results_dir)
      run(`ffmpeg -f gif -i mainSimPath.gif RESULT.mp4`)
      write("description.txt", description)
    cd(r.main_dir)
  else
    s=Settings(;save=true,MPC=false,simulate=false,format=:png);
    pSimPath(n,r,s,c,2)
    allPlots(n,r,s,2)
  end
  nothing
end

function pp(n,r,s,c)
  anim = @animate for ii in 1:length(r.dfs)
    pSimPath(n,r,s,c,ii);
  end
  gif(anim, string(r.results_dir,"pSimPath.gif"), fps = 5);
  nothing
end

println("Plotting the Final Results!")
s=Settings(;reset=false,save=true,simulate=true,MPC=false,format=:png);
pmain(n,r,s,c)
pp(n,r,s,c)

if r.dfs_opt[r.eval_num-1][:status]==:Infeasible
  s=Settings(;evalConstraints=true,save=true,MPC=false,simulate=false,format=:png);
  postProcess(n,r,s)
  # trouble getting a feasible solution? -> look at the constraints
  print(r.constraint.value)
end # then consider relaxing tolerances etc. to make it work
