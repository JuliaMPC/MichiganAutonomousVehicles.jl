#=
using PrettyPlots
ENV["GKS_WSTYPE"]="mov"
using Plots
gr(show=true)

#pgfplots()
#pyplot()

description = string(
"In this test: \n",c.m.name,"\n
* m.Ni=",c.m.Ni," \n
* m.Nck=",c.m.Nck,"\n
* m.tp=",c.m.tp," \n
* m.tex=",c.m.tex,"\n
* m.max_cpu_time=",c.m.max_cpu_time," \n
* m.Nck=",c.m.Nck,"\n
")
results_dir=string(r.main_dir,"/results","/testB_",c.m.name,"_",c.m.solver,"/")
resultsDir!(r,results_dir;description=description);

println("Plotting the Final Results!")
if r.eval_num>1;
  for ii in 1:length(r.dfs_plant)
    mainSim(n,r,s,c,pa,ii);
  end
#  gif(anim, string(r.results_dir,"mainSim.gif"), fps = Int(ceil(1/c.m.tex)));
  cd(r.results_dir)
  #  run(`ffmpeg -f gif -i mainSim.gif RESULT.mp4`)
    write("description.txt", description)
  cd(r.main_dir)
end

#s=Settings(;save=true,MPC=true,simulate=false,format=:png);
#allPlots(n,r,s,2)


#anim = @animate
=#
using PrettyPlots
ENV["GKS_WSTYPE"]="mov"
using Plots
gr(show=true)

for ii in 1:length(r.dfs_plant)
  mainSim(n,r,s,c,pa,ii);
end
