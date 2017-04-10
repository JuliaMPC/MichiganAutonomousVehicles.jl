using PrettyPlots, Plots
#gr()

s=Settings();


r.results_dir = string(r.main_dir,"/results/",c.m.name,"_",c.m.solver,"_INITAL/")
resultsDir(r.results_dir);


#s=Settings(;reset=false,save=true,simulate=true,MPC=true,format=:png);
s=Settings(;save=false)
anim = @animate for ii in 1:length(r.dfs)
  pSimPath(n,r,s,c,ii)
end
gif(anim, string(r.results_dir,"pSimPath.gif"), fps = 5);
#cd(r.results_dir)
 #run(`ffmpeg -f gif -i pSimPath.gif RESULT.mp4`)
#cd(r.main_dir)


s=Settings(;MPC=false)

pSimPath(n,r,s,c,1)
