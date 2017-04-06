using PrettyPlots, Plots
#gr();pgfplots();
pyplot();

r.results_dir = string(r.main_dir,"/results","/TMP1_",c.m.name,"_",c.m.solver,"/")
resultsDir(r.results_dir);

description = string(
"In this test: \n",c.name,"\n
 RESULTS DISCUSSION:  \n
 *    m.max_cpu_time=m.tex*10;\n
 *
 ")

cd(r.results_dir)
  write("description.txt", description)
cd(r.main_dir)

println("Plotting the Final Results!")
if r.eval_num>1;
  anim = @animate for ii in 1:length(r.dfs_plant)
    mainSim(n,r,s,c,pa,ii);
  end
  gif(anim, string(r.results_dir,"mainSim.gif"), fps = 1);
end

s=Settings(;save=true,MPC=true,simulate=false,format=:png);
allPlots(n,r,s,2)
