using PrettyPlots
using Plots

#pgfplots()
pyplot()

description = string(
"In this test: \n",c.m.name,"\n
* m.Ni=",c.m.Ni," \n
* m.Nck=",c.m.Nck,"\n
* m.tp=",c.m.tp," \n
* m.tex=",c.m.tex,"\n
* m.max_cpu_time=",c.m.max_cpu_time," \n
* m.Nck=",c.m.Nck,"\n
* X0=",c.o.X0,"\n
* Y0=",c.o.Y0,"\n
* A=",c.o.A,"\n
* B=",c.o.B,"\n
* sx=",c.o.s_x,"\n
* sy=",c.o.s_y,"\n
")
results_dir=string("posterA_",c.m.name,"_",c.m.solver,"/")
resultsDir!(r,results_dir;description=description);

println("Plotting the Final Results!")
if r.eval_num>1;
 anim = @animate for ii in 1:length(r.dfs_plant)
    mainSim(n,r,s,c,pa,ii);
  end
  gif(anim, string(r.results_dir,"mainSim.gif"), fps = Int(ceil(1/c.m.tex)));
  cd(r.results_dir)
    run(`ffmpeg -f gif -i mainSim.gif RESULT.mp4`)
    write("description.txt", description)
  cd(r.main_dir)
end

#s=Settings(;save=true,MPC=true,simulate=false,format=:png);
#allPlots(n,r,s,2)
