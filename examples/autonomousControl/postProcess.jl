using PrettyPlots
using Plots
#gr()
#pgfplots()
pyplot()
default(guidefont = font(17), tickfont = font(15), legendfont = font(12), titlefont = font(20))

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

results_dir=string("posterA2_",c.m.name,"_",c.m.solver,"/")
resultsDir!(r,results_dir;description=description);
savePlantData(n,r)
if s.simulate
  println("Plotting the Final Results!")
  mainSim(n,r,s,c,pa;(:mode=>:open1))
end

s=Settings(;save=true,MPC=true,simulate=false,format=:png,plantOnly=true);
posterP(n,r,s,c,pa)
