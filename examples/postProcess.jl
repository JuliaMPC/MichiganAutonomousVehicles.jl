using DataFrames, CSV
#gr()
#pgfplots()
a = 2
#pyplot(guidefont=font(a*17),tickfont=font(a*15),legendfont=font(a*12),titlefont=font(a*20))
#a=0.25;
#default(guidefont=font(a*17),tickfont=font(a*15),legendfont=font(a*12),titlefont=font(a*20))
plotSettings(;(:polyPts=>false),(:simulate=>true),(:mpc_markers =>(:circle,:blueviolet,0.0,0.0)),(:plant=>true),(:plantOnly=>false),(:size=>(a*900,a*600)),(:format=>"png"));

resultsDir!(n)
dfs = case2dfs(c)
filename = "case"
CSV.write(string("results/",filename,".csv"), dfs; quotechar = ' ')

################################################
# save obstacle and vehicle data for ploting
pts = 4 # sample at several points in time
t = Vector(linspace(0,n.r.ip.plant[:t][end],pts)) # assuming it starts at 0!
#t[3] = 19.5 # for s9_D2 to line up crash
indmin(abs.(t[2]-n.r.ip.plant[:t]))

xv = zeros(pts)
yv = zeros(pts)
psi = zeros(pts)
dfs = DataFrame()
for j in 1:pts
  idx = indmin(abs.(t[j]-n.r.ip.plant[:t]))
  xv[j] = n.r.ip.plant[:x][idx]
  yv[j] = n.r.ip.plant[:y][idx]
  psi[j] = n.r.ip.plant[:psi][idx]
end
dfs[:idx] = 1:pts
dfs[:t] = round.(t,1,10) # round for graph
dfs[:xv] = xv
dfs[:yv] = yv
dfs[:psi] = psi
cd(n.r.resultsDir)
  CSV.write("veh.csv", dfs; quotechar = ' ')
cd(n.r.mainDir)

for i in 1:length(c["obstacle"]["x0"])
  xo = zeros(pts)
  yo = zeros(pts)
  ro = zeros(pts)
  clr = zeros(pts)
  dfs = DataFrame()
  for j in 1:pts
    xo[j] = c["obstacle"]["x0"][i] + c["obstacle"]["vx"][i]*t[j]
    yo[j] = c["obstacle"]["y0"][i] + c["obstacle"]["vy"][i]*t[j]
    ro[j] = c["obstacle"]["radius"][i]
    clr[j] = (j)/pts*100
  end
  dfs[:idx] = 1:pts
  dfs[:t] = round.(t,1,10) # round for graph
  dfs[:xo] = xo
  dfs[:yo] = yo
  dfs[:ro] = ro
  dfs[:xv] = xv
  dfs[:yv] = yv
  dfs[:clr] = clr
  cd(n.r.resultsDir)
    CSV.write(string("obs_",i,".csv"), dfs; quotechar = ' ')
  cd(n.r.mainDir)
end
# save obstacle and vehicle data for ploting
################################################

savePlantData!(n)
saveOptData(n)

if _pretty_defaults[:simulate]
  println("Plotting the Final Results!")
  mainSim(n;(:mode=>:zoom1))
end

#optPlot(n)
#posterP(n)


# calculate the control effort
dfs = DataFrame()
n.r.ip.plant[:sa][isnan.(n.r.ip.plant[:sa])]=0 # remove any NaNs
n.r.ip.plant[:sr][isnan.(n.r.ip.plant[:sr])]=0
n.r.ip.plant[:jx][isnan.(n.r.ip.plant[:jx])]=0
dfs[:sa] = mean(n.r.ip.plant[:sa].^2)
dfs[:sr] = mean(n.r.ip.plant[:sr].^2)
dfs[:jx] = mean(n.r.ip.plant[:jx].^2)
dfs[:total] = dfs[:sa] + dfs[:sr] + dfs[:jx]
cd(n.r.resultsDir)
  CSV.write("ce.csv", dfs; quotechar = ' ')
cd(n.r.mainDir)
