using PrettyPlots, Plots
default(guidefont = font(17), tickfont = font(15), legendfont = font(12), titlefont = font(20))

"""

--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 4/13/2017, Last Modified: 4/13/2017 \n
--------------------------------------------------------------------------------------\n
"""

function posterP(n,r,s,c)

  # static plots for each frame
  idx=r.eval_num;
  sap=statePlot(n,r,s,idx,6)
  longv=statePlot(n,r,s,idx,7)
  axp=axLimsPlot(n,r,s,pa,idx); # add nonlinear acceleration limits
  axp=statePlot(n,r,s,idx,8,axp;(:lims=>false),(:append=>true));
  pp=statePlot(n,r,s,idx,1,2;(:lims=>false));
  if s.MPC; tp=tPlot(n,r,s,idx); else; tp=plot(0,leg=:false); end
  vt=vtPlot(n,r,s,pa,c,idx)

  # dynamic plots ( maybe only update every 5 frames or so)
  v=Vector(1:5:r.eval_num); if v[end]!=r.eval_num; append!(v,r.eval_num); end
  for ii in v
    if ii==1
      st1=1;st2=2;
      # values
  		temp = [r.dfs_plant[jj][n.state.name[st1]] for jj in 1:r.eval_num];
  		vals1=[idx for tempM in temp for idx=tempM];

  		# values
  		temp = [r.dfs_plant[jj][n.state.name[st2]] for jj in 1:r.eval_num];
  		vals2=[idx for tempM in temp for idx=tempM];

  		pp=plot(vals1,vals2,w=s.lw2,label="Vehicle Trajectory");

      pp=obstaclePlot(n,r,s,c,ii,pp;(:append=>true),(:posterPlot=>true));                     # add obstacles
      pp=vehiclePlot(n,r,s,c,ii,pp;(:append=>true),(:posterPlot=>true));  # add the vehicle
    else
      pp=obstaclePlot(n,r,s,c,ii,pp;(:append=>true),(:posterPlot=>true));  # add obstacles
      pp=vehiclePlot(n,r,s,c,ii,pp;(:append=>true),(:posterPlot=>true));  # add the vehicle
    end
  end
  l = @layout [a{0.5w} [grid(2,2)
                        b{0.2h}]]
  poster=plot(pp,sap,vt,longv,axp,tp,layout=l,size=(1800,1200));
  savefig(string(r.results_dir,"poster",".",s.format));
  nothing
end

#pp=obstaclePlot(n,r,s,c,ii,pp;(:append=>true),(:posterPlot=>true)); # add obstacles
#pp=vehiclePlot(n,r,s,c,ii,pp;(:append=>true),(:posterPlot=>true));  # add the vehicle


function obstaclePlot(n,r,s,c,idx,args...;kwargs...)
  kw = Dict(kwargs);

  # check to see if is a basic plot
  if !haskey(kw,:basic); kw_ = Dict(:basic => false); basic = get(kw_,:basic,0);
  else; basic=get(kw,:basic,0);
  end

  # check to see if is a poster plot
  if !haskey(kw,:posterPlot); kw_ = Dict(:posterPlot=>false); posterPlot= get(kw_,:posterPlot,0);
  else; posterPlot=get(kw,:posterPlot,0);
  end

  if basic
    s=Settings();
    pp=plot(0,leg=:false)
    if !isempty(c.o.A)
      for i in 1:length(c.o.A)
          # create an ellipse
          pts = Plots.partialcircle(0,2π,100,c.o.A[i])
          x, y = Plots.unzip(pts)
          tc=0;
          x += c.o.X0[i] + c.o.s_x[i]*tc;
          y = c.o.B[i]/c.o.A[i]*y + c.o.Y0[i] + c.o.s_y[i]*tc;
          pts = collect(zip(x, y))
          if i==1
            plot!(pts, line=(s.lw1,0.7,:solid,:red),fill = (0, 0.7, :red),leg=true,label="Obstacles")
          else
            plot!(pts, line=(s.lw1,0.7,:solid,:red),fill = (0, 0.7, :red),leg=true,label="")
          end
      end
    end
  else
    # check to see if user would like to add to an existing plot
    if !haskey(kw,:append); kw_ = Dict(:append => false); append = get(kw_,:append,0);
    else; append = get(kw,:append,0);
    end
    if !append; pp=plot(0,leg=:false); else pp=args[1]; end

    # plot the goal; assuming same in x and y
    if c.g.name!=:NA
      if isnan(n.XF_tol[1]); rg=1; else rg=n.XF_tol[1]; end
      if !posterPlot || idx ==r.eval_num
        pts = Plots.partialcircle(0,2π,100,rg);
        x, y = Plots.unzip(pts);
        x += c.g.x_ref;  y += c.g.y_ref;
        pts = collect(zip(x, y));
        plot!(pts,line=(s.lw1,1,:solid,:green),fill = (0, 1, :green),leg=true,label="Goal")
      end
    end

    if c.o.name!=:NA
      for i in 1:length(c.o.A)
        # create an ellipse
        pts = Plots.partialcircle(0,2π,100,c.o.A[i])
        x, y = Plots.unzip(pts)
        if s.MPC
          x += c.o.X0[i] + c.o.s_x[i]*r.dfs_plant[idx][:t][end];
          y = c.o.B[i]/c.o.A[i]*y + c.o.Y0[i] + c.o.s_y[i]*r.dfs_plant[idx][:t][end];
        else
          if r.dfs[idx]!=nothing
            tc=r.dfs[idx][:t][end];
          else
            tc=0;
            warn("\n Obstacles set to inital condition for current frame!! \n")
          end
          x += c.o.X0[i] + c.o.s_x[i]*tc;
          y = c.o.B[i]/c.o.A[i]*y + c.o.Y0[i] + c.o.s_y[i]*tc;
        end
        pts = collect(zip(x, y))
        if posterPlot
          shade=idx/r.eval_num;
          if idx==r.eval_num && i==4  # NOTE either the obstacles increase in size or we do not get a legend, so this is a fix for now ! -> making it an obstacle that does not appear on the plot at that time
            plot!(pts,line=(s.lw1,shade,:solid,:red),fill=(0,shade,:red),leg=true,label="Obstacles")
        #    plot!(pts,line=(s.lw1,0.0,:solid,:red),fill=(0,shade,:red),leg=true,label="Obstacles")
          else
            plot!(pts,line=(s.lw1,0.0,:solid,:red),fill=(0,shade,:red),leg=true,label="")
          end
          #annotate!(x[1],y[1],text(string("t=",idx*c.m.tex," s"),10,:black,:center))
          X=c.o.X0[i] + c.o.s_x[i]*r.dfs_plant[idx][:t][end]
          Y=c.o.Y0[i] + c.o.s_y[i]*r.dfs_plant[idx][:t][end];
          annotate!(X,Y,text(string(idx*c.m.tex,"s"),10,:black,:center))
        else
          if i==1
            plot!(pts, line=(s.lw1,0.7,:solid,:red),fill = (0, 0.7, :red),leg=true,label="Obstacles")
          else
            plot!(pts, line=(s.lw1,0.0,:solid,:red),fill = (0, 0.7, :red),leg=true,label="")
          end
        end
      end
    end

    xaxis!(n.state.description[1]);
    yaxis!(n.state.description[2]);
    if !s.simulate savefig(string(r.results_dir,"/",n.state.name[1],"_vs_",n.state.name[2],".",s.format)); end
  end
  return pp
end

"""
pp=vehiclePlot(n,r,s,c,idx);
pp=vehiclePlot(n,r,s,c,idx,pp;(:append=>true));
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/11/2017, Last Modified: 4/4/2017 \n
--------------------------------------------------------------------------------------\n
"""
function vehiclePlot(n,r,s,c,idx,args...;kwargs...)
  kw = Dict(kwargs);

  if !haskey(kw,:zoom); kw_=Dict(:zoom => false); zoom=get(kw_,:zoom,0);
  else; zoom=get(kw,:zoom,0);
  end

  # check to see if user would like to add to an existing plot
  if !haskey(kw,:append); kw_ = Dict(:append => false); append = get(kw_,:append,0);
  else; append = get(kw,:append,0);
  end
  if !append; pp=plot(0,leg=:false); else pp=args[1]; end

  # check to see if is a poster plot
  if !haskey(kw,:posterPlot); kw_ = Dict(:posterPlot=>false); posterPlot= get(kw_,:posterPlot,0);
  else; posterPlot=get(kw,:posterPlot,0);
  end

  # contants
  w=1.9; #width TODO move these to vehicle parameters
  h=3.3; #height
  XQ = [-w/2 w/2 w/2 -w/2 -w/2];
  YQ = [h/2 h/2 -h/2 -h/2 h/2];

  # plot the vehicle
  if s.MPC
    X_v = r.dfs_plant[idx][:x][end]  # using the end of the simulated data from the vehicle model
    Y_v = r.dfs_plant[idx][:y][end]
    PSI_v = r.dfs_plant[idx][:psi][end]-pi/2
  else
    X_v = r.dfs[idx][:x][1] # start at begining
    Y_v = r.dfs[idx][:y][1]
    PSI_v = r.dfs[idx][:psi][1]-pi/2
  end

  P = [XQ;YQ];
  ct = cos(PSI_v);
  st = sin(PSI_v);
  R = [ct -st;st ct];
  P2 = R * P;
  if !posterPlot || idx==r.eval_num
    scatter!((P2[1,:]+X_v,P2[2,:]+Y_v), markershape = :square, markercolor = :black, markersize = s.ms1, fill = (0, 1, :black),leg=true, grid=true,label="Vehicle")
  else
    scatter!((P2[1,:]+X_v,P2[2,:]+Y_v), markershape = :square, markercolor = :black, markersize = s.ms1, fill = (0, 1, :black),leg=true, grid=true,label="")
  end
  if !zoom
    if s.MPC
      xlims!(minDF(r.dfs_plant,:x),maxDF(r.dfs_plant,:x));
      ylims!(minDF(r.dfs_plant,:y),maxDF(r.dfs_plant,:y));
    else
      xlims!(minDF(r.dfs,:x),maxDF(r.dfs,:x));
      ylims!(minDF(r.dfs,:y),maxDF(r.dfs,:y));
    end
  else
    xlims!(X_v-20.,X_v+80.);
    ylims!(Y_v-50.,Y_v+50.);
    plot!(leg=:false)
  end

  if posterPlot
    t=idx*c.m.tex;
    annotate!(X_v,Y_v-4,text(string("t=",t," s"),10,:black,:center))
    xlims!(c.m.Xlims[1],c.m.Xlims[2]);
    ylims!(c.m.Ylims[1],c.m.Ylims[2]);
    #xlims!(158,224);
    #ylims!(-2,130);
  end
  if !s.simulate; savefig(string(r.results_dir,"x_vs_y",".",s.format)); end
  return pp
end

s=Settings(;save=true,MPC=true,simulate=false,format=:png,plantOnly=true);
#results_dir=string("testB_",c.m.name,"_",c.m.solver,"/")
#resultsDir!(r,results_dir;description=description);

posterP(n,r,s,c)

#=
a=5
b=1
x0=2
y0=0

pts=Plots.partialcircle(0,2pi,100,a);
x, y = Plots.unzip(pts);
x+=x0
y=b/a*y+ y0
pts = collect(zip(x, y));
plot(pts,fill=(0,1,:red));
ylims!(-10,10)
xlims!(-10,10)
=#
