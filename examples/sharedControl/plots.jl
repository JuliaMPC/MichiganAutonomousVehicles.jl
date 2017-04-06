function plotTrack(c::Case)
  pp=plot(0,leg=:false);
  title!(@sprintf("min D (m) = %0.2f",minD(c)))
  pts = Plots.partialcircle(0,2π,100,c.R);
  x, y = Plots.unzip(pts)
  x += t.x0;
  y += t.y0;
  pts = collect(zip(x, y));
  plot!(pts,line=(:path,5,:solid,:green),label="Track");
  XX=[c.Xn;c.x0];YY=[c.Yn;c.y0];
  plot!(XX,YY,line=(:path,5,:solid),label="D # ");

  pts = Plots.partialcircle(0,2π,100,0.25);
  x, y = Plots.unzip(pts);
  x += c.Xn;  y += c.Yn;
  pts = collect(zip(x, y));
  plot!(pts,c=:black,fill=:true,label="Vehicle")
  #xlabel!("X (m)");  ylabel!("Y (m)");
  #xlims!(-10,10); ylims!(-15,10); #TODO --> use this as a position plot
end

"""
pp=initializePlot(n)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/17/2017, Last Modified: 2/17/2017 \n
--------------------------------------------------------------------------------------\n
"""
function initializePlot(n)
  X0_obs = [0.0,-2.5,70.0,200.0,319.0,235.0];  #TODO PASS THIS YINGSHI!!!
  Y0_obs = [210.0,330.0,430.0,452.0,550.0,701.0];
  A = [1.5,2.0,1.5,1.5,2,1.5]; B=A; Q=length(A);
  pp=plot(0)
  for i in 1:Q
    pts = Plots.partialcircle(0,2π,100,A[i])
    x, y = Plots.unzip(pts)
    x += X0_obs[i];
    y += Y0_obs[i];
    pts = collect(zip(x, y))
    if i==1
      plot!(Shape(x,y), c=:red,leg=true,label="Obstacles",leg=:bottomleft)
    else
      plot!(Shape(x,y), c=:red,leg=true,label= "",leg=true,leg=:bottomleft)
    end
  end
  xaxis!(n.state.description[1]);
  yaxis!(n.state.description[2]);
  return pp
end



"""
plotP(r,s,pp,t_feas,pass)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/17/2017, Last Modified: 2/17/2017 \n
--------------------------------------------------------------------------------------\n
"""
function plotP(r,s,pp,t_feas,pass)
  idx=1;LL=5;
  plot!(r.dfs_plant[idx][n.state.name[1]],r.dfs_plant[idx][n.state.name[2]],line=(5,:solid,:blue));
  xlims!(maximum(r.X[:,1])-LL,minimum(r.X[:,1])+LL);
  ylims!(maximum(r.X[:,2])-LL,minimum(r.X[:,2])+LL);
  title!(string("\n calcualtion time (s) = ",t_feas,"\n pass = ", Bool(pass), "\n"));
  gui()
end
