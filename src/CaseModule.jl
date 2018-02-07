module CaseModule

using DataFrames

export
      Case,
      defineCase,
      setWeights!,
      setMisc!,
      setupResults,
      case2dfs,  # currently not working
      dataSet,
      Obs,
      defineObs
################################################################################
# Basic Types
################################################################################
############################### track info ########################################
type Track
  name
  func
  dir
  a
  b
  c
  y0
  Y
  X
end

function Track()
Track([],
      [],
      [],
      [],
      [],
      [],
      [],
      [],
      []);
end

function defineTrack(name)
  t=Track();
  if name==:path # test case for testPathFollowing.jl
    t.name=name;
    t.func=:poly;
    t.dir=:posY
    t.a=[0.0,0.0,2.33e-3,-6.43e-6,5.07e-9];
    t.Y=0:0.1:250;
    t.X=0:0.1:65;
  elseif name==:caseStudy  # TODO consider adding an @NLexpression for the path
    t.name=name;
    t.func=:fourier;
    t.dir=:posX
    dfs=dataSet(name,Pkg.dir("MAVs/examples/Cases/Tracks/"));
    t.a=dfs[1][:a][1:8];
    t.b=dfs[1][:b][1:8];
    t.c=dfs[1][:c][1:8];
    t.y0=dfs[1][:y0][1];
    t.Y=dfs[1][:Y];
    t.X=dfs[1][:X];
  end
  return t
end

############################### weights ########################################
type Weights
  goal  # should be zero when vehicle is not within goal distance during prediction horizon
  psi
  time
  haf
  Fz
  ce
  sa
  sr
  jx
  path
  driver
end

function Weights()
  Weights(1.,      # w_goal
          0.01,    # w_psi
          0.05,    # w_time
          1.0e-5,  # w_haf
          0.1,     # w_Fz  0.5 in paper
          1.,      # w_ce
          0.1,     # w_sa
          1.,      # w_sr
          0.01,    # w_jx
          1.0,     # path following
          1.0,     # follow driver steering angle
          );
end

function defineWeights(name)
  if name==:auto
    w=Weights();
  elseif name==:path
    w=Weights();
    w.sr=0.1;
    w.path=1.0;
    w.driver=1.0;
  elseif name==:m3

  elseif name!==:NA
    error("\n Pick a name for weights! \n")
  end
  return w
end

"""
setWeights!(c;sr=0.08,path=10.0,driver=0.5)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 4/12/2017, Last Modified: 4/12/2017 \n
--------------------------------------------------------------------------------------\n
"""
function setWeights!(c;
                      sr=c.w.sr,
                      path=c.w.path,
                      driver=c.w.driver
                      )
    c.w.sr=sr;
    c.w.path=path;
    c.w.driver=driver;
    return nothing
end
############################### obstacle info ########################################
type Obs
  name
  A
  B
  s_x
  s_y
  X0
  Y0
  status
end

function Obs()
  Obs([],
            [],
            [],
            [],
            [],
            [],
            [],
            []);
end

"""
o=defineObs(name)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/28/2017, Last Modified: 7/5/2017 \n
--------------------------------------------------------------------------------------\n
"""
function defineObs(name)
  o=Obs();
  if name==:auto
    o.name=name;
    o.A=[5.];
    o.B=[5.];
    o.s_x=[0.0];
    o.s_y=[0.0];
    o.X0=[200.];
    o.Y0=[75.];
    o.status=falses(length(o.X0));
  elseif name==:autoBench
    o.name=name;
    o.A=[10.];
    o.B=[5.];
    o.s_x=[0.];
    o.s_y=[0.];
    o.X0=[200.];
    o.Y0=[43.];
    o.status=falses(length(o.X0));
  elseif name==:s1
    o.name=name;
    o.A=[1.] # 5
    o.B=[1.] # 5
    o.s_x=[0.]
    o.s_y=[0.]
    o.X0=[200.]
    o.Y0=[50.]
    o.status=falses(length(o.X0));
  elseif name==:s2
    o.name=name;
    o.A=[1.]
    o.B=[1.]
    o.s_x=[2.]
    o.s_y=[0.]
    o.X0=[190]
    o.Y0=[50]
    o.status=falses(length(o.X0));
  elseif name==:s3
    o.name=name;
    o.A=[5.,10.,2.]
    o.B=[5.,10.,2.]
    o.s_x=[0.,0.,0.]
    o.s_y=[0.,0.,0.]
    o.X0=[205.,180.,200.]
    o.Y0=[50.,75.,30.]
    o.status=falses(length(o.X0));
  elseif name==:s4
    o.name=name;
    o.A=[5,4,2]
    o.B=[5,4,2]
    o.s_x=[-2,-1,0]
    o.s_y=[0,1,4.5]
    o.X0=[205,180,200]
    o.Y0=[57,75,30]
    o.status=falses(length(o.X0));
  elseif name==:s5
    o.name=name;
    A=[2.5 1.5];      # small car
    B=[3.3 1.9];      # HUMMVEEs
    C=[9.8/2 3.65/2]; # tanks
    random=[0 0.6 1 0.25 0.6 0.5495 0.4852 0.8905 0.7990 1];
    o.X0=[225,225,220,170,220,165,200];
    o.Y0=[25,30,45,60,75,95,50];
    o.A=[B[1],B[1],B[1],C[1],A[1],C[1],C[2]];
    o.B=[B[2],B[2],B[2],C[2],A[2],C[2],C[1]];
    o.s_x=[-5,-5.5,-6,6,-5,4,-2];
    o.s_y=[0,0,0,0,0,0,5];
    o.status=falses(length(o.X0));
  elseif name==:s6
    o.name=name;
    o.A=[10,2]
    o.B=[10,2]
    o.s_x=[0,0]
    o.s_y=[0,4]
    o.X0=[180,200]
    o.Y0=[75,30]
    o.status=falses(length(o.X0));
  elseif name==:autoGazebo
    o.name=name;
    o.A=[1.];
    o.B=[1.];
    o.s_x=[0.];
    o.s_y=[0.];
    o.X0=[200.];
    o.Y0=[50.];
    o.status=falses(length(o.X0));
  elseif name==:autoARC
    o.name=name;
    A=[2.5 1.5];      # small car
    B=[3.3 1.9];      # HUMMVEEs
    C=[9.8/2 3.65/2]; # tanks
    random=[0 0.6 1 0.25 0.6 0.5495 0.4852 0.8905 0.7990 1];
    o.X0=[225,225,220,170,220,165,200];
    o.Y0=[25,30,45,60,75,95,50];
    o.A=[B[1],B[1],B[1],C[1],A[1],C[1],C[2]];
    o.B=[B[2],B[2],B[2],C[2],A[2],C[2],C[1]];
    o.s_x=[-5,-5.5,-6,6,-5,4,-2];
    o.s_y=[0,0,0,0,0,0,5];
  elseif name==:path  # test case for testPathFollowing.jl
    o.name=name;
    o.A=[5.,4.,10.]
    o.B=[5.,5.,6.]
    o.s_x=[0.,0.,0.]
    o.s_y=[0.,0.,0.]
    o.X0=[7.0648272,17.37,39.98];
    o.Y0=[60.,130,173]
    o.status=falses(length(o.X0));
  elseif name==:o3  # test case for testSharedControl.jl
    o.name=name;
    o.A=[1.5,2.0,1.5]
    o.B=[1.5,2.0,1.5]
    o.s_x=[0.,0.,0.]
    o.s_y=[0.,0.,0.]
    o.X0=[0.0,-2.5,10.]
    o.Y0=[40.,30.,75.]
    o.status=falses(length(o.X0));
  elseif name==:o4  # test case for testPathFollowing.jl
    o.name=name;
    o.A=[5.,10.]
    o.B=[7.5,3.]
    o.s_x=[0.,0.]
    o.s_y=[0.,0.]
    o.X0=[17.,49.];
    o.Y0=[100.,200.];
    o.status=falses(length(o.X0));
  elseif name==:caseStudyPath # test case for testPathFollowing.jl
    o.name=:caseStudy;
    dfs=dataSet(o.name,Pkg.dir("MAVs/examples/Cases/Obstacles/"));
    o.X0=dfs[1][:X];
    o.Y0=dfs[1][:Y];
    o.A=dfs[1][:A];
    o.B=dfs[1][:B];
    o.status=dfs[1][:status];
    o.s_x=zeros(length(o.X0));
    o.s_y=zeros(length(o.X0));
    o.status=falses(length(o.X0));
  elseif name==:caseStudy # test case for testPathFollowing.jl
    o.name=:caseStudy;
    dfs=dataSet(o.name,Pkg.dir("MAVs/examples/Cases/Obstacles/"));
    o.X0=dfs[1][:X][1];
    o.Y0=dfs[1][:Y][1];
    o.A=dfs[1][:A][1];
    o.B=dfs[1][:B][1];
    o.status=dfs[1][:status][1];
    o.s_x=zeros(length(o.X0));
    o.s_y=zeros(length(o.X0));
    o.status=falses(length(o.X0));
  elseif name!==:NA
    error("\n Pick a name for obstacle data! \n")
  end
  return o
end

############################### goal info ########################################
type Goal
    name
    x_ref
    y_ref
    psi_ref
end

function Goal()
       Goal([],
            [],
            [],
            []);
end

"""
defineGoal(name)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/28/2017, Last Modified: 3/28/2017 \n
--------------------------------------------------------------------------------------\n
"""
function defineGoal(name)
  if name==:auto
    g=Goal();
    g.name=name;
    g.x_ref=200.;
    g.y_ref=125.;
    g.psi_ref=pi/2;
  elseif name==:autoBench
    g=Goal();
    g.name=name;
    g.x_ref=200.;
    g.y_ref=100.;
    g.psi_ref=pi/2;
  elseif name==:autoGazebo
    g=Goal();
    g.name=name;
    g.x_ref=200.;
    g.y_ref=100.;
    g.psi_ref=pi/2;
  elseif name==:path # test case for testPathFollowing.jl
    g=Goal();
    g.name=name;
    g.x_ref=65.0;
    g.y_ref=250.0;
    g.psi_ref=pi/2; #NA
  elseif name==:caseStudy
    g=Goal();
    g.name=name;
    g.x_ref=702.372760886434;
    g.y_ref=120.986699170193;
    g.psi_ref=pi/2;
  elseif name==:NA # g is already defined
    g=Goal();
    g.name=name;
  else
    error("\n Pick a name for goal data! \n")
  end
  return g
end

############################### misc info ########################################
type Misc
  name
  model              # function name
  X0                 # intial state (at the start of the simulation)
  Xlims              # limit for x
  Ylims              # limit for Y
  UX                 # vehicle speed (m/s)
  tp                 # prediction horizon (s)
  tex                # execution horizon (s)
  max_cpu_time       # maximum time the algorithm has
  sm                 # (m) distance to make sure we don't hit obstacle
  Lr                 # LiDAR range (m)
  L_rd               # relaxation distance to LiDAR range
  sigma              # 0.05 (m)small margin, if the vehicle is within this margin, then the target is considered to be reached
  Nck                # number of points per interval
  N                  # number of points in :tm methods
  solver             # either :IPOPT or :KNITRO
  max_iter           # max evaluations in optimization
  mpc_max_iter       # an MPC parameter
  PredictX0          # if the state that the vehicle will be at when the optimization is finished is to be predicted
  FixedTp            # fixed final time for predicting X0
  activeSafety       # a bool if the system is only used to avoid collisions with obstacles
  followDriver       # a bool to try and follow the driver
  followPath         # a bool to follow the path
  NF                 # number to subtract off the constraint check vector-> used for activeSafety mode so driver sees obstacle before autonomy
  integrationScheme
end

function Misc()
       Misc([],
            [],
            [],
            [],
            [],
            0.0,
            0.0,
            [],
            [],
            [],
            [],
            [],
            [],
            [],
            [],
            [],
            [],
            [],
            [],
            [],
            [],
            [],
            [],
            [],
            []
            );
end

"""
defineMisc(name)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/28/2017, Last Modified: 4/3/2017 \n
--------------------------------------------------------------------------------------\n
"""
function defineMisc(name)
  m=Misc();
  if name==:auto
    m.name=name;
    m.model=:ThreeDOFv2;
    m.X0=[200.0, 0.0, 0.0, 0.0, pi/2,0.0,7.0,0.0];
    m.Xlims=[-1., 400.]
    m.Ylims=[-1., 400.]
    m.tex=0.5;
    m.max_cpu_time=m.tex;
    m.sm=2.0;
    m.sigma=1.0;
    m.Nck=[12,10,8,6];
    m.solver=:KNITRO;
    m.max_iter=500;
    m.mpc_max_iter=600;
    m.PredictX0=true;
    m.FixedTp=true;
    m.Lr=50.;
    m.L_rd=5.;
    m.integrationScheme=:lgrExplicit
  elseif name==:autoBench
    m.name=name;
    m.model=:ThreeDOFv2;
    m.X0=[200.0, 0.0, 0.0, 0.0, pi/2,0.0,15.0,0.0];
    m.Xlims=[111.,250.]
    m.Ylims=[-1., 140.]
    m.tex=0.5;
    m.max_cpu_time=0.47;#0.41;
    m.sm=5.0;
    m.sigma=1.0;
    m.Nck=[10,8,6];#[12,10,8,6];
    m.solver=:Ipopt;
    m.max_iter=500;
    m.mpc_max_iter=60;
    m.PredictX0=true;
    m.FixedTp=true;
    m.Lr=150. # not in play
    m.L_rd=5.;
    m.integrationScheme=:lgrExplicit
  elseif name==:autoGazebo
    m.name=name;
    m.model=:ThreeDOFv2;
    m.X0=[200.0, 0.0, 0.0, 0.0, pi/2, 0.0, 0.0, 0.0];
    m.Xlims=[111.,250.]
    m.Ylims=[-1., 140.]
    m.tex=0.5;
    m.max_cpu_time=0.47;#0.41;
    m.sm=5.0;
    m.sigma=1.0;
    m.Nck=[10,8,6];#[12,10,8,6];
    m.solver=:KNITRO;
    m.max_iter=500;
    m.mpc_max_iter=60;
    m.PredictX0=true;
    m.FixedTp=true;
    m.Lr= 50. # not in play
    m.L_rd=5.;
    m.integrationScheme=:lgrExplicit
  elseif name==:autoARC
    m.name=name;
    m.model=:ThreeDOFv2;
    m.X0=[200.0, 0.0, 0.0, 0.0, pi/2,0.0,17.0,0.0];
    m.Xlims=[111.,250.]
    m.Ylims=[-1., 140.]
    m.tex=0.5;
    m.max_cpu_time=0.47;#0.41;
    m.sm=2.6;
    m.sigma=1.0;
    m.Nck=[10,8,6];#[12,10,8,6];
    m.solver=:KNITRO;
    m.max_iter=500;
    m.mpc_max_iter=600;
    m.PredictX0=true;
    m.FixedTp=true;
    m.Lr=50.
    m.L_rd=5.;
    m.integrationScheme=:lgrExplicit
  elseif name==:RTPP
    m.name=name;
    m.model=:ThreeDOFv2;
    m.X0=[200.0, 0.0, 0.0, 0.0, pi/2, 0.0, 17.0, 0.0];
    m.Xlims=[111.,250.]
    m.Ylims=[-1., 140.]
    m.tex=0.5;
    m.max_cpu_time=300.;
    m.sm=2.6;
    m.sigma=1.0;
    #m.Nck=[10,8,6];#[12,10,8,6];
    #m.n.N=
    #m.solver=:KNITRO;
    m.max_iter=500;
    m.mpc_max_iter=60;
    m.PredictX0=true;
    m.FixedTp=true;
    m.Lr= 50. # not in play
    m.L_rd=5.;
  #  m.integrationScheme=:lgrExplicit
  elseif name==:path
    m.name=name;
    m.model=:ThreeDOFv2;
    m.UX=10.0;
    m.X0=[0.0, 0.0, 0.0, 0.0, pi/2,0.0,m.UX,0.0];
    m.Xlims=[-1., 100.]
    m.Ylims=[-1., 300.]
    m.tp=6.0;
    m.tex=0.5;
    m.max_cpu_time=m.tex;
    m.sm=2.6;
    m.sigma=1.0;
    m.Nck=[10,8,6];
    m.solver=:KNITRO;
    m.max_iter=500;
    m.mpc_max_iter=30;
    m.PredictX0=true;
    m.FixedTp=true;
    m.integrationScheme=:lgrExplicit
  elseif name==:caseStudy
    m.name=name;
    m.model=:ThreeDOFv2;
    m.UX=10.0;
    m.X0=[0.0, 0.0, 0.0, 0.0, 1.2037,0.0,m.UX,0.0];
    m.Xlims=[-10., 750.]
    m.Ylims=[-10., 200.]
    m.tp=6.0;
    m.tex=0.3;
    m.max_cpu_time=0.25;  #NOTE:this needs to match Matlab model!
    m.sm=2.0;
    m.sigma=1.0;
    m.Nck=[10,8,6];
    m.solver=:KNITRO;
    m.max_iter=450;
    m.mpc_max_iter=600;
    m.PredictX0=true;
    m.FixedTp=true;
    m.activeSafety=true;
    m.followPath=false;
    m.followDriver=false;
    m.NF=0;
    m.integrationScheme=:lgrExplicit
  elseif name==:caseStudyPath # test case for testPathFollowing.jl with other model
    m.name=:caseStudy;
    m.model=:ThreeDOFv2;
    m.UX=10.0;
    m.X0=[0.0,0.0, 0.0, 0.0,1.2037,0.0,m.UX,0.0];
    m.Xlims=[-10., 730.]
    m.Ylims=[-10., 200.]
    m.tp=7.0;
    m.tex=0.3;
    m.max_cpu_time=0.25;
    m.sm=2.0;
    m.sigma=1.0;
    m.Nck=[10,8,6];
    m.solver=:KNITRO;
    m.max_iter=300;
    m.mpc_max_iter=30;
    m.PredictX0=true;
    m.FixedTp=true;
    m.followPath=true;
    m.integrationScheme=:lgrExplicit
  elseif name!==:NA
    error("\n Pick a name for misc data! \n")
  end
  return m
end

"""
setMisc!(c;activeSafety=true,followPath=false,followDriver=false,predictX0=false,fixedTp=false,tp=5.0,tex=0.3,max_cpu_time=0.26,Ni=3,Nck=[10,8,6]);
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/28/2017, Last Modified: 4/7/2017 \n
--------------------------------------------------------------------------------------\n
"""
function setMisc!(c;
                Xlims=c.m.Xlims,
                Ylims=c.m.Ylims,
                tp=c.m.tp,
                tex=c.m.tex,
                max_cpu_time=c.m.max_cpu_time,
                sm=c.m.sm,
                Lr=c.m.Lr,
                L_rd=c.m.L_rd,
                sigma=c.m.sigma,
                Nck=c.m.Nck,
                N=c.m.N,
                solver=c.m.solver,
                max_iter=c.m.max_iter,
                mpc_max_iter=c.m.mpc_max_iter,
                PredictX0=c.m.PredictX0,
                FixedTp=c.m.FixedTp,
                activeSafety=c.m.activeSafety,
                followPath=c.m.followPath,
                followDriver=c.m.followDriver,
                NF=c.m.NF,
                integrationScheme=c.m.integrationScheme);
    c.m.Xlims=Xlims;
    c.m.Ylims=Ylims;
    c.m.tp=tp;
    c.m.tex=tex;
    c.m.max_cpu_time=max_cpu_time;
    c.m.sm=sm;
    c.m.Lr=Lr;
    c.m.L_rd=L_rd;
    c.m.sigma=sigma;
    c.m.Nck=Nck;
    c.m.N=N;
    c.m.solver=solver;
    c.m.max_iter=max_iter;
    c.m.mpc_max_iter=mpc_max_iter;
    c.m.PredictX0=PredictX0;
    c.m.FixedTp=FixedTp;
    c.m.activeSafety=activeSafety;
    c.m.followPath=followPath;
    c.m.followDriver=followDriver;
    c.m.NF=NF;
    c.m.integrationScheme=integrationScheme;
    return nothing
end

################################################################################
# Model Class
################################################################################
abstract type AbstractCase end
type Case <: AbstractCase
 name
 g::Goal        # goal data
 o::Obs   # obstacle data
 w::Weights     # weight data
 t::Track       # track data
 m::Misc        # miscelaneous data
end

"""
c = Case();
# default constructor
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/11/2017, Last Modified: 3/28/2017 \n
--------------------------------------------------------------------------------------\n
"""
function Case()
 Case(Any,
      Goal(),
      Obs(),
      Weights(),
      Track(),
      Misc()
    );
end

"""
# a basic example of autonomousControl
c=defineCase(;(mode=>:auto));


# a basic example of PathFollowing
c=defineCase(;(mode=>:path));


--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/11/2017, Last Modified: 7/4/2017 \n
--------------------------------------------------------------------------------------\n
"""
function defineCase(;name::Symbol=:auto,
                    goal::Symbol=:auto,
                    obstacles::Symbol=:auto,
                    weights::Symbol=:auto,
                    track::Symbol=:NA,
                    misc::Symbol=:auto, kwargs...)

  kw = Dict(kwargs);
  if !haskey(kw,:mode); kw_ = Dict(:mode => :auto); mode = get(kw_,:mode,0);
  else; mode = get(kw,:mode,0);
  end

   c=Case();
   if mode==:auto
     c.name=mode;
     c.g=defineGoal(c.name);
     c.o=defineObs(c.name);
     c.w=defineWeights(c.name);
     c.t=defineTrack(:NA);
     c.m=defineMisc(c.name);
   elseif mode==:autoBench
     c.name=mode;
     c.g=defineGoal(c.name);
     c.o=defineObs(c.name);
     c.w=defineWeights(:auto);
     c.t=defineTrack(:NA);
     c.m=defineMisc(c.name);
   elseif mode==:autoGazebo
     c.name=mode;
     c.g=defineGoal(c.name);
     c.o=defineObs(c.name);
     c.w=defineWeights(:auto);
     c.t=defineTrack(:NA);
     c.m=defineMisc(c.name);
   elseif mode==:autoARC
     c.name=:auto;
     c.g=defineGoal(c.name);
     c.o=defineObs(:autoARC);
     c.w=defineWeights(c.name);
     c.t=defineTrack(:NA);
     c.m=defineMisc(:autoARC);
   elseif mode==:RTPP
     c.name=mode
     c.g=defineGoal(:autoGazebo);
     c.o=defineObs(:autoGazebo);
     c.w=defineWeights(:auto);
     c.t=defineTrack(:NA);
     c.m=defineMisc(c.name);
   elseif mode==:path
     c.name=mode;
     c.g=defineGoal(c.name);
     c.o=defineObs(c.name);
     c.w=defineWeights(c.name);
     c.t=defineTrack(c.name);
     c.m=defineMisc(c.name);
   elseif mode==:caseStudy
     c.name=mode;
     c.g=defineGoal(c.name);
     c.o=defineObs(c.name);
     c.w=defineWeights(:path);
     c.t=defineTrack(c.name);
     c.m=defineMisc(c.name);
   elseif mode==:caseStudyPath
     c.name=:caseStudy;
     c.g=defineGoal(c.name);
     c.o=defineObs(:caseStudyPath);
     c.w=defineWeights(:path);
     c.t=defineTrack(c.name);
     c.m=defineMisc(:caseStudyPath);
   else
     name=:user;
     c.name=name;
     c.g=defineGoal(goal);
     c.o=defineObs(obstacles);
     c.w=defineWeights(weights);
     c.t=defineTrack(:NA);
     c.m=defineMisc(misc);
   end

   return c
end

################################################################################
# data stuff
################################################################################
"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 12/7/2016, Last Modified: 3/28/2017 \n
--------------------------------------------------------------------------------------\n
"""
function dataSet(set,path)
  main_dir=pwd();
  dfs=Vector{DataFrame}(1) # create am empty DataFrame
  cd(path)
  dfs[1]=readtable(string(set,".csv"))
  cd(main_dir)
  return dfs
end

"""
case2dfs(c)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/11/2017, Last Modified: 3/11/2017 \n
--------------------------------------------------------------------------------------\n
"""
# save senario data  # TODO add all options and settings here?
function case2dfs(c)
# TODO think about just saving n,r,s,params, and mdl
 ###########
 # goal data
 x_ref_data = DataFrame(ID=1, x_ref = c.x_ref)
 y_ref_data = DataFrame(ID=1, y_ref = c.y_ref)
 psi_ref_data = DataFrame(ID=1, psi_ref = c.psi_ref)

 s_data = join(x_ref_data,y_ref_data,on=:ID)
 s_data = join(s_data,psi_ref_data,on=:ID)

 ###############
 # obstacle data
 A_data = DataFrame(ID = 1:Q, A = c.A);
 B_data = DataFrame(ID = 1:Q, B = c.B);
 s_x_data = DataFrame(ID = 1:Q, s_x = c.s_x);
 s_y_data = DataFrame(ID = 1:Q, s_y = c.s_y);
 X0_data = DataFrame(ID = 1:Q, X0 = c.X0_obs);
 Y0_data = DataFrame(ID = 1:Q, Y0 = c.Y0_obs);

 obs_data = join(A_data,B_data, on = :ID);
 obs_data = join(obs_data,s_x_data,on =:ID);
 obs_data = join(obs_data,s_y_data,on =:ID);
 obs_data = join(obs_data,X0_data,on =:ID);
 obs_data = join(obs_data,Y0_data,on=:ID);

 return case_data, obs_data
end

end # module
