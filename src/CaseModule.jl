module CaseModule

using DataFrames

export
      Case,
      defineCase,
      setMisc!,
      setupResults,
      case2dfs,  # currently not working
      dataSet,
      defineObstacles
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
          0.01);   # w_jx
end

function defineWeights(name)
  if name==:auto
    w=Weights();
  elseif name==:path
    w=Weights();
    w.sr=0.1;
  elseif name==:m3

  elseif name!==:NA
    error("\n Pick a name for weights! \n")
  end
  return w
end
############################### obstacle info ########################################
type Obstacles
  name
  A
  B
  s_x
  s_y
  X0
  Y0
  status
end

function Obstacles()
  Obstacles([],
            [],
            [],
            [],
            [],
            [],
            [],
            []);
end

"""
o=defineObstacles(name)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/28/2017, Last Modified: 4/3/2017 \n
--------------------------------------------------------------------------------------\n
"""
function defineObstacles(name)
  o=Obstacles();
  if name==:auto
    o.name=name;
    o.A=[5.];
    o.B=[5.];
    o.s_x=[0.0];
    o.s_y=[0.0];
    o.X0=[200.];
    o.Y0=[75.];
    o.status=falses(length(o.X0));
  elseif name==:auto2
    o.name=name;
    o.A=[5.,10.,2.];
    o.B=[5.,10.,2.];
    o.s_x=[-2.,0.,0.];
    o.s_y=[0.,0.,4.];
    o.X0=[205.,180.,200.];
    o.Y0=[50.,75.,30.];
    o.status=falses(length(o.X0));
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
    o.X0=dfs[1][:X][1:3];
    o.Y0=dfs[1][:Y][1:3];
    o.A=dfs[1][:A][1:3];
    o.B=dfs[1][:B][1:3];
    o.status=dfs[1][:status][1:3];
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
    g.y_ref=150.;
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
  model  # function name
  X0     # intial state (at the start of the simulation)
  Xlims  # limit for x
  Ylims  # limit for Y
  UX     # vehicle speed (m/s)
  tp     # prediction horizon (s)
  tex    # execution horizon (s)
  max_cpu_time  # maximum time the algorithm has
  sm            # (m) distance to make sure we don't hit obstacle
  Lr            # LiDAR range (m)
  L_rd          # relaxation distance to LiDAR range
  sigma         # 0.05 (m)small margin, if the vehicle is within this margin, then the target is considered to be reached
  Ni       # number of intervals
  Nck      # number of points per interval
  solver   # either :IPOPT or :KNITRO
  max_iter # max evaluations in optimization
  mpc_max_iter # an MPC parameter
  PredictX0    # if the state that the vehicle will be at when the optimization is finished is to be predicted
  FixedTp      # fixed final time for predicting X0
  activeSafety # a bool if the system is only used to avoid collisions with obstacles
  followDriver # a bool to try and follow the driver
  followPath   # a bool to follow the path
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
    m.Lr=100.;
    m.L_rd=1.;
    m.sigma=1.0;
    m.Ni=4;
    m.Nck=[12,10,8,6];
    m.solver=:KNITRO;
    m.max_iter=50;
    m.mpc_max_iter=600;
    m.PredictX0=true;
    m.FixedTp=true;
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
    m.sm=2.0;
    m.Lr=100.;
    m.L_rd=1.;
    m.sigma=1.0;
    m.Ni=3;
    m.Nck=[10,8,6];
    m.solver=:KNITRO;
    m.max_iter=50;
    m.mpc_max_iter=30;
    m.PredictX0=true;
    m.FixedTp=true;
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
    m.Lr=60.;
    m.L_rd=1.;
    m.sigma=1.0;
    m.Ni=3;
    m.Nck=[10,8,6];
    m.solver=:KNITRO;
    m.max_iter=450;
    m.mpc_max_iter=600;
    m.PredictX0=true;
    m.FixedTp=true;
    activeSafety=true;
    followPath=false;
    followDriver=false;
  elseif name==:caseStudyPath # test case for testPathFollowing.jl with other model
    m.name=:caseStudy;
    m.model=:ThreeDOFv2;
    m.UX=10.0;
    m.X0=[0.0,0.0, 0.0, 0.0,1.2037,0.0,m.UX,0.0];
    m.Xlims=[-10., 750.]
    m.Ylims=[-10., 200.]
    m.tp=7.0;
    m.tex=0.3;
    m.max_cpu_time=0.25;
    m.sm=2.0;
    m.Lr=100.;
    m.L_rd=1.;
    m.sigma=1.0;
    m.Ni=3;
    m.Nck=[10,8,6];
    m.solver=:KNITRO;
    m.max_iter=300;
    m.mpc_max_iter=30;
    m.PredictX0=true;
    m.FixedTp=true;
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
                Xlims::Vector{Float64}=[-10., 750.],
                Ylims::Vector{Float64}=[-10., 200.],
                UX::Float64=10.0,
                tp::Float64=5.0,
                tex::Float64=0.3,
                max_cpu_time::Float64=0.26,
                sm::Float64=2.0,
                Lr::Float64=100.0,
                L_rd::Float64=1.0,
                sigma::Float64=1.0,
                Ni::Int64=3,
                Nck::Vector{Int64}=[10,8,6],
                solver::Symbol=:KNITRO,
                max_iter::Int64=300,
                mpc_max_iter::Int64=200,
                PredictX0::Bool=false,
                FixedTp::Bool=false,
                activeSafety::Bool=false,
                followPath::Bool=false,
                followDriver::Bool=false);
    c.m.Xlims=Xlims;
    c.m.Ylims=Ylims;
    c.m.UX=UX;
    c.m.tp=tp;
    c.m.tex=tex;
    c.m.max_cpu_time=max_cpu_time;
    c.m.sm=sm;
    c.m.Lr=Lr;
    c.m.L_rd=L_rd;
    c.m.sigma=sigma;
    c.m.Ni=Ni;
    c.m.Nck=Nck;
    c.m.solver=solver;
    c.m.max_iter=max_iter;
    c.m.mpc_max_iter=mpc_max_iter;
    c.m.PredictX0=PredictX0;
    c.m.FixedTp=FixedTp;
    c.m.activeSafety=activeSafety;
    c.m.followPath=followPath;
    c.m.followDriver=followDriver;
    nothing
end

################################################################################
# Model Class
################################################################################
abstract AbstractCase
type Case <: AbstractCase
 name
 g::Goal        # goal data
 o::Obstacles   # obstacle data
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
      Obstacles(),
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
Date Create: 3/11/2017, Last Modified: 4/3/2017 \n
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
     c.o=defineObstacles(c.name);
     c.w=defineWeights(c.name);
     c.t=defineTrack(:NA);
     c.m=defineMisc(c.name);
   elseif mode==:path
     c.name=mode;
     c.g=defineGoal(c.name);
     c.o=defineObstacles(c.name);
     c.w=defineWeights(c.name);
     c.t=defineTrack(c.name);
     c.m=defineMisc(c.name);
   elseif mode==:caseStudy
     c.name=mode;
     c.g=defineGoal(c.name);
     c.o=defineObstacles(c.name);
     c.w=defineWeights(:path);
     c.t=defineTrack(c.name);
     c.m=defineMisc(c.name);
   elseif mode==:caseStudyPath
     c.name=:caseStudy;
     c.g=defineGoal(c.name);
     c.o=defineObstacles(:caseStudyPath);
     c.w=defineWeights(:path);
     c.t=defineTrack(c.name);
     c.m=defineMisc(:caseStudyPath);
   else
     name=:user;
     c.name=name;
     c.g=defineGoal(goal);
     c.o=defineObstacles(obstacles);
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