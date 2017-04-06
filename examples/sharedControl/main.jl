using NLOptControl
using MAVs

# initialization data -> TODO Yingshi should pass all of this
#ObsX = [0.1,-3.9602,-31.8776]; ObsY = [30.5,99.8831,194.783]; ObsR = [1.0,1.0,1.0]; UX = 10.0;

# initializatin data
c=defineCase(;track=:NA,obstacles=:o3);
c.m=setMisc(;sm=2.0,tex=0.2,tp=3.0,X0=[0.0, 0.0, 0.0, 0.0, pi/2],UX=10.0);

mdl,d,n,r,params = initializeSharedControl(c); # initialize problem

function run()
  # set up the UDP communication
  s1 = UDPSocket();bind(s1,ip"141.212.135.219",36880); # change this ip to the server (or julia code)
  s2 = UDPSocket();bind(s2,ip"141.212.135.219",12000); # change this ip to the server (or julia code)

  s=Settings(;reset=true,save=false);
  OA(s,s1,s2) # call OA "block"
end
