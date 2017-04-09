
s1 = UDPSocket();
bind(s1,ip"35.2.235.173",36880); # change this ip to the server (or julia code)
s2 = UDPSocket();
bind(s2,ip"35.2.235.173",12000); # change this ip to the server (or julia code)


gains=[30.0,0.15,3.0];


function tune(gains)
  MsgOutString = ' ';
  for j in 1:length(gains)
      MsgOutString = string(MsgOutString,' ',gains[j]);
  end
  send(s2,ip"141.212.141.245",36881,MsgOutString);  # change this to the ip where you are running Simulink!
  print(MsgOutString)
end

tune(gains)
recv(s1)
