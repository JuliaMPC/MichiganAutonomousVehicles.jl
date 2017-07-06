using PkgBenchmark, MAVs, NLOptControl

@benchgroup "psMethods"  ["integrationScheme","Nck"] begin
    for scheme in (:lgrExplicit,:lgrImplicit)
        for Nck in ([10,8,6],[12,10,8,6])
          c=defineCase(;(:mode=>:autoBench));
          setMisc!(c;integrationScheme=scheme,Nck=Nck,max_cpu_time=20.0)
          n=initializeAutonomousControl(c);
          @bench string(scheme), Nck  optimize!($n)
        end
    end
end
