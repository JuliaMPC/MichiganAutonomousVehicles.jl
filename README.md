# Michigan Autonomous Vehicles

[![Build Status](https://travis-ci.org/JuliaMPC/MichiganAutonomousVehicles.jl.svg?branch=master)](https://travis-ci.org/JuliaMPC/MichiganAutonomousVehicles.jl)
[![MichiganAutonomousVehicles](http://pkg.julialang.org/badges/MichiganAutonomousVehicles_0.6.svg)](http://pkg.julialang.org/detail/MichiganAutonomousVehicles)

This package uses NLOptControl and VehicleModels to solve NMPC problems.

julia packages:
```
Pkg.add("PyCall")
ENV["PYTHON"]="/usr/bin/python2.7"; Pkg.build("PyCall");
Pkg.clone("https://github.com/JuliaMPC/MichiganAutonomousVehicles.jl")

```

To test:
```
Pkg.test("MichiganAutonomousVehicles")
```
