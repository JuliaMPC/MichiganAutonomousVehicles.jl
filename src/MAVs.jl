# isdefined(Base, :__precompile__) && __precompile__()  -> VehicleModels cannot be precompiled becasue of DifferentialEquations.jl

module MAVs

include("CaseModule.jl")
using .CaseModule

include("AutonomousControl.jl")
include("SharedControl.jl")
include("SimpleModel.jl")
include("PathFollowing.jl")

using .AutonomousControl
using .SharedControl
using .SimpleModel
using .PathFollowing


export

      # CaseModule.jl
      Case,
      defineCase,
      setMisc,
      setupResults,
      case2dfs,  # currently not working
      dataSet,

      # AutonomousControl.jl
      initializeAutonomousControl,
      autonomousControl,

      # SharedControl.jl
      initializeSharedControl,
      sharedControl,
      feasible,
      OA,

      # SimpleModel.jl
      initializeSimpleModel,
      updateSimpleModel,
      runSimpleModel,
      compareSimpleModels,
      resetDesignVariables,

      # PathFollowing.jl
      initializePathFollowing,
      updatePathParams
end
