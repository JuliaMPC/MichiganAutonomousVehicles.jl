isdefined(Base, :__precompile__) && __precompile__()

module MAVs

include("CaseModule.jl")
using .CaseModule

include("AutonomousControl.jl")
include("SharedControl.jl")
#include("SimpleModel.jl")
include("PathFollowing.jl")

using .AutonomousControl
using .SharedControl
#using .SimpleModel
using .PathFollowing

export

      # CaseModule.jl
      Case,
      defineCase,
      setMisc!,
      setupResults,
      case2dfs,  # currently not working
      dataSet,
      defineObstacles,

      # AutonomousControl.jl
      initializeAutonomousControl,
      autonomousControl!,
      updateAutoParams!,

      # SharedControl.jl
      initializeSharedControl!,
      checkFeasibility!,
      sharedControl!,
      getPlantData!,
      sendOptData!,
      ExternalModel,

      # SimpleModel.jl
  #    initializeSimpleModel,
    #  updateSimpleModel,
  #    runSimpleModel,
  #    compareSimpleModels,
#      resetDesignVariables,

      # PathFollowing.jl
      initializePathFollowing,
      updatePathParams!
end
