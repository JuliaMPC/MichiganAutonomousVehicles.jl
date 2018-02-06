isdefined(Base, :__precompile__) && __precompile__()

module MAVs
using NLOptControl
using VehicleModels

include("CaseModule.jl")
using .CaseModule

include("AutonomousControl.jl")
#include("SharedControl.jl")
#include("SimpleModel.jl")
#include("PathFollowing.jl")

using .AutonomousControl
#using .SharedControl
#using .SimpleModel
#using .PathFollowing

export

      # CaseModule.jl
      Case,
      defineCase,
      setWeights!,
      setMisc!,
      setupResults,
      case2dfs,  # currently not working
      dataSet,
      Obs,
      defineObs,

      # AutonomousControl.jl
      initializeAutonomousControl,
      updateAutoParams!,
      avMpc

      # SharedControl.jl
    #  initializeSharedControl!,
    #  checkFeasibility!,
    #  sharedControl!,
    #  getPlantData!,
    #  sendOptData!,
    #  ExternalModel,

      # SimpleModel.jl
  #    initializeSimpleModel,
    #  updateSimpleModel,
  #    runSimpleModel,
  #    compareSimpleModels,
#      resetDesignVariables,

      # PathFollowing.jl
    #  initializePathFollowing,
    #  updatePathParams!
end
