#ifndef DTE3607_PHYSENGINE_FIXTURE_API_H
#define DTE3607_PHYSENGINE_FIXTURE_API_H


#include "bits/types.h"
#include "bits/concepts.h"
#include "solvers/solver_dev_level0.h"



namespace dte3607::physengine::api
{

  // SOLVER
  template <concepts::SolverFixture Fixture_T>
  void solve(Fixture_T& scenario, types::NanoSeconds timestep)
  {
    // Call another solver !_!
    solver_dev::level0::solve(scenario, timestep);
  }
}


#endif // DTE3607_PHYSENGINE_FIXTURE_API_H
