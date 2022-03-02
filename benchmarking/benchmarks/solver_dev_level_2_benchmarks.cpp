// google benchmark
#include <benchmark/benchmark.h>

// stl
#include <memory>
#include <iostream>
#include <algorithm>

#include <physengine/solvers/solver_dev_level2.h>
#include <physengine/bits/fixtures.h>
#include <physengine/bits/types.h>

using namespace std::chrono_literals;


namespace dte3607::benchmarking::predef
{
  size_t BENCHMARK_SIZE = 10;

  struct SolverLevel2BenchmarkF : benchmark::Fixture {

    using benchmark::Fixture::Fixture;
    std::unique_ptr<dte3607::physengine::fixtures::FixtureLevel2> m_scenario;
    ~SolverLevel2BenchmarkF() override {}

    void SetUp(benchmark::State const&) final
    {
      // Create Fixture
      m_scenario
        = std::make_unique<dte3607::physengine::fixtures::FixtureLevel2>();


      // Set external forces
      m_scenario->setGravity({0, 0, 0});


      // make plane
      m_scenario->createFixedInfPlane({-1, 0, 0}, {10, 0, 0});

      m_scenario->createFixedInfPlane({1, 0, 0}, {-10, 0, 0});

      for (int i = 0; i < BENCHMARK_SIZE; i++) {
        // make sphere
        m_scenario->createSphere(1.0, {100, 0, 0}, {0, 0, i * 2.0});
      }
    }

    void TearDown(benchmark::State const&) override { m_scenario.release(); }
  };
}   // namespace dte3607::benchmarking::predef



// Qualify predefined fixtures
using namespace dte3607::benchmarking::predef;

// Dummy benchmarks
BENCHMARK_DEFINE_F(SolverLevel2BenchmarkF, ms17)
(benchmark::State& st)
{
  for ([[maybe_unused]] auto const& _ : st)
    dte3607::physengine::solver_dev::level2::solve(*m_scenario, 17ms);
}

BENCHMARK_DEFINE_F(SolverLevel2BenchmarkF, ms33)
(benchmark::State& st)
{
  for ([[maybe_unused]] auto const& _ : st)
    dte3607::physengine::solver_dev::level2::solve(*m_scenario, 33ms);
}

BENCHMARK_REGISTER_F(SolverLevel2BenchmarkF, ms17);

BENCHMARK_REGISTER_F(SolverLevel2BenchmarkF, ms33);

BENCHMARK_MAIN();
