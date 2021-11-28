// google benchmark
#include <benchmark/benchmark.h>

// stl
#include <memory>
#include <iostream>
#include <algorithm>


namespace dte3607::benchmarking::predef {

    struct GoldDummyBenchmarkF : benchmark::Fixture {

      using benchmark::Fixture::Fixture;
      ~GoldDummyBenchmarkF() override {}

      void SetUp(benchmark::State const&) final {}
      void TearDown(benchmark::State const&) override {}
    };

}   // namespace dte3607::benchmarking::predef



// Qualify predefined fixtures
using namespace dte3607::benchmarking::predef;

// Dummy benchmarks
BENCHMARK_DEFINE_F(GoldDummyBenchmarkF, dummy01)
(benchmark::State& st)
{
  for (auto const& _ : st)
    std::cout << "Hello world!\n";
}

BENCHMARK_DEFINE_F(GoldDummyBenchmarkF, dummy02)
(benchmark::State& st)
{
  for (auto const& _ : st)
    std::cout << "Hello world!" << std::endl;
}

BENCHMARK_MAIN();
