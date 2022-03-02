include(../cmake/testing_utils.cmake)

SETUP_BENCHMARK_ENV()

if(benchmark_FOUND)


  # MY benchmarks
  set( BENCHMARKS_DIR benchmarks )
  set( BENCHMARKS
    solver_dev_level_2_benchmarks
    )

  ADD_BENCHMARKS(dte3607::physengine ${BENCHMARKS_DIR} BENCHMARKS)

endif(benchmark_FOUND)
