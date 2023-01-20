//
// Created by efarhan on 1/20/23.
//

#include "benchmark/benchmark.h"
#include "boids.h"

static void BM_Update1Branch(benchmark::State& state) {
    boids::BoidManager boidManager;
    boidManager.Begin();
    for (auto _ : state) {
        boidManager.Update(0.1f);
    }
}
BENCHMARK(BM_Update1Branch);
static void BM_Update2Branch(benchmark::State& state) {
    boids::BoidManager boidManager;
    boidManager.Begin();
    for (auto _ : state) {
        boidManager.Update2(0.1f);
    }
}
BENCHMARK(BM_Update2Branch);