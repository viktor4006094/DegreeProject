#pragma once

#define SCENE_ARCADE 0
#define SCENE_TEMPLE 1
#define SCENE_BISTRO 2

#define VERSION_BASELINE  0
#define VERSION_POLARIZED 1
#define VERSION_HYBRID    2

#define MAX_RECURSION_DEPTH 1

#define TEST_NR 0
#define TEST_ITERATION ((TEST_NR/9)+1)
#define ACTIVE_VERSION (TEST_NR%3)
#define ACTIVE_SCENE ((TEST_NR/3)%3)

//#define ACTIVE_VERSION VERSION_BASELINE
//#define ACTIVE_VERSION VERSION_POLARIZED
//#define ACTIVE_VERSION VERSION_HYBRID

//#define ACTIVE_SCENE SCENE_ARCADE
//#define ACTIVE_SCENE SCENE_TEMPLE
//#define ACTIVE_SCENE SCENE_BISTRO

#define RAYS_PER_PIXEL (MAX_RECURSION_DEPTH+1)


#if ACTIVE_VERSION == VERSION_BASELINE
#define VERSION_NAME "Baseline"
#define SHADER_NAME  "Baseline.rt.hlsl"
#elif ACTIVE_VERSION == VERSION_POLARIZED
#define VERSION_NAME "Polarized"
#define SHADER_NAME  "Polarized.rt.hlsl"
#elif ACTIVE_VERSION == VERSION_HYBRID
#define VERSION_NAME "Hybrid"
#define SHADER_NAME  "Hybrid.rt.hlsl"
#endif


#if ACTIVE_SCENE == SCENE_ARCADE
#define SCENE_FILE "PerformanceTest/PerfTest_Arcade1Light.fscene"
#define SCENE_NAME "Arcade"
#elif ACTIVE_SCENE == SCENE_TEMPLE
#define SCENE_FILE "PerformanceTest/PerfTest_SunTemple2Lights.fscene"
#define SCENE_NAME "Temple"
#elif ACTIVE_SCENE == SCENE_BISTRO
#define SCENE_FILE "PerformanceTest/PerfTest_BistroExterior8Lights.fscene"
#define SCENE_NAME "Bistro"
#endif

#define WINDOW_TITLE VERSION_NAME " version. " SCENE_NAME " scene. " 

#define TO_STR__(n) #n
#define TO_STR_(n) TO_STR__(n)
#define TO_STR(n) TO_STR_(n)

//#define PROFILING_FOLDER "../../../PerformanceTests/"
#define PROFILING_FOLDER "../"

// Ex: PolarizedArcade3_run4 for the fourth polarized test on arcade with MAX_RECURSION_DEPTH 3
#define PROFILING_FILE_NAME PROFILING_FOLDER VERSION_NAME SCENE_NAME
