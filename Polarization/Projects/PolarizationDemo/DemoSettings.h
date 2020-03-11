#pragma once

#define SCENE_ARCADE 1
#define SCENE_TEMPLE 2
#define SCENE_BISTRO 3
#define SCENE_EXPERIMENTAL 4

#define ACTIVE_SCENE   SCENE_EXPERIMENTAL

#if ACTIVE_SCENE == SCENE_ARCADE
#define SCENE_FILE "PerformanceTest/PerfTest_Arcade1Light.fscene"
#define SCENE_NAME "Arcade"
#elif ACTIVE_SCENE == SCENE_TEMPLE
#define SCENE_FILE "PerformanceTest/PerfTest_SunTemple2Lights.fscene"
#define SCENE_NAME "Temple"
#elif ACTIVE_SCENE == SCENE_BISTRO
#define SCENE_FILE "PerformanceTest/PerfTest_BistroExterior8Lights.fscene"
#define SCENE_NAME "Bistro"
#elif ACTIVE_SCENE == SCENE_EXPERIMENTAL
#define SCENE_FILE "Experimental/arcadeWithCubes.fscene"
#define SCENE_NAME "Arcade Experimental"
#endif

#define WINDOW_TITLE "Polarization demo. " SCENE_NAME " scene. " 
