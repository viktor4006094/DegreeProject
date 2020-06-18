Polarizing Filters in Real-Time Ray Tracing With DXR
====
This repository hosts the source code for my master's thesis titled _Real-Time Ray Tracing With Polarization Parameters_.

Three renderers are implemented in the application:
- **Baseline** uses conventional shading methods without polarization parameters.
- **Polarization** uses polarization parameters in all rays.
- **Hybrid** uses the _Polarization_ version's methods/payloads for all primary rays, and the _Baseline_ version's methods/payloads for all reflection rays.

All three renderers use Whitted-style ray tracing with reflection rays (but no shadow or refraction rays).
The shader code for the three renderers can be found in [`Baseline.rt.hlsl`](https://github.com/viktor4006094/DegreeProject/blob/master/Polarization/Projects/PerformanceTest/Data/Baseline.rt.hlsl), [`Polarized.rt.hlsl`](https://github.com/viktor4006094/DegreeProject/blob/master/Polarization/Projects/PerformanceTest/Data/Polarized.rt.hlsl), and [`Hybrid.rt.hlsl`](https://github.com/viktor4006094/DegreeProject/blob/master/Polarization/Projects/PerformanceTest/Data/Hybrid.rt.hlsl) respectively.
A demo project called `PolarizationDemo` is also included to compare the graphical difference between the different renderers.

For more information about the underlying theory and the implementation, see the included [thesis \[pdf\]](https://github.com/viktor4006094/DegreeProject/blob/master/GitHubMedia/Documents/Thesis.pdf).


Polarizing filter demonstration
----

<img src="https://raw.githubusercontent.com/viktor4006094/DegreeProject/readmeImages/GitHubMedia/Screenshots/PolarizationUnfiltered.jpg" width="425"/>! <img src="https://raw.githubusercontent.com/viktor4006094/DegreeProject/readmeImages/GitHubMedia/Screenshots/PolarizationVer.jpg" width="425"/>



Demo Settings
----
- **Recursion depth** The maximum number of recursive reflection rays per pixel (e.g., with a recursion depth of `2` there will be one primary ray and up to two reflection rays per pixel).
- **Versions**
	- **Basline**
		- [ ] Polarized
	- **Polarization**
		- [x] Polarized
		- [ ] Hybrid
	- **Hybrid**
		- [x] Polarized
		- [x] Hybrid
- **Miss color** The color of the sky and any ray that doesn't hit any scene geometry.
- **Uniform light** Disables all the light sources in the scene and uniformly lights the scene with diffuse light.
- **Attach light** Attaches a point light to the camera (only works if the first light in the scene is a point light).
- **Polarizing filter**
	- **Angle** The rotation angle of the linear polarizing filter in degrees, with `0` repressenting a horizontally angled filter.
- **Polarization Output**
	- **DOP** Degree of Polarization.
	- **Plane** The angle of linear polarization.
	- **TOP** Type of Polarization (_blue_ is linear, _yellow_ is elliptical).
	- **Chirality** Handedness of elliptical polarization (_blue_ is left handed, _yellow_ is right-handed).
	- **Sanity** Should result in a black screen unless there are impossible Stokes vectors in the output image.
	- **Normal** Surface normals in the scene.
- **Materials** Pre-defined metals can be selected from a list. Their complex index of refraction values can also be set manually.


Requirements and Installation
----
Nvidia's [Falcor Rendering Framework](https://github.com/NVIDIAGameWorks/Falcor) was used as the base of this project, so the same restrictions and requirements apply.
Follow the installation guide for [Falcor 3.2.2](https://github.com/NVIDIAGameWorks/Falcor/tree/f2b53b1bb9f8433f3c9e2570d2dc90dcd2440415) if you want to build this project.

Test Scenes
----
Test scenes can be downloaded from the [Open Research Content Archive (ORCA)](https://developer.nvidia.com/orca). You might have to edit some path names in the code to get them to load correctly.

Scenes can also be created/modified with the included `SceneEditor` project.

Versions
-----
- [`v1.01`](https://github.com/viktor4006094/DegreeProject/releases/tag/v1.01) Cleaned up version with a minor (non-performance impacting) bug fix.
- [`v1.00`](https://github.com/viktor4006094/DegreeProject/releases/tag/v1.00) Version used to run the performance tests in the thesis

Citation
--------

### Thesis
```bibtex
@mastersthesis{Enfeldt20thesis,  
   author  = {Enfeldt, Viktor},  
   title   = {Real-Time Ray Tracing With Polarization Parameters},
   school  = {{Blekinge Institute of Technology}},
   address = {Karlskrona, Sweden},
   year    = {2020},  
   month   = {06}
}
```

### Application
```bibtex
@Misc{Enfeldt20application,  
   author = {Enfeldt, Viktor},  
   title  = {Polarizing Filters in Real-Time Ray Tracing With {DXR}},  
   year   = {2020},  
   month  = {06},  
   url    = {https://github.com/viktor4006094/DegreeProject},  
   note   = {Version: 1.01}  
}
```
If you cite the application then remember to also [cite the Falcor Rendering Framework](https://github.com/NVIDIAGameWorks/Falcor/tree/f2b53b1bb9f8433f3c9e2570d2dc90dcd2440415#Citation).
