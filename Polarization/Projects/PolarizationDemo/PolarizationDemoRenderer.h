/***************************************************************************
# Copyright (c) 2018, NVIDIA CORPORATION.
# Copyright (c) 2020, Viktor Enfeldt.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of NVIDIA CORPORATION nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************/

#pragma once
#include "Falcor.h"
#include "FalcorExperimental.h"

/**
Some X Macros to make it easy to add more metals

Arguments
	name: The name of the metal
	nr  : IoR n at 680 nm (red)
	ng  : IoR n at 530 nm (green)
	nb  : IoR n at 470 nm (blue)
	kr  : IoR k at 680 nm (red)
	kg  : IoR k at 530 nm (green)
	kb  : IoR k at 470 nm (blue)
*/
#define EXPAND( x ) x

#define X_NAME_(name, nr, ng, nb, kr, kg, kb, ...) name,
#define X_DROP_(name, nr, ng, nb, kr, kg, kb, ...) { MetalPreset::##name, #name },
#define X_IOR_N_(name, nr, ng, nb, kr, kg, kb, ...) { nr, ng, nb },
#define X_IOR_K_(name, nr, ng, nb, kr, kg, kb, ...) { kr, kg, kb },

#define X_NAME(...)  EXPAND( X_NAME_(__VA_ARGS__) )
#define X_DROP(...)  EXPAND( X_DROP_(__VA_ARGS__) )
#define X_IOR_N(...) EXPAND( X_IOR_N_(__VA_ARGS__) )
#define X_IOR_K(...) EXPAND( X_IOR_K_(__VA_ARGS__) )

// Materials from refractiveindex.info
#define METAL_TABLE \
X(Aluminium, 1.7680f  , 0.93029f , 0.70362f , 7.9819f, 6.3965f, 5.6953f) \
X(Brass,     0.44400f , 0.57300f , 0.90000f , 3.9430f, 2.5680f, 1.9570f) \
X(Copper,    0.22905f , 0.82310f , 1.2438f  , 3.9073f, 2.4763f, 2.2880f) \
X(Gold,      0.13544f , 0.55758f , 1.3148f  , 3.8820f, 2.2039f, 1.8534f) \
X(Iron,      2.8920f  , 2.8889f  , 2.6660f  , 3.1420f, 2.9164f, 2.8175f) \
X(Platium,   2.4710f  , 2.0332f  , 1.8906f  , 4.4177f, 3.6000f, 3.2520f) \
X(Silver,    0.045444f, 0.053285f, 0.049317f, 4.6447f, 3.4101f, 2.8545f)

constexpr float TMIN = 0.001f;
constexpr float TMAX = 100000.0f;

using namespace Falcor;

class PolarizationDemoRenderer : public Renderer
{
public:
	enum DebugOutputType : int32_t
	{
		DOP       = 0,
		Plane     = 1,
		TOP       = 2,
		Chirality = 3,
		Sanity    = 4,
		Normal    = 5,
		PolDbg    = 6,
		None      = 7,
	};


	void onLoad(SampleCallbacks* pSample, RenderContext* pRenderContext) override;
	void onFrameRender(SampleCallbacks* pSample, RenderContext* pRenderContext, const Fbo::SharedPtr& pTargetFbo) override;
	void onResizeSwapChain(SampleCallbacks* pSample, uint32_t width, uint32_t height) override;
	bool onKeyEvent(SampleCallbacks* pSample, const KeyboardEvent& keyEvent) override;
	bool onMouseEvent(SampleCallbacks* pSample, const MouseEvent& mouseEvent) override;
	void onGuiRender(SampleCallbacks* pSample, Gui* pGui) override;

private:
#define X(...) X_NAME(__VA_ARGS__)
	enum MetalPreset : uint32_t
	{
		METAL_TABLE
		NUM_METALS
	};
#undef X

#define X(...) X_DROP(__VA_ARGS__)
	Falcor::Gui::DropdownList mpMetalPresets =
	{
		METAL_TABLE
	};
#undef X

#define X(...) X_IOR_N(__VA_ARGS__)
	glm::vec3 mpMetalPresetsN[MetalPreset::NUM_METALS] = { METAL_TABLE };
#undef X

#define X(...) X_IOR_K(__VA_ARGS__)
	glm::vec3 mpMetalPresetsK[MetalPreset::NUM_METALS] = { METAL_TABLE };
#undef X

	RtScene::SharedPtr       mpScene;
	SceneRenderer::SharedPtr mpSceneRenderer;

	Camera::SharedPtr           mpCamera;
	FirstPersonCameraController mCamController;

	RtProgram::SharedPtr       mpRaytraceProgram = nullptr;
	GraphicsVars::SharedPtr    mpProgramVars = nullptr;
	RtProgramVars::SharedPtr   mpRtVars;
	RtState::SharedPtr         mpRtState;
	RtSceneRenderer::SharedPtr mpRtRenderer;
	Texture::SharedPtr         mpRtOut;

	//
	// GUI stuff
	//
	float mpCamSpeed = 0.25f;
	bool mpLightOnCamera = false;

	glm::vec3 mpMissColor = glm::vec3(0.0f, 0.0f, 0.0f);

	// Material stuff
	uint32_t  mpSelectedMetal = MetalPreset::Gold;
	glm::vec3 mpMetalIoRn     = mpMetalPresetsN[MetalPreset::Gold];
	glm::vec3 mpMetalIoRk     = mpMetalPresetsK[MetalPreset::Gold];
	float     mpNonMetalIoRn  = 1.5f;

	// Polarizing filter
	bool  mpFilterEnabled   = false;
	float mpFilterAngle     = 0.0f;
	float mpFilterCos2Angle = 1.0f;
	float mpFilterSin2Angle = 0.0f;

	int32_t mpMaxRecursionDepth = 3;

	DebugOutputType mpDbgOutputType = DebugOutputType::None;
	bool  mpDbgOutputSwitches[DebugOutputType::None] = { false }; // Needed for imGuI
	float mpTMin         = TMIN;
	float mpTMax         = TMAX;
	bool  mpUniformLight = false;
	bool  mpPolarized    = true;

	// For profiling
	IDXGIAdapter3* mpAdapter3 = nullptr;
	DXGI_QUERY_VIDEO_MEMORY_INFO mVidMemInfo;

	void setPerFrameVars(const Fbo* pTargetFbo);
	void loadScene(const std::string& filename, const Fbo* pTargetFbo);
};
