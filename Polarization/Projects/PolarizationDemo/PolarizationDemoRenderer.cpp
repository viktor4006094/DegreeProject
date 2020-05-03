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

#include "PolarizationDemoRenderer.h"
#include "DemoSettings.h"

#include <sstream>

static const glm::vec4 kClearColor(0.0f, 0.0f, 0.0f, 1);

std::string to_string(const vec3& v)
{
	std::string s;
	s += "(" + std::to_string(v.x) + ", " + std::to_string(v.y) + ", " + std::to_string(v.z) + ")";
	return s;
}

void PolarizationDemoRenderer::onGuiRender(SampleCallbacks* pSample, Gui* pGui)
{
	//
	// Helper lambdas
	//
	auto setMaterialPreset = [&, this](uint32_t selected) {
		mpMetalIoRn = mpMetalPresetsN[selected];
		mpMetalIoRk = mpMetalPresetsK[selected];
	};

	// Polarization debug output types
	auto addDbgOutputSwitch = [&, this](const char* label, DebugOutputType selected) {
		if (pGui->addCheckBox(label, mpDbgOutputSwitches[selected]) && mpDbgOutputSwitches[selected]) {
			for (auto& option : mpDbgOutputSwitches) {
				option = false;
			}
			mpDbgOutputSwitches[selected] = true;
		}
	};

	auto setDbgOptionFromSwitches = [&, this]() {
		int32_t selectedIndex = 0;
		for (auto& option : mpDbgOutputSwitches) {
			if (option) {
				mpDbgOutputType = static_cast<DebugOutputType>(selectedIndex);
				return;
			}
			selectedIndex++;
		}
		mpDbgOutputType = DebugOutputType::None;
	};

	auto formatVRAM = [](UINT64 mem) -> std::string {
		std::ostringstream ss;
		ss.imbue(std::locale(""));

		//ss << (static_cast<float>(mem)/(1024.f)) << " KiB";
		ss << (static_cast<float>(mem) / (1024.f * 1024.f)) << " MiB";
		//ss << (static_cast<float>(mem)/(1024.f*1024.f*1024.f)) << " GiB";

		return ss.str();
	};

	//
	// VRAM Usage
	//
	std::string availableVRAM = "Available: " + formatVRAM(mVidMemInfo.AvailableForReservation);
	std::string budgetVRAM = "Budget: " + formatVRAM(mVidMemInfo.Budget);
	std::string reservedVRAM = "CurrentReservation: " + formatVRAM(mVidMemInfo.CurrentReservation);
	std::string usageVRAM = "CurrentUsage: " + formatVRAM(mVidMemInfo.CurrentUsage);

	pGui->addText("VRAM");
	pGui->addText(availableVRAM.c_str());
	pGui->addText(budgetVRAM.c_str());
	pGui->addText(reservedVRAM.c_str());
	pGui->addText(usageVRAM.c_str());

	//
	// GUI settings
	//

	pGui->addIntSlider("Recursion depth", mpMaxRecursionDepth, 0, 5, false, 150.0f);
	if (pGui->addCheckBox("Polarized", mpPolarized)) {
		if (mpPolarized) {
			mpRaytraceProgram->addDefine("POLARIZED_VERSION");
		} else {
			mpRaytraceProgram->removeDefine("POLARIZED_VERSION");
		}
	}
	if (mpPolarized) {
		if (pGui->addCheckBox("Hybrid", mpHybrid)) {
			if (mpHybrid) {
				mpRaytraceProgram->addDefine("HYBRID_VERSION");
			} else {
				mpRaytraceProgram->removeDefine("HYBRID_VERSION");
			}
		}
	}


	if (pGui->addButton("Load Scene")) {
		std::string filename;
		if (openFileDialog(Scene::kFileExtensionFilters, filename)) {
			loadScene(filename, pSample->getCurrentFbo().get());
		}
	}

	pGui->addRgbColor("Miss color", mpMissColor);
	pGui->addCheckBox("Uniform light", mpUniformLight);
	pGui->addCheckBox("Attach light", mpLightOnCamera);
	if (pGui->addFloatSlider("Camera speed", mpCamSpeed, 0.02f, 10.0f, false, "%.2f")) {
		mCamController.setCameraSpeed(mpCamSpeed);
	}

	if (mpPolarized) {
		if (pGui->beginGroup("Polarizing filter", true)) {
			pGui->addCheckBox("Enable", mpFilterEnabled);
			if (pGui->addFloatSlider("Angle", mpFilterAngle, 0.0, 180)) {
				mpFilterCos2Angle = static_cast<float>(std::cos(static_cast<double>(-mpFilterAngle)* M_PI / 90.0));
				mpFilterSin2Angle = static_cast<float>(std::sin(static_cast<double>(-mpFilterAngle)* M_PI / 90.0));
			}
			pGui->endGroup();
		}

		if (pGui->beginGroup("Polarization Output", true)) {
			//TODO add toggle for overlay here
			pGui->addSeparator();
			addDbgOutputSwitch("DOP", DebugOutputType::DOP);
			addDbgOutputSwitch("Plane", DebugOutputType::Plane);
			addDbgOutputSwitch("TOP", DebugOutputType::TOP);
			addDbgOutputSwitch("Chirality", DebugOutputType::Chirality);
			addDbgOutputSwitch("Sanity", DebugOutputType::Sanity);
			addDbgOutputSwitch("Normal", DebugOutputType::Normal);
			addDbgOutputSwitch("Debug", DebugOutputType::PolDbg);

			setDbgOptionFromSwitches();

			pGui->endGroup();
		}

		if (pGui->beginGroup("Materials", true)) {
			pGui->addText("Metals");
			if (pGui->addDropdown("Presets", mpMetalPresets, mpSelectedMetal)) {
				setMaterialPreset(mpSelectedMetal);
			}

			pGui->addFloat3Var("IoR n", mpMetalIoRn);
			pGui->addFloat3Var("IoR k", mpMetalIoRk);
			pGui->addText("Non-metals");
			pGui->addFloatVar("IoR n", mpNonMetalIoRn);
			pGui->endGroup();
		}
	}

	if (pGui->beginGroup("Rays", true)) {
		pGui->addFloatSlider("TMax", mpTMax, 0.0f, TMAX, false, "%.2f");
		pGui->addFloatSlider("TMin", mpTMin, 0.0f, TMIN, false, "%1.6f");

		pGui->endGroup();
	}

	for (uint32_t i = 0; i < mpScene->getLightCount(); i++) {
		std::string group = "Point Light" + std::to_string(i);
		mpScene->getLight(i)->renderUI(pGui, group.c_str());
	}
}

void PolarizationDemoRenderer::loadScene(const std::string& filename, const Fbo* pTargetFbo)
{
	Falcor::RtBuildFlags buildFlags = RtBuildFlags::FastTrace | RtBuildFlags::AllowCompaction;

	mpScene = RtScene::loadFromFile(filename, buildFlags, Model::LoadFlags::RemoveInstancing);

	Model::SharedPtr pModel = mpScene->getModel(0);
	float radius = pModel->getRadius();

	mpCamera = mpScene->getActiveCamera();
	assert(mpCamera);

	mCamController.attachCamera(mpCamera);

	Sampler::Desc samplerDesc;
	samplerDesc.setFilterMode(Sampler::Filter::Linear, Sampler::Filter::Linear, Sampler::Filter::Linear);
	Sampler::SharedPtr pSampler = Sampler::create(samplerDesc);
	pModel->bindSamplerToMaterials(pSampler);

	// Update the controllers
	mCamController.setCameraSpeed(mpCamSpeed);
	float nearZ = std::max(0.1f, pModel->getRadius() / 750.0f);
	float farZ = radius * 10;
	mpCamera->setDepthRange(nearZ, farZ);
	mpCamera->setAspectRatio(1920.f / 1080.f);
	mpSceneRenderer = SceneRenderer::create(mpScene);
	mpRtVars = RtProgramVars::create(mpRaytraceProgram, mpScene);
	mpRtRenderer = RtSceneRenderer::create(mpScene);
}


void PolarizationDemoRenderer::onLoad(SampleCallbacks* pSample, RenderContext* pRenderContext)
{
	if (gpDevice->isFeatureSupported(Device::SupportedFeatures::Raytracing) == false) {
		logErrorAndExit("Device does not support raytracing!", true);
	}

	RtProgram::Desc rtProgDesc;
	rtProgDesc.addShaderLibrary("PolarizedDemo.rt.hlsl").setRayGen("rayGen");
	rtProgDesc.addHitGroup(0, "primaryClosestHit", "").addMiss(0, "primaryMiss");
	rtProgDesc.addHitGroup(1, "simpleClosestHit", "").addMiss(1, "simpleMiss");
	mpRaytraceProgram = RtProgram::create(rtProgDesc);
	if (mpPolarized) {
		mpRaytraceProgram->addDefine("POLARIZED_VERSION");
	}
	if (mpHybrid) {
		mpRaytraceProgram->addDefine("HYBRID_VERSION");
	}

	mpRtState = RtState::create();
	mpRtState->setProgram(mpRaytraceProgram);

	loadScene(SCENE_FILE, pSample->getCurrentFbo().get());

	// 1 for calling TraceRay from RayGen, the rest for reflections
	mpRtState->setMaxTraceRecursionDepth(10); // up to ?? reflections

	IDXGIFactory4Ptr factoryPtr = nullptr;
	d3d_call(CreateDXGIFactory2(0, IID_PPV_ARGS(&factoryPtr)));
	LUID luid = gpDevice->getApiHandle()->GetAdapterLuid();
	HRESULT hr = factoryPtr->EnumAdapterByLuid(luid, IID_PPV_ARGS(&mpAdapter3));
}

void PolarizationDemoRenderer::setPerFrameVars(const Fbo* pTargetFbo)
{
	PROFILE("setPerFrameVars");
	GraphicsVars* pVars = mpRtVars->getGlobalVars().get();
	ConstantBuffer::SharedPtr pCB = pVars->getConstantBuffer("PerFrameCB");
	pCB["invView"] = glm::inverse(mpCamera->getViewMatrix());
	pCB["viewportDims"] = vec2(pTargetFbo->getWidth(), pTargetFbo->getHeight());
	float fovY = focalLengthToFovY(mpCamera->getFocalLength(), Camera::kDefaultFrameHeight);
	pCB["tanHalfFovY"] = tanf(fovY * 0.5f);

	pCB = pVars->getConstantBuffer("SettingsCB");
	pCB["metalIoRn"] = mpMetalIoRn;
	pCB["nonMetalIoRn"] = mpNonMetalIoRn;
	pCB["metalIoRk"] = mpMetalIoRk;
	pCB["filterCos2A"] = mpFilterCos2Angle;
	pCB["missColor"] = mpMissColor;
	pCB["filterSin2A"] = mpFilterSin2Angle;
	pCB["filterEnabled"] = mpFilterEnabled;

	pCB["maxRecursionDepth"] = mpMaxRecursionDepth;
	pCB["dbgOutputType"] = static_cast<int32_t>(mpDbgOutputType);
	pCB["tMin"] = mpTMin;
	pCB["tMax"] = mpTMax;
	pCB["uniformLighting"] = mpUniformLight;

	// Move point light if attach light is enabled
	if (mpLightOnCamera && mpScene->getLight(0)->getType() == LightPoint) {
		std::dynamic_pointer_cast<Falcor::PointLight>(mpScene->getLight(0))->setWorldPosition(mpScene->getActiveCamera()->getPosition());
	}
}

void PolarizationDemoRenderer::onFrameRender(SampleCallbacks* pSample, RenderContext* pRenderContext, const Fbo::SharedPtr& pTargetFbo)
{
	pRenderContext->clearFbo(pTargetFbo.get(), kClearColor, 1.0f, 0, FboAttachmentType::All);

	if (mpScene) {
		mpAdapter3->QueryVideoMemoryInfo(0, DXGI_MEMORY_SEGMENT_GROUP_LOCAL, &mVidMemInfo);

		mCamController.update();

		auto currentTime = pSample->getCurrentTime();
		mpRtRenderer->update(currentTime);

		setPerFrameVars(pTargetFbo.get());

		pRenderContext->clearUAV(mpRtOut->getUAV().get(), kClearColor);
		mpRtVars->getRayGenVars()->setTexture("gOutput", mpRtOut);

		mpRtRenderer->renderScene(pRenderContext, mpRtVars, mpRtState, uvec3(pTargetFbo->getWidth(), pTargetFbo->getHeight(), 1), mpCamera.get());
		pRenderContext->blit(mpRtOut->getSRV(), pTargetFbo->getRenderTargetView(0));
	}
}

bool PolarizationDemoRenderer::onKeyEvent(SampleCallbacks* pSample, const KeyboardEvent& keyEvent)
{
	if (mCamController.onKeyEvent(keyEvent)) {
		return true;
	}

	return false;
}

bool PolarizationDemoRenderer::onMouseEvent(SampleCallbacks* pSample, const MouseEvent& mouseEvent)
{
	return mCamController.onMouseEvent(mouseEvent);
}

void PolarizationDemoRenderer::onResizeSwapChain(SampleCallbacks* pSample, uint32_t width, uint32_t height)
{
	float h = (float)height;
	float w = (float)width;

	float aspectRatio = (w / h);
	mpCamera->setAspectRatio(aspectRatio);

	mpRtOut = Texture::create2D(width, height, ResourceFormat::RGBA16Float, 1, 1, nullptr, Resource::BindFlags::UnorderedAccess | Resource::BindFlags::ShaderResource);
}

int WINAPI WinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPSTR lpCmdLine, _In_ int nShowCmd)
{
	PolarizationDemoRenderer::UniquePtr pRenderer = std::make_unique<PolarizationDemoRenderer>();
	SampleConfig config;

	const std::string versionString = WINDOW_TITLE;

	config.windowDesc.title = versionString.c_str();
	config.windowDesc.resizableWindow = true;

	config.windowDesc.width = 1920;
	config.windowDesc.height = 1080;


	Sample::run(config, pRenderer);
}
