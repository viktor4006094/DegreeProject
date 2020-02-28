/***************************************************************************
# Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
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
#include "BaselineDXR.h"

static const glm::vec4 kClearColor(0.38f, 0.52f, 0.10f, 1);
//static const std::string kDefaultScene = "Arcade/Arcade.fscene";
static const std::string kDefaultScene = "Experimental/arcadeWithCubes.fscene";
//static const std::string kDefaultScene = "PolarizationTest/PolarizationScene218.fscene";

std::string to_string(const vec3& v)
{
	std::string s;
	s += "(" + std::to_string(v.x) + ", " + std::to_string(v.y) + ", " + std::to_string(v.z) + ")";
	return s;
}

void BaselineDXR::onGuiRender(SampleCallbacks* pSample, Gui* pGui)
{
	auto addOutputSwitch = [&, this](const char* label, OutputType selected) {
		if (pGui->addCheckBox(label, mpOutputSwitches[selected]) && mpOutputSwitches[selected]) {
			for (auto& option : mpOutputSwitches) {
				option = false;
			}
			mpOutputSwitches[selected] = true;
		}
	};

	auto setOptionFromSwitches = [&, this]() {
		int32_t selectedIndex = 0;
		for (auto& option : mpOutputSwitches) {
			if (option) {
				mpOutputType = static_cast<OutputType>(selectedIndex);
				return;
			}
			selectedIndex++;
		}
		mpOutputType = OutputType::Result;
	};



	pGui->addCheckBox("Ray Trace", mRayTrace);
	if (pGui->addButton("Load Scene")) {
		std::string filename;
		if (openFileDialog(Scene::kFileExtensionFilters, filename)) {
			loadScene(filename, pSample->getCurrentFbo().get());
		}
	}

	for (uint32_t i = 0; i < mpScene->getLightCount(); i++) {
		std::string group = "Point Light" + std::to_string(i);
		mpScene->getLight(i)->renderUI(pGui, group.c_str());
	}

	if (pGui->beginGroup("Output", true)) {
		addOutputSwitch("Normals", BaselineDXR::OutputType::Normals);
		addOutputSwitch("Specular", OutputType::Specular);
		addOutputSwitch("Diffuse", OutputType::Diffuse);
		addOutputSwitch("Reflectivity", OutputType::Reflectivity);
		addOutputSwitch("Reflections", OutputType::Reflections);

		setOptionFromSwitches();

		pGui->endGroup();
	}


	if (pGui->beginGroup("Rays", true)) {
		// TODO: rename
		pGui->addIntSlider("Recursion depth", mpMaxRecursionDepth, 0, 4, false, 100.0f);
		pGui->addFloatSlider("TMax", mpTMax, 0.0f, TMAX, false, "%.2f");
		pGui->addFloatSlider("TMin", mpTMin, 0.0f, TMIN, false, "%1.6f");
		pGui->addCheckBox("Uniform light", mpUniformLight);

		pGui->addCheckBox("Attach light", mpLightOnCamera);


		pGui->endGroup();
	}

	if (pGui->beginGroup("Camera")) {

		if (pGui->addFloatSlider("Speed", mpCamSpeed, 0.02f, 10.0f, false, "%.2f")) {
			mCamController.setCameraSpeed(mpCamSpeed);
		}



		pGui->endGroup();
	}
}

void BaselineDXR::loadScene(const std::string& filename, const Fbo* pTargetFbo)
{
	mpScene = RtScene::loadFromFile(filename, RtBuildFlags::None, Model::LoadFlags::RemoveInstancing);

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
	mpCamera->setAspectRatio((float)pTargetFbo->getWidth() / (float)pTargetFbo->getHeight());
	mpSceneRenderer = SceneRenderer::create(mpScene);
	mpRtVars = RtProgramVars::create(mpRaytraceProgram, mpScene);
	mpRtRenderer = RtSceneRenderer::create(mpScene);
}

void BaselineDXR::onLoad(SampleCallbacks* pSample, RenderContext* pRenderContext)
{
	if (gpDevice->isFeatureSupported(Device::SupportedFeatures::Raytracing) == false) {
		logErrorAndExit("Device does not support raytracing!", true);
	}

	RtProgram::Desc rtProgDesc;
	rtProgDesc.addShaderLibrary("BaselineDXR.rt.hlsl").setRayGen("rayGen");
	rtProgDesc.addHitGroup(0, "primaryClosestHit", "").addMiss(0, "primaryMiss");
	rtProgDesc.addHitGroup(1, "", "shadowAnyHit").addMiss(1, "shadowMiss");

	mpRaytraceProgram = RtProgram::create(rtProgDesc);

	mpRasterProgram = GraphicsProgram::createFromFile("HelloDXR.ps.hlsl", "", "main");

	loadScene(kDefaultScene, pSample->getCurrentFbo().get());

	mpProgramVars = GraphicsVars::create(mpRasterProgram->getReflector());
	mpGraphicsState = GraphicsState::create();
	mpGraphicsState->setProgram(mpRasterProgram);

	mpRtState = RtState::create();
	mpRtState->setProgram(mpRaytraceProgram);


	//TODO Should be hard-coded for the testing
	// Used to be 3
	// 1 for calling TraceRay from RayGen, 1 for calling it from the primary-ray ClosestHitShader for reflections, 1 for reflection ray tracing a shadow ray
	mpRtState->setMaxTraceRecursionDepth(5);
	//mpRtState->setMaxTraceRecursionDepth(mpMaxRecursionDepth);
}

void BaselineDXR::renderRaster(RenderContext* pContext)
{
	mpGraphicsState->setRasterizerState(nullptr);
	mpGraphicsState->setDepthStencilState(nullptr);
	mpGraphicsState->setProgram(mpRasterProgram);
	pContext->setGraphicsState(mpGraphicsState);
	pContext->setGraphicsVars(mpProgramVars);
	mpSceneRenderer->renderScene(pContext, mpCamera.get());
}

void BaselineDXR::setPerFrameVars(const Fbo* pTargetFbo)
{
	PROFILE("setPerFrameVars");
	GraphicsVars* pVars = mpRtVars->getGlobalVars().get();
	ConstantBuffer::SharedPtr pCB = pVars->getConstantBuffer("PerFrameCB");
	pCB["invView"] = glm::inverse(mpCamera->getViewMatrix());
	pCB["viewportDims"] = vec2(pTargetFbo->getWidth(), pTargetFbo->getHeight());
	float fovY = focalLengthToFovY(mpCamera->getFocalLength(), Camera::kDefaultFrameHeight);
	pCB["tanHalfFovY"] = tanf(fovY * 0.5f);



	pCB = pVars->getConstantBuffer("SettingsCB");
	pCB["maxRecursionDepth"] = mpMaxRecursionDepth;
	pCB["tMin"] = mpTMin;
	pCB["tMax"] = mpTMax;
	pCB["uniformLighting"] = mpUniformLight;
	pCB["outputType"] = static_cast<int32_t>(mpOutputType);

	//TODO move, this only works for point lights
	if (mpLightOnCamera) {
		//mpScene->getLight(0)->move(mpCamera->getPosition(), mpCamera->getTarget(), mpCamera->getUpVector());

		std::dynamic_pointer_cast<Falcor::PointLight>(mpScene->getLight(0))->setWorldPosition(mpScene->getActiveCamera()->getPosition());
	}
}

void BaselineDXR::renderRT(RenderContext* pContext, const Fbo* pTargetFbo)
{
	PROFILE("renderRT");
	setPerFrameVars(pTargetFbo);

	pContext->clearUAV(mpRtOut->getUAV().get(), kClearColor);
	mpRtVars->getRayGenVars()->setTexture("gOutput", mpRtOut);

	mpRtRenderer->renderScene(pContext, mpRtVars, mpRtState, uvec3(pTargetFbo->getWidth(), pTargetFbo->getHeight(), 1), mpCamera.get());
	pContext->blit(mpRtOut->getSRV(), pTargetFbo->getRenderTargetView(0));
}

void BaselineDXR::onFrameRender(SampleCallbacks* pSample, RenderContext* pRenderContext, const Fbo::SharedPtr& pTargetFbo)
{
	pRenderContext->clearFbo(pTargetFbo.get(), kClearColor, 1.0f, 0, FboAttachmentType::All);

	if (mpScene) {
		mpGraphicsState->setFbo(pTargetFbo);
		mCamController.update();

		if (mRayTrace) {
			renderRT(pRenderContext, pTargetFbo.get());
		} else {
			renderRaster(pRenderContext);
		}
	}
}

bool BaselineDXR::onKeyEvent(SampleCallbacks* pSample, const KeyboardEvent& keyEvent)
{
	if (mCamController.onKeyEvent(keyEvent)) {
		return true;
	}
	if (keyEvent.key == KeyboardEvent::Key::Space && keyEvent.type == KeyboardEvent::Type::KeyPressed) {
		mRayTrace = !mRayTrace;
		return true;
	}
	return false;
}

bool BaselineDXR::onMouseEvent(SampleCallbacks* pSample, const MouseEvent& mouseEvent)
{
	return mCamController.onMouseEvent(mouseEvent);
}

void BaselineDXR::onResizeSwapChain(SampleCallbacks* pSample, uint32_t width, uint32_t height)
{
	float h = (float)height;
	float w = (float)width;

	mpCamera->setFocalLength(18);
	float aspectRatio = (w / h);
	mpCamera->setAspectRatio(aspectRatio);

	mpRtOut = Texture::create2D(width, height, ResourceFormat::RGBA16Float, 1, 1, nullptr, Resource::BindFlags::UnorderedAccess | Resource::BindFlags::ShaderResource);
}

int WINAPI WinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPSTR lpCmdLine, _In_ int nShowCmd)
{
	BaselineDXR::UniquePtr pRenderer = std::make_unique<BaselineDXR>();
	SampleConfig config;
	config.windowDesc.title = "BaselineDXR";
	config.windowDesc.resizableWindow = true;

	config.windowDesc.width = 960;
	config.windowDesc.height = 540;


	Sample::run(config, pRenderer);
}
