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

RWTexture2D<float4> gOutput;
__import Raytracing;
__import Lights;

#include "../../../Polarization/Projects/PerformanceTest/Settings.h"

#define M_PI     3.14159265358979323846
#define M_PI2    6.28318530717958647692
#define M_INV_PI 0.3183098861837906715

#define COLOR_BLACK  float3(0.0, 0.0, 0.0)
#define COLOR_RED    float3(1.0, 0.0, 0.0)
#define COLOR_GREEN  float3(0.0, 1.0, 0.0)
#define COLOR_BLUE   float3(0.0, 0.0, 1.0)
#define COLOR_CYAN   float3(0.0, 1.0, 1.0)
#define COLOR_YELLOW float3(1.0, 1.0, 0.0)

#define REFL_MISS_COLOR COLOR_BLACK

//**************************************************************************************************
// Constant buffers
//**************************************************************************************************

shared
cbuffer PerFrameCB
{
	float4x4 invView;
	float4x4 invModel;
	float2   viewportDims;
	float    tanHalfFovY;
};

shared
cbuffer SettingsCB
{
	float3 metalIoRn;
	float  nonMetalIoRn;
	float3 metalIoRk;
	float  filterCos2A;
	float3 missColor;
	float  filterSin2A;
	bool   filterEnabled;
};

//**************************************************************************************************
// Ray payload
//**************************************************************************************************

struct Payload
{
	float3 color;
	uint   recursionDepth; // How many reflections deep the payload is
};

Payload initPayload(uint depth)
{
	Payload pay;
	pay.color = COLOR_BLACK;
	pay.recursionDepth = depth;

	return pay;
}

//**************************************************************************************************
// Shading functions
//**************************************************************************************************

/** Lambertian diffuse
*/
float3 Fd_Lambert(float3 color, float NdotL)
{
	return color*M_INV_PI*NdotL;
}

/** GGX Normal Distribution Function (D)
*/
float D_GGX(float alpha, float NdotH)
{
	float a2 = alpha*alpha;
	float d  = ((NdotH*a2 - NdotH)*NdotH + 1.0);
	return a2/(M_PI*d*d);
}

/** Smith-GGX Visibility Function (V)
  V = G/(4*NdotL*NdotV)
*/
float V_SmithGGX(float NdotL, float NdotV, float roughness)
{
	float a2 = roughness*roughness;
	float ggxv = NdotL*sqrt((-NdotV*a2 + NdotV)*NdotV + a2);
	float ggxl = NdotV*sqrt((-NdotL*a2 + NdotL)*NdotL + a2);
	return 0.5/(ggxv + ggxl);
}

/** Schlick's Approximation Fresnel Function (F)
  f90 has been hard-coded to 1.0
*/
float3 F_Schlick(float3 f0, float u)
{
	return f0 + (float3(1.0, 1.0, 1.0) - f0)*pow((1.0 - u), 5.0);
}

/** Cook-Torrance Specular Term
*/
float3 CookTorrance(ShadingData sd, LightSample ls)
{
	float  D = D_GGX(sd.roughness, ls.NdotH);
	float  V = V_SmithGGX(ls.NdotL, sd.NdotV, sd.roughness);
	float3 F = F_Schlick(sd.specular, max(0, ls.LdotH));

	return (V * D * ls.NdotL) * F;
}

/** Calculates the reflection color using the Cook-Torrance specular term
*/
float3 getReflectionColor(ShadingData sd, float3 originW, float3 v, uint hitDepth)
{
	// N is used instead of H since (N == H) for perfect reflections. 
	float NdotV = saturate(dot(sd.N, v));
	
	// Early exit if we're out of rays or if the surface is not facing the ray
	if (hitDepth >= MAX_RECURSION_DEPTH || NdotV <= 0.0) {
		return REFL_MISS_COLOR;
	}

	Payload rPayload = initPayload((hitDepth + 1));

	RayDesc reflectionRay;
	reflectionRay.Origin = originW;
	reflectionRay.Direction = reflect(-v, sd.N);
	reflectionRay.TMin = TMIN;
	reflectionRay.TMax = TMAX;

	float NdotL = saturate(dot(sd.N, reflectionRay.Direction));
	float LdotH = NdotL;
	
	float  D = D_GGX(sd.roughness, 1.0); // NdotH=1.0 since (N == H)
	float  V = V_SmithGGX(NdotL, NdotV, sd.roughness);
	float3 F = F_Schlick(sd.specular, max(0.0, LdotH));

	//saturate to prevent blown out colors
	float3 reflectionBrdf = saturate(D*V*NdotL)*F;
	
	// Send a reflection ray into the scene
	TraceRay(gRtScene, RAY_FLAG_FORCE_OPAQUE, 0xFF, 0, hitProgramCount, 0, reflectionRay, rPayload);

	// Multiply with the reflection BRDF
	rPayload.color *= reflectionBrdf;

	return rPayload.color;
}

/** Primary miss shader
*/
[shader("miss")]
void primaryMiss(inout Payload hitData)
{
	hitData.color = missColor;
}

/** Primary closest hit shader
*/
[shader("closesthit")]
void primaryClosestHit(inout Payload hitData, in BuiltInTriangleIntersectionAttributes attribs)
{
	// Get the hit-point data
	float3 rayOrigW    = WorldRayOrigin();
	float3 rayDirW     = WorldRayDirection();
	float hitT         = RayTCurrent();
	uint triangleIndex = PrimitiveIndex();

	float3 posW = rayOrigW + hitT*rayDirW;
	// prepare the shading data
	VertexOut va = getVertexAttributes(triangleIndex, attribs);
	ShadingData sd = prepareShadingData(va, gMaterial, rayOrigW, 0);

	[unroll]
	for (int i = 0; i < gLightsCount; i++) {
		LightSample ls = evalLight(gLights[i], sd);

		// Only shade surfaces which are hit by the light
		if (ls.NdotL > 0.0 && sd.NdotV > 0.0) {
			/* Diffuse component */
			float3 diffuseComp = ls.diffuse*Fd_Lambert(sd.diffuse.rgb, ls.NdotL);

			/* Specular component */
			float3 specularComp = ls.specular*CookTorrance(sd, ls);

			hitData.color += specularComp;
			hitData.color += diffuseComp;
		}
	}

	hitData.color += sd.emissive;

	// Shoot a reflection ray
	float3 reflectColor = getReflectionColor(sd, posW, -rayDirW, hitData.recursionDepth);
	hitData.color += reflectColor;
}

/** Shader entry point and ray generation
*/
[shader("raygeneration")]
void rayGen()
{
	uint3 launchIndex = DispatchRaysIndex();
	float2 d = (((launchIndex.xy + 0.5)/viewportDims)*2.0 - 1.0);
	float aspectRatio = viewportDims.x/viewportDims.y;

	RayDesc ray;
	ray.Origin = invView[3].xyz;

	// We negate the Z exis because the 'view' matrix is generated using a 
	// Right Handed coordinate system with Z pointing towards the viewer
	// The negation of Z axis is needed to get the rays go out in the direction away fromt he viewer.
	// The negation of Y axis is needed because the texel coordinate system, used in the UAV we write into using launchIndex
	// has the Y axis flipped with respect to the camera Y axis (0 is at the top and 1 at the bottom)
	ray.Direction = normalize((d.x*invView[0].xyz*tanHalfFovY*aspectRatio) - (d.y*invView[1].xyz*tanHalfFovY) - invView[2].xyz);

	ray.TMin = 0.0; // no self 
	ray.TMax = TMAX;

	/* Shoot a ray into the scene from the camera */
	Payload payload = initPayload(0);
	TraceRay(gRtScene, RAY_FLAG_FORCE_OPAQUE, 0xFF, 0, hitProgramCount, 0, ray, payload);

	gOutput[launchIndex.xy] = float4(payload.color, 1.0);
}
