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
RWTexture2D<float4> gOutput;
__import Raytracing;
__import Lights;


/*
* TODO
*	Look over which flags to use
*/

//TODO performance test on Turing card
#define RAY_FLAGS (RAY_FLAG_FORCE_OPAQUE)
//#define RAY_FLAGS (RAY_FLAG_NONE)

#define M_PI     (3.14159265358979323846)
#define M_PI2    (6.28318530717958647692)
#define M_INV_PI (0.3183098861837906715)

//#define MISS_COLOR (float3(0.7,0.7,0.7))
//#define MISS_COLOR (float3(0.7,0,0))
#define MISS_COLOR (float3(0.0,0.0,0.0))

// Output types
#define SHOW_NORMALS      (0)
#define SHOW_SPECULAR     (1)
#define SHOW_DIFFUSE      (2)
#define SHOW_REFLECTIVITY (3)
#define SHOW_REFLECTIONS  (4)
#define SHOW_RESULT       (5)


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

//TODO rewrite as defines for the performance test
shared
cbuffer SettingsCB
{
	int   maxRecursionDepth;
	float tMin;
	float tMax;
	bool  uniformLighting; // i.e. no specular but 100% diffuse and reflections
	int   outputType;
	float reflectionCutoff;
};

//**************************************************************************************************
// Ray payloads
//**************************************************************************************************

struct Payload
{
	float3 color;
	float  hitT;
	uint   depth;
};



//**************************************************************************************************
// Shading functions
//**************************************************************************************************
//TODO rename roughness arguments to a or alpha

/** Lambertian diffuse
*/
float3 Fd_Lambert(float3 color, float NdotL)
{
	if (uniformLighting) {
		return color * M_INV_PI;
	} else {
		return color * (M_INV_PI * NdotL);
	}
}

/** GGX Normal Distribution Function (D)
*/
float D_GGX(float roughness, float NdotH)
{
	float a2 = roughness * roughness;
	float d = ((NdotH * a2 - NdotH) * NdotH + 1.0);
	return a2 / (M_PI * d * d);
}

/** Smith-GGX Visibility Function (V)
    V = G/(4*NdotL*NdotV)
*/
float V_SmithGGX(float NdotL, float NdotV, float roughness)
{
	float a2 = roughness * roughness;
	float ggxv = NdotL * sqrt((-NdotV * a2 + NdotV) * NdotV + a2);
	float ggxl = NdotV * sqrt((-NdotL * a2 + NdotL) * NdotL + a2);
	return 0.5 / (ggxv + ggxl);
}

/** Schlick's Approximation Fresnel Function (F)
    f90 has been hard-coded to 1.0
*/
float3 F_Schlick(float3 f0, float u)
{
	return f0 + (1.0 - f0) * pow(1.0 - u, 5.0);
}

//sd.specular, sd.roughness, sd.NdotV, ls.NdotH, ls.NdotL, ls.LdotH
/** Cook-Torrance Specular Term
*/
float3 CookTorrance(ShadingData sd, LightSample ls)
{
	float  D = D_GGX(sd.roughness, ls.NdotH);
	float  V = V_SmithGGX(ls.NdotL, sd.NdotV, sd.roughness);
	float3 F = F_Schlick(sd.specular, max(0, ls.LdotH));

	return (D * V * ls.NdotL) * F;
}


//**************************************************************************************************
// Helper functions
//**************************************************************************************************


/** 
*/
float3 getReflectionColor(ShadingData sd, float3 originW, float3 v, uint hitDepth)
{
	float3 reflectColor = float3(0.0, 0.0, 0.0);

	if (hitDepth < maxRecursionDepth) {
		RayDesc reflectionRay;
		reflectionRay.Origin = originW;
		reflectionRay.Direction = reflect(-v, sd.N);
		reflectionRay.TMin = tMin;
		reflectionRay.TMax = tMax;

		//TODO? replace H with N for reflections
		float3 H = normalize(v + reflectionRay.Direction);
		//float3 H = normalize(sd.N);
		float NdotV = saturate(dot(sd.N, v));
		float NdotL = saturate(dot(sd.N, reflectionRay.Direction));
		float LdotH = saturate(dot(reflectionRay.Direction, H));
		float VdotH = saturate(dot(v, H));

		if (NdotL > 0.0) {
			float  V = V_SmithGGX(NdotL, NdotV, sd.roughness);
			float3 F = F_Schlick(sd.specular, max(0, LdotH));

			//saturate to prevent blown out colors
			float3 specularBrdf = saturate((V * NdotL) * F);

			//TODO remove?
			// Only trace rays if at least reflectionCutoff of the incoming light will be reflected
			if (luminance(specularBrdf) > reflectionCutoff) {
				Payload rPayload;
				rPayload.depth.r = hitDepth + 1;

				TraceRay(gRtScene, RAY_FLAGS, 0xFF, 0, hitProgramCount, 0, reflectionRay, rPayload);

				reflectColor = (rPayload.hitT == -1.0) ? MISS_COLOR : rPayload.color;
			}

			reflectColor *= specularBrdf;// * NdotL;

			// only for debugging
			if (outputType == SHOW_REFLECTIVITY) {
				reflectColor = saturate((V * NdotL) * F);
			}
		}
	}
	return saturate(reflectColor);
}



//**************************************************************************************************
// Primary rays
//**************************************************************************************************

/**
*/
[shader("miss")]
void primaryMiss(inout Payload hitData)
{
	hitData.color = MISS_COLOR;
	hitData.hitT = -1.0;
}


/**
*/
[shader("closesthit")]
void primaryClosestHit(inout Payload hitData, in BuiltInTriangleIntersectionAttributes attribs)
{
	// Get the hit-point data
	float3 rayOrigW = WorldRayOrigin();
	float3 rayDirW = WorldRayDirection();
	float hitT = RayTCurrent();
	uint triangleIndex = PrimitiveIndex();

	float3 posW = rayOrigW + hitT * rayDirW;
	// prepare the shading data
	VertexOut va = getVertexAttributes(triangleIndex, attribs);
	ShadingData sd = prepareShadingData(va, gMaterial, rayOrigW, 0);


	// Shoot a reflection ray
	float3 reflectColor = getReflectionColor(sd, posW, -rayDirW, hitData.depth.r);
	float3 color = 0;

	//TODO remove for performance test
	float3 dbgSpecular = 0;
	float3 dbgDiffuse = 0;
	

	if (uniformLighting) {
		// uniform has diffuse but no specular since there are no light sources
		color += Fd_Lambert(sd.diffuse.rgb, 0);
		hitData.color = color;
		dbgDiffuse = color;
	} else {

		[unroll]
		for (int i = 0; i < gLightsCount; i++) {

			/* initShadingResult */
			ShadingResult sr;
			sr.diffuse = 0;
			sr.color.rgb = 0;
			sr.color.a = 1;
			sr.specular = 0;
			sr.diffuseBrdf = 0;
			sr.specularBrdf = 0;


			LightSample ls = evalLight(gLights[i], sd);


			// Shade surfaces which are hit by the light
			if (ls.NdotL > 0.0) {
				float VdotH = saturate(dot(sd.V, normalize(sd.V + ls.L)));

				// Calculate the specular term
				sr.specularBrdf = saturate(CookTorrance(sd, ls));
				sr.specular = ls.specular * sr.specularBrdf;// * ls.NdotL;
				dbgSpecular += sr.specularBrdf;//*ls.NdotL;
				sr.color.rgb += sr.specular;

				// Calculate the diffuse term
				sr.diffuse = ls.diffuse * Fd_Lambert(sd.diffuse.rgb, ls.NdotL);

				//TODO remove?
				// Compensating for fresnel
				//float3 F = F_Schlick(sd.specular, max(0, VdotH)); // Fresnel coeff
				//float comp = (F.r + F.g + F.b) / 3.0;
				//sr.diffuse += (sr.diffuse * (1.0 - comp));

				dbgDiffuse += sr.diffuse;
				sr.color.rgb += sr.diffuse;
				
				sr.color.a = sd.opacity;

				color += sr.color.rgb;
			}
		}
		hitData.color = color;
	}

	// both uniform and light source types have reflections
	hitData.color += reflectColor;
	hitData.color += sd.emissive;
	//hitData.color.a = 1;
	
	hitData.hitT = hitT;


	// Debugging outputs
	if (outputType == SHOW_NORMALS) {
		hitData.color = sd.N;
	} else if (outputType == SHOW_SPECULAR) {
		hitData.color = dbgSpecular;
	} else if (outputType == SHOW_DIFFUSE) {
		hitData.color = dbgDiffuse;
	} else if (outputType == SHOW_REFLECTIVITY) {
		hitData.color = reflectColor;
	} else if (outputType == SHOW_REFLECTIONS) {
		hitData.color = reflectColor;
	} else {

	}
}


//**************************************************************************************************
// Ray generation
//**************************************************************************************************

/** Shader entry point
*/
[shader("raygeneration")]
void rayGen()
{
	uint3 launchIndex = DispatchRaysIndex();
	float2 d = (((launchIndex.xy + 0.5) / viewportDims) * 2.0 - 1.0);
	float aspectRatio = viewportDims.x / viewportDims.y;

	RayDesc ray;
	ray.Origin = invView[3].xyz;

	// We negate the Z exis because the 'view' matrix is generated using a 
	// Right Handed coordinate system with Z pointing towards the viewer
	// The negation of Z axis is needed to get the rays go out in the direction away fromt he viewer.
	// The negation of Y axis is needed because the texel coordinate system, used in the UAV we write into using launchIndex
	// has the Y axis flipped with respect to the camera Y axis (0 is at the top and 1 at the bottom)
	ray.Direction = normalize((d.x * invView[0].xyz * tanHalfFovY * aspectRatio) - (d.y * invView[1].xyz * tanHalfFovY) - invView[2].xyz);

	ray.TMin = 0.0; // no self 
	ray.TMax = tMax;

	Payload payload;
	payload.depth = 0;
	payload.color = float3(0.0, 0.0, 0.0);

	TraceRay(gRtScene, RAY_FLAGS, 0xFF, 0, hitProgramCount, 0, ray, payload);

	gOutput[launchIndex.xy] = float4(payload.color, 1.0);
}
