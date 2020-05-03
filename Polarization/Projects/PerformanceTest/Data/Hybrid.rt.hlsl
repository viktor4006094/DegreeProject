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

#define TMIN 0.001
#define TMAX 10000.0

#define M_PI     3.14159265358979323846
#define M_PI2    6.28318530717958647692
#define M_INV_PI 0.3183098861837906715

#define COLOR_WHITE  float3(1.0, 1.0, 1.0)
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
// Data structures and initializer functions
//**************************************************************************************************

struct MuellerData
{
	// red, green, and blue Mueller matrices
	float4x4 mmR;
	float4x4 mmG;
	float4x4 mmB;
};

struct StokesLight
{
	// red, green, and blue Stokes vectors
	float4 svR;
	float4 svG;
	float4 svB;

	// local coordinate system's x-axis unit vector
	float3 referenceX;
};

StokesLight initStokes()
{
	StokesLight sl;
	sl.svR = float4(0.0, 0.0, 0.0, 0.0);
	sl.svG = float4(0.0, 0.0, 0.0, 0.0);
	sl.svB = float4(0.0, 0.0, 0.0, 0.0);
	sl.referenceX = float3(1.0, 0.0, 0.0);

	return sl;
}

StokesLight unPolarizedLight(float3 color)
{
	StokesLight sl = initStokes();
	sl.svR.x = color.r;
	sl.svG.x = color.g;
	sl.svB.x = color.b;

	return sl;
}

float3 stokesToColor(StokesLight sl) {
	return saturate(float3(sl.svR.x, sl.svG.x, sl.svB.x));
}

//**************************************************************************************************
// Operator macros for StokesLight and MuellerData
//**************************************************************************************************

// operator += for StokesLight and unpolarized float3 color
// unpolarized light can be added directly, no need to first create a Stokes vector for it
#define SL_ADD_EQ_UNPOL(sl, c) sl.svR.x += c.r; \
                               sl.svG.x += c.g; \
                               sl.svB.x += c.b;

// operator += for two already aligned StokesLight parameters
#define SL_ADD_EQ_POL(sl_a, sl_b) sl_a.svR += sl_b.svR; \
                                  sl_a.svG += sl_b.svG; \
                                  sl_a.svB += sl_b.svB;

// operator *= for StokesLight and MuellerData
#define SL_MUL_EQ_MD(sl, md) sl.svR = mul(sl.svR, md.mmR); \
                             sl.svG = mul(sl.svG, md.mmG); \
                             sl.svB = mul(sl.svB, md.mmB);


// operator *= for MuellerData and a scalar
#define MD_MUL_EQ_SCALAR(md, s) md.mmR *= s; md.mmG *= s; md.mmB *= s;

//**************************************************************************************************
// Ray payloads
//**************************************************************************************************

struct Payload
{
	StokesLight lightData;
	uint        recursionDepth; // How many reflections deep the payload is
};

Payload initPayload(uint depth)
{
	Payload pay;
	pay.lightData = initStokes();
	pay.recursionDepth = depth;
	
	return pay;
}


struct SimplePayload
{
	float3 color;
	uint   recursionDepth; // How many reflections deep the payload is
};

SimplePayload initSimplePayload(uint depth)
{
	SimplePayload pay;
	pay.color = COLOR_BLACK;
	pay.recursionDepth = depth;

	return pay;
}

//**************************************************************************************************
// StokesLight functions
//**************************************************************************************************

// Get the normalized reference x vector from normalized y and z vectors
float3 computeX(float3 y, float3 z)
{
	return normalize(cross(y, z));
}

/** Rotate the reference frame of a Stokes vector
  c2p: cos(2phi)
  s2p: sin(2phi)
*/
void rotateStokes(inout float4 S, float c2p, float s2p)
{
	float old_y = S.y;
	float old_z = S.z;

	S.y =  c2p*old_y + s2p*old_z;
	S.z = -s2p*old_y + c2p*old_z;
}

/** Rotate reference frame
  The light's direction vector dir is needed to rotate the reference X around
*/
void rotateReferenceFrame(inout StokesLight light, float3 newX, float3 dir)
{
	//TODO find some decent reference to how this math
	//INPUTS FLIPPED??
	float dotX = dot(light.referenceX, newX);
	float detX = dot(dir, cross(light.referenceX, newX));
	float phi = atan2(detX, dotX);

	float c2p = cos(2*phi);
	float s2p = sin(2*phi);
	
	rotateStokes(light.svR, c2p, s2p);
	rotateStokes(light.svG, c2p, s2p);
	rotateStokes(light.svB, c2p, s2p);
	light.referenceX = newX;
}

/** a+=b operator for StokesLight
  Rotates b's reference frame before addition if needed.
*/
void slAddEquals(inout StokesLight a, StokesLight b, float3 dir)
{
	// Mmake sure b's reference frame matches a's before adding them
	rotateReferenceFrame(b, a.referenceX, dir);
	SL_ADD_EQ_POL(a, b);
}

/** Applies the polarizing filter to a Stokes vector s
*/
void polarizeStokes(inout float4 s)
{
	uniform float a = filterCos2A;
	uniform float b = filterSin2A;
	float3 oldXYZ = s.xyz;

	s.x = dot(oldXYZ, float3(1.0,   a,   b));
	s.y = dot(oldXYZ, float3(  a, a*a, a*b));
	s.z = dot(oldXYZ, float3(  b, a*b, b*b));
	s.w = 0.0;
}

/** Applies a horizontal polarizing filter that's been rotated clockwise by angle A
  Note: also doubles the intensity to compensate for the filter on average blocking half of the
        incoming light.
*/
void applyPolarizingFilter(inout StokesLight l)
{
	polarizeStokes(l.svR);
	polarizeStokes(l.svG);
	polarizeStokes(l.svB);
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

/** Mueller Matrix for Fresnel reflections
    n, k : Real and complex parts of the Index of Refraction
    theta: angle between h and v
*/
float4x4 F_MuellerMatrix(float n, float k, float sinTheta, float cosTheta, float tanTheta)
{
	float n2 = n*n;
	float k2 = k*k;
	float st2 = sinTheta*sinTheta;

	float left  = sqrt((n2 - k2 - st2)*(n2 - k2 - st2) + 4*n2*k2);
	float right = n2 - k2 - st2;

	float a2 = 0.5*(left + right);
	float b2 = 0.5*(left - right);

	float a = sqrt(a2);
	float b = sqrt(max(b2,0.0));
	float ct2 = cosTheta*cosTheta;

	// orthogonal
	float ortA = a2 + b2 + ct2;
	float ortB = 2.0*a*cosTheta;

	// parallel
	float parA = a2 + b2 + st2*tanTheta*tanTheta;
	float parB = 2.0*a*sinTheta*tanTheta;

	// Fresnel parameters
	float F_ort = (ortA - ortB)/(ortA + ortB);
	float F_par = ((parA - parB)/(parA + parB))*F_ort;
	float D_ort = atan((2*cosTheta)/(ct2 - a2 - b2));
	float D_par = atan((2*b*cosTheta*((n2 - k2)*b - 2*n*k*a))/((n2 + k2)*(n2 + k2)*ct2 - a2 - b2));

	float phaseDiff = D_ort - D_par;

	// Matrix components
	float A = 0.5*(F_ort + F_par);
	float B = 0.5*(F_ort - F_par);
	float C = cos(phaseDiff)*sqrt(F_ort*F_par);
	float S = sin(phaseDiff)*sqrt(F_ort*F_par);

	return float4x4(  A,   B, 0.0, 0.0,
	                  B,   A, 0.0, 0.0,
	                0.0, 0.0,   C,   S,
	                0.0, 0.0,  -S,   C);
}

/** Schlick's Approximation Fresnel Function (F)
  f90 has been hard-coded to 1.0
*/
float3 F_Schlick(float3 f0, float u, float metalness)
{
	//return f0 + (float3(1.0, 1.0, 1.0) - f0)*pow((1.0 - u), 5.0);

	float3 IoR_n = lerp(nonMetalIoRn, metalIoRn, metalness);
	float3 IoR_k = metalness*metalIoRk; // k is zero for non-metals


	float3 R0 = ((IoR_n - 1.0)*(IoR_n - 1.0) + IoR_k*IoR_k) / ((IoR_n + 1.0)*(IoR_n + 1.0) + IoR_k*IoR_k);
	return R0 + (float3(1.0, 1.0, 1.0) - R0)*pow((1.0 - u), 5.0);
}

/** Polarization sensitive Fresnel Function (F)
*/
MuellerData F_Polarizing(float metalness, float sinTheta, float cosTheta, float tanTheta)
{
	// Index of Refraction is not available in material textures so it is set from the constant buffer
	float3 IoR_n = lerp(nonMetalIoRn, metalIoRn, metalness);
	float3 IoR_k = metalness*metalIoRk; // k is zero for non-metals

	MuellerData mdF;
	mdF.mmR = F_MuellerMatrix(IoR_n.r, IoR_k.r, sinTheta, cosTheta, tanTheta);
	mdF.mmG = F_MuellerMatrix(IoR_n.g, IoR_k.g, sinTheta, cosTheta, tanTheta);
	mdF.mmB = F_MuellerMatrix(IoR_n.b, IoR_k.b, sinTheta, cosTheta, tanTheta);

	return mdF;
}

/** Cook-Torrance Specular Term
*/
float3 CookTorrance_Simple(ShadingData sd, LightSample ls)
{
	float  D = D_GGX(sd.roughness, ls.NdotH);
	float  V = V_SmithGGX(ls.NdotL, sd.NdotV, sd.roughness);
	float3 F = F_Schlick(sd.specular, max(0, ls.LdotH), sd.metalness);

	return (V * D * ls.NdotL) * F;
}

/** Cook-Torrance Specular Term
*/
MuellerData CookTorrance_Pol(ShadingData sd, LightSample ls)
{
	float  D = D_GGX(sd.roughness, ls.NdotH);
	float  V = V_SmithGGX(ls.NdotL, sd.NdotV, sd.roughness);

	float3 H = normalize(sd.V + ls.L);
	
	float sinTheta = length(cross(ls.L, H));
	float cosTheta = ls.LdotH; // used since (LdotH == VdotH)
	float tanTheta = sinTheta/cosTheta;

	MuellerData F = F_Polarizing(sd.metalness, sinTheta, cosTheta, tanTheta);
	MD_MUL_EQ_SCALAR(F, (D*V*ls.NdotL));
	
	return F;
}

/** Calculates reflection color without polarization parameters with the Cook-Torrance specular term
*/
float3 getSimpleReflectionColor(ShadingData sd, float3 originW, float3 v, uint hitDepth)
{
	// N is used instead of H since (N == H) for perfect reflections. 
	float NdotV = saturate(dot(sd.N, v));
	
	// Early exit if we're out of rays or if the surface is not facing the ray
	if (hitDepth >= MAX_RECURSION_DEPTH || NdotV <= 0.0) {
		return REFL_MISS_COLOR;
	}

	SimplePayload rPayload = initSimplePayload((hitDepth + 1));

	RayDesc reflectionRay;
	reflectionRay.Origin = originW;
	reflectionRay.Direction = reflect(-v, sd.N);
	reflectionRay.TMin = TMIN;
	reflectionRay.TMax = TMAX;

	float NdotL = saturate(dot(sd.N, reflectionRay.Direction));
	float LdotH = NdotL;
	
	float  D = D_GGX(sd.roughness, 1.0); // NdotH=1.0 since (N == H)
	float  V = V_SmithGGX(NdotL, NdotV, sd.roughness);
	float3 F = F_Schlick(sd.specular, max(0.0, LdotH), sd.metalness);

	//saturate to prevent blown out colors
	float3 reflectionBrdf = saturate(D*V*NdotL)*F;
	
	// Send a simple reflection ray into the scene
	TraceRay(gRtScene, RAY_FLAG_FORCE_OPAQUE, 0xFF, 1, hitProgramCount, 1, reflectionRay, rPayload);

	// Multiply with the reflection BRDF
	rPayload.color *= reflectionBrdf;

	return rPayload.color;
}

/** Calculates reflection color with polarization parameters with the Cook-Torrance specular term
*/
StokesLight getReflectionData(ShadingData sd, float3 originW, float3 v, uint hitDepth)
{
	// N is used instead of H since (N == H) for perfect reflections. 
	float NdotV = saturate(dot(sd.N, v));

	// Early exit if we're out of rays or if the surface is not facing the ray
	if (hitDepth >= MAX_RECURSION_DEPTH || NdotV <= 0.0 ) {
		return unPolarizedLight(REFL_MISS_COLOR);
	}

	SimplePayload rPayload = initSimplePayload((hitDepth + 1));
	
	RayDesc reflectionRay;
	reflectionRay.Origin = originW;
	reflectionRay.Direction = reflect(-v, sd.N);
	reflectionRay.TMin = TMIN;
	reflectionRay.TMax = TMAX;

	float NdotL = saturate(dot(sd.N, reflectionRay.Direction));
	float sinTheta = length(cross(reflectionRay.Direction, sd.N));
	float cosTheta = NdotL;
	float tanTheta = sinTheta/cosTheta;

	float D = D_GGX(sd.roughness, 1.0); // NdotH=1.0 since (N == H)
	float V = V_SmithGGX(NdotL, NdotV, sd.roughness);

	MuellerData reflectionBrdf = F_Polarizing(sd.metalness, sinTheta, cosTheta, tanTheta);
	//saturate to prevent blown out colors
	MD_MUL_EQ_SCALAR(reflectionBrdf, saturate(D*V*NdotL));

	// Send a reflection ray into the scene
	TraceRay(gRtScene, RAY_FLAG_FORCE_OPAQUE, 0xFF, 1, hitProgramCount, 1, reflectionRay, rPayload);
	StokesLight reflected = unPolarizedLight(rPayload.color);

	// Align the incoming light's reference frame
	float3 incomingRefX = computeX(sd.N, reflectionRay.Direction);
	rotateReferenceFrame(reflected, incomingRefX, reflectionRay.Direction);

	// Multiply with the reflection BRDF Mueller matrices
	SL_MUL_EQ_MD(reflected, reflectionBrdf);
	
	return reflected;
}

//**************************************************************************************************
// Hit shaders
//**************************************************************************************************

/** Primary miss shader for rays without polarization parameters
*/
[shader("miss")]
void simpleMiss(inout SimplePayload hitData)
{
	hitData.color = missColor;
}

/** Primary closest hit shader for rays without polarization parameters
*/
[shader("closesthit")]
void simpleClosestHit(inout SimplePayload hitData, in BuiltInTriangleIntersectionAttributes attribs)
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
			float3 specularComp = ls.specular*CookTorrance_Simple(sd, ls);

			hitData.color += specularComp;
			hitData.color += diffuseComp;
		}
	}

	hitData.color += sd.emissive;

	// Shoot a reflection ray
	float3 reflectColor = getSimpleReflectionColor(sd, posW, -rayDirW, hitData.recursionDepth);
	hitData.color += reflectColor;
}

/** Primary miss shader for rays with polarization parameters
*/
[shader("miss")]
void primaryMiss(inout Payload hitData)
{
	hitData.lightData = unPolarizedLight(missColor);
}

/** Primary closest hit shader for rays with polarization parameters
*/
[shader("closesthit")]
void primaryClosestHit(inout Payload hitData, in BuiltInTriangleIntersectionAttributes attribs)
{
	// Get the hit-point data
	float3 rayOrigW      = WorldRayOrigin();
	float3 rayDirW       = WorldRayDirection();
	float  hitT          = RayTCurrent();
	uint   triangleIndex = PrimitiveIndex();

	float3 posW = rayOrigW + hitT*rayDirW;
	// prepare the shading data
	VertexOut   va = getVertexAttributes(triangleIndex, attribs);
	ShadingData sd = prepareShadingData(va, gMaterial, rayOrigW, 0);

	// The reference x which is orthogonal to the light's direction and the surface's normal
	hitData.lightData.referenceX = computeX(sd.N, -rayDirW);

	[unroll]
	for (int i = 0; i < gLightsCount; i++) {
		LightSample ls = evalLight(gLights[i], sd);

		// Only shade surfaces which are hit by the light
		if (ls.NdotL > 0.0 && sd.NdotV > 0.0) {
			/* Diffuse component */
			// Diffuse is unpolarized so calculations with a float3 is sufficient
			float3 diffuseComp = ls.diffuse*Fd_Lambert(sd.diffuse.rgb, ls.NdotL);

			/* Specular component */
			MuellerData specularMueller = CookTorrance_Pol(sd, ls);

			// All light sources are unpolarized so no reference frame rotation needed before multiplication
			StokesLight specularStokes = unPolarizedLight(ls.specular);
			SL_MUL_EQ_MD(specularStokes, specularMueller);

			// The output reference frame's Y vector lies in the specular reflection's plane of
			// incidence so the microfacet normal H is used to calculate the X vector
			float3 H = normalize(sd.V + ls.L);
			specularStokes.referenceX = computeX(H, sd.V);

			// slAddEquals will rotate reference frame if needed
			slAddEquals(hitData.lightData, specularStokes, -rayDirW);
			SL_ADD_EQ_UNPOL(hitData.lightData, diffuseComp);
		}
	}

	SL_ADD_EQ_UNPOL(hitData.lightData, sd.emissive);

	// Shoot a reflection ray

	StokesLight reflectedLight = getReflectionData(sd, posW, -rayDirW, hitData.recursionDepth);
	slAddEquals(hitData.lightData, reflectedLight, -rayDirW);
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

	/* Rotate the ray's reference frame to align with the camera's one */
	// camera's x points up, y points left //TODO? check that this is correct
	float3 cameraUp = normalize(invView[1].xyz);
	float3 cameraX  = computeX(cameraUp, -ray.Direction);

	rotateReferenceFrame(payload.lightData, cameraX, -ray.Direction);

	/* Apply polarizing filter */
	if (filterEnabled) {
		applyPolarizingFilter(payload.lightData);
	}

	/* Get the final output color from the Stokes data */
	float3 outputColor = stokesToColor(payload.lightData);

	gOutput[launchIndex.xy] = float4(outputColor, 1.0);
}
