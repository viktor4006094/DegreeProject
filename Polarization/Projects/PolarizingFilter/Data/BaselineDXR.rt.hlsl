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

__import Shading;
__import Lights;


#define M_PI     3.14159265358979323846

#define MAX_RAY_DEPTH (3)
//#define MISS_COLOR (float4(0.53f, 0.81f, 0.92f, 1))
//#define MISS_COLOR (float4(0.1,0.1,0.1,1.0))
#define MISS_COLOR (float4(0.2,0.2,0.2,1.0))

shared
cbuffer PerFrameCB
{
    float4x4 invView;
    float4x4 invModel;
    float2 viewportDims;
    float tanHalfFovY;
};


//TODO look into what shared actually means in this case
shared
cbuffer SettingsCB
{
    int maxRecursionDepth;
    float tMin;
    float tMax;
    bool uniformLighting;
};

struct PrimaryRayPayload
{
    float4 color;
    uint depth;
    float hitT;
};

struct ShadowRayData
{
    bool hit;
};




// New shadow ray payload
struct ShadowRayPayload
{
    float visibility; // 0.0 means shadowed, 1.0 means hit
};

struct IndirectPayload
{
    float3 color;
    uint depth;
};


//**************************************************************************
// Helper functions
//**************************************************************************


/** Lambertian diffuse
*/
float3 lambertianDiffuse(float3 color)
{
    return color * (1 / M_PI);
}


//    return sr;
//};




float shootShadowRay(RayDesc ray)
{
    ShadowRayPayload shadowPayload = { 0.0f }; //TODO: check which initial val it should have

    // TODO: look over which flags to use
    uint flags = RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH | RAY_FLAG_SKIP_CLOSEST_HIT_SHADER;

    TraceRay(gRtScene, flags, 0xFF, 1, hitProgramCount, 1, ray, shadowPayload);
    return shadowPayload.visibility;
}


//float3 shootReflectionRay(RayDesc ray)
//{
//    //IndirectPayload reflectionPayload = { float3(0.0, 0.0, 0.0), 0 };
//    PrimaryRayPayload reflectionPayload;
//    reflectionPayload.color = float3(0.0, 0.0, 0.0);
//    reflectionPayload.depth = 1;
    

//    while (reflectionPayload.depth <= 2)
//    {
//        // Currently uses same shaders as the primary ray
//        TraceRay(gRtScene, RAY_FLAG_NONE, 0xFF, 0, hitProgramCount, 0, ray, reclectionPayload);
 
        
    
//    }


//    return reflectionPayload.color;
//}




// Old functions


bool checkLightHit(uint lightIndex, float3 origin)
{
    float3 direction = gLights[lightIndex].posW - origin;
    RayDesc ray;
    ray.Origin = origin;
    ray.Direction = normalize(direction);
    ray.TMin = tMin;
    ray.TMax = max(tMin+0.01, length(direction));

    ShadowRayData rayData;
    rayData.hit = true;
    TraceRay(gRtScene, RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH, 0xFF, 1 /* ray index */, hitProgramCount, 1, ray, rayData);
    return rayData.hit;
}

float3 getReflectionColor(float3 worldOrigin, VertexOut v, float3 worldRayDir, uint hitDepth)
{
    float3 finalColor = float3(0, 0, 0);
    float3 reflectColor = float3(0, 0, 0);
    //if (hitDepth < MAX_RAY_DEPTH)
    if (hitDepth < maxRecursionDepth)
    {
        PrimaryRayPayload secondaryRay;
        secondaryRay.depth.r = hitDepth + 1;
        RayDesc ray;
        ray.Origin = worldOrigin;
        ray.Direction = reflect(worldRayDir, v.normalW);
        ray.TMin = tMin;
        ray.TMax = tMax;
        TraceRay(gRtScene, 0 /*rayFlags*/, 0xFF, 0 /* ray index*/, hitProgramCount, 0, ray, secondaryRay);
        reflectColor = secondaryRay.hitT == -1 ? MISS_COLOR.rgb : secondaryRay.color.rgb;
        //reflectColor = secondaryRay.hitT == -1 ? 0 : secondaryRay.color.rgb;
        float falloff = max(1, (secondaryRay.hitT * secondaryRay.hitT));
        reflectColor *= 20 / falloff;

        finalColor += reflectColor;
    }
    else
    {
        finalColor = float3(0.5, 0.5, 0.5);
    }
    return finalColor;
}


//**************************************************************************
// Shadow rays
//**************************************************************************

// TODO: make one of these empty since the same default value will always be used
[shader("miss")]
void shadowMiss(inout ShadowRayPayload payload)
{
    payload.visibility = 1.0f;
}

// empty since shadow payload is considered not-visible by default
[shader("anyhit")]
void shadowAnyHit(inout ShadowRayPayload payload, in BuiltInTriangleIntersectionAttributes attribs)
{
}


//**************************************************************************
// Primary rays
//**************************************************************************

[shader("miss")]
void primaryMiss(inout PrimaryRayPayload hitData)
{
    hitData.color = MISS_COLOR;
    hitData.hitT = -1;
}


[shader("closesthit")]
void primaryClosestHit(inout PrimaryRayPayload hitData, in BuiltInTriangleIntersectionAttributes attribs)
{
    // Get the hit-point data
    float3 rayOrigW = WorldRayOrigin();
    float3 rayDirW = WorldRayDirection();
    float hitT = RayTCurrent();
    uint triangleIndex = PrimitiveIndex();

    float3 posW = rayOrigW + hitT * rayDirW;
    // prepare the shading data
    VertexOut v = getVertexAttributes(triangleIndex, attribs);
    ShadingData sd = prepareShadingData(v, gMaterial, rayOrigW, 0);

    // Shoot a reflection ray
    float3 reflectColor = getReflectionColor(posW, v, rayDirW, hitData.depth.r);
    float3 color = 0;






    if (uniformLighting)
    {
        color += (sd.diffuse.rgb * (1 / M_PI));
    }
    else
    {
        [unroll]
        for (int i = 0; i < gLightsCount; i++)
        {
        //if (checkLightHit(i, posW) == false)

            float3 direction = gLights[i].posW - posW;
            RayDesc shadowRay;
            shadowRay.Origin = posW;
            shadowRay.Direction = normalize(direction);
            shadowRay.TMin = 0.001;
            shadowRay.TMax = max(0.01, length(direction));

        // TODO test without if-statement
        //if (shootShadowRay(shadowRay) != 0.0)
        //{
        //    color += evalMaterial(sd, gLights[i], 1).color.xyz;
        //}
        //color += (shootShadowRay(shadowRay) * evalMaterial(sd, gLights[i], 1).color.xyz);

            //color += evalMaterial(sd, gLights[i], 1).color.xyz; // diffuse


            //WIP what will be used eventually

            //TODO rewrite with cutom functions
            LightSample ls1 = evalLight(gLights[i], sd);
            
            //diffuse
            // Something's weird here
            color += (ls1.diffuse * saturate(sd.diffuse.rgb * (1 / M_PI)) * ls1.NdotL).rgb;

        }
    }
        // Uniform color
    //color += evalMaterialUniform(sd).color.xyz;

    //hitData.color.rgb = color;
    //hitData.hitT = hitT;
    //// A very non-PBR inaccurate way to do reflections
    float roughness = min(0.5, max(1e-8, sd.roughness));
    ////hitData.color.rgb *= (1.0 - roughness * roughness); //EXPERIMENTAL
    //hitData.color.rgb += sd.specular * reflectColor * (roughness * roughness);
    //hitData.color.rgb += sd.emissive;

    //hitData.color.rgb += lambertianDiffuse(color);

    //float3 cdiff = evalMaterial(sd, gLights[i], 1).color.xyz;


    hitData.color.rgb += (color + sd.specular * reflectColor * roughness);
    hitData.color.rgb += sd.emissive;

    hitData.hitT = hitT;
    hitData.color.a = 1;
}



//**************************************************************************
// Ray generation
//**************************************************************************

[shader("raygeneration")]
void rayGen()
{
    uint3 launchIndex = DispatchRaysIndex();
    float2 d = (((launchIndex.xy + 0.5) / viewportDims) * 2.f - 1.f);
    float aspectRatio = viewportDims.x / viewportDims.y;

    RayDesc ray;
    ray.Origin = invView[3].xyz;

    // We negate the Z exis because the 'view' matrix is generated using a 
    // Right Handed coordinate system with Z pointing towards the viewer
    // The negation of Z axis is needed to get the rays go out in the direction away fromt he viewer.
    // The negation of Y axis is needed because the texel coordinate system, used in the UAV we write into using launchIndex
    // has the Y axis flipped with respect to the camera Y axis (0 is at the top and 1 at the bottom)
    ray.Direction = normalize((d.x * invView[0].xyz * tanHalfFovY * aspectRatio) - (d.y * invView[1].xyz * tanHalfFovY) - invView[2].xyz);
    
    ray.TMin = 0; // no self 
    ray.TMax = tMax;

    PrimaryRayPayload payload;
    payload.depth = 0;
    payload.color = 0;
    TraceRay(gRtScene, 0 /*rayFlags*/, 0xFF, 0 /* ray index*/, hitProgramCount, 0, ray, payload);
    gOutput[launchIndex.xy] = payload.color;
}
