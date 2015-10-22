#version 300 es

precision highp float;

in vec4 Color0;
in vec3 Normal0;
in vec2 TexCoord0;

out vec4 FragColor;

struct DirectionalLight
{
    vec3 Color;
    float AmbientIntensity;
    float DiffuseIntensity;
    vec3 Direction;
};
                                        
uniform DirectionalLight gDirectionalLight;
uniform sampler2D gSampler;
uniform bool useTexture;

void main()
{
    vec4 AmbientColor = vec4(gDirectionalLight.Color, 1.0f) * gDirectionalLight.AmbientIntensity;

    float DiffuseFactor = dot(normalize(Normal0), -gDirectionalLight.Direction);

    vec4 DiffuseColor;

    if (DiffuseFactor > 0.0f) {
        DiffuseColor = vec4(gDirectionalLight.Color, 1.0f) * gDirectionalLight.DiffuseIntensity * DiffuseFactor;
    }
    else {
        DiffuseColor = vec4(0.0f, 0.0f, 0.0f, 0.0f);
    }

    if(!useTexture){
        FragColor = Color0 * (AmbientColor + DiffuseColor);
    }
    else {
        FragColor = texture2D(gSampler, TexCoord0.st) * (AmbientColor + DiffuseColor);
    }
}
