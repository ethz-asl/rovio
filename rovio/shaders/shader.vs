#version 300 es

precision highp float;

layout (location = 0) in vec3 Position;
layout (location = 1) in vec3 Normal;
layout (location = 2) in vec4 Color;

uniform mat4 V_TF_B;
uniform mat4 W_TF_B;

out vec4 Color0;
out vec3 Normal0;
out vec2 TexCoord0;

void main()
{
    gl_Position = V_TF_B * vec4(Position, 1.0f);
    Color0 = Color;
    Normal0 = (W_TF_B * vec4(Normal, 0.0)).xyz;
    TexCoord0 = Color.xy;
}
