#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(binding = 0) uniform UniformBufferObject {
    mat4 model;
    mat4 view;
    mat4 proj;
} ubo;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec3 inNormal;

layout(location = 0) out vec3 fragColor;

vec3 light_direction = vec3(0.0, 0.0, -1.0);

void main() {
    gl_Position = ubo.proj * ubo.view * ubo.model * vec4(inPosition, 10.0);  
    float cos_theta = dot(inNormal, light_direction);
    if(cos_theta < 0.0) cos_theta *= -1.0;
    if(cos_theta < 0.05) cos_theta = 0.05; //without black triangles
    fragColor = inColor * cos_theta;

}
