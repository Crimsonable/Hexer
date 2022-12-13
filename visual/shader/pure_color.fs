#version 330 core
out vec4 FragColor;

in vec2 TexCoords;

uniform vec4 color_in;

void main()
{    
    FragColor = color_in;
}