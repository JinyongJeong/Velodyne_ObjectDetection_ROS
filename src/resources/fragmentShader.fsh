#version 450

//! [0]
uniform vec4 color;

out vec4 fragColor;

void main(void)
{
    fragColor = color;
}
//! [0]
