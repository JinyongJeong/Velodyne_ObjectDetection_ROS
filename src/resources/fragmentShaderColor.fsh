#version 450

//! [0]
in vec4 varyingColor;

out vec4 fragColor;

void main(void)
{
    fragColor = varyingColor;
}
//! [0]
