#version 300 es

#ifdef GL_ES
precision mediump float;
#endif

/*
 The shader has two behaviours:
    1. distorts the image aka fish-eye lens
    2. colorizes the inspection matrix pixels
 This shader may run from ModernGL, but also from glslViewer.
 These environments instantiate the uniforms differently,
 this is why there are few if-else statements in this code
 exploring what data is available for the shader
*/

// Position 0 in the list of uniforms has the texture
uniform sampler2D u_texture;

// These uniforms are injected automatically by glslViewers,
// and also manually in opengl_renderer.py to support compatibility with glslViewer
uniform vec2 u_resolution;
uniform float u_time;
uniform vec2 u_mouse;

// These additional uniform set in opengl_renderer.py
// - supports zomming with the mouse scroll
uniform float u_zoom;
// - enables colorization of the inspection matrix pixels
uniform bool u_colorize;

const vec3 BLANK_COLOR = vec3(0.0, 0.0, 0.2);

// Constants used for colorizing the inspection matrix
const float PI = 3.14159265359;
const float COLOR_GAIN = 1.0 / sin( PI / 3.0 ) / 2.0;
const float PHASE_120 = 2.0 * PI / 3.0;
const vec3 MATRIX_COLOR = vec3(1, 0, 1);

// Constants for image distortion
const vec2 CENTER = vec2(0.5, 0.5);
const vec2 EXP    = vec2(2.0, 1.5);
const vec2 ZOOM   = vec2(0.5, 0.5);

// Pipeline attributes
in vec2 v_uv;      // modernGL only
out vec4 out_color;

void main() {
    vec2 uv;

    // Choose the source of UV coordinates
    uv = v_uv == vec2(0) ? gl_FragCoord.xy / u_resolution : v_uv;

    // Choose the source of the parameters of the image distortion algorithm
    vec2 zoom = u_zoom == 0.0 ? ZOOM : vec2(u_zoom);
    vec2 center = u_mouse.x == 0.0 ? CENTER : vec2(1.0 - u_mouse.x / u_resolution.x, u_mouse.y / u_resolution.y);

    // Apply distortion
    vec2 off_center = uv - center;

    if (off_center.x != 0.0) {
        float scale_x = off_center.x < 0.0 ? center.x : 1.0 - center.x;
        off_center.x *= (1.0 + zoom.x * pow(abs(off_center.x / scale_x), EXP.x))/(1.0 + zoom.x);
    }
    if (off_center.y != 0.0) {
        float scale_y = off_center.y < 0.0 ? center.y : 1.0 - center.y;
        off_center.y *= (1.0 + zoom.y * pow(abs(off_center.y / scale_y), EXP.y))/(1.0 + zoom.y);
    }

    uv = center + off_center;

    // Calculate the output color
    vec3 color;

    if (uv.x > 1.0 || uv.x < 0.0 || uv.y > 1.0 || uv.y < 0.0) {
        // sets the pixels outside the texture to the blank color
        color = BLANK_COLOR;
    } else {
        color = texture(u_texture, uv).rgb;

        // colorize the matrix pixels if the colorization mode is on
        if (u_colorize && color == MATRIX_COLOR) {
            color = vec3(
                0.5 + COLOR_GAIN * sin(u_time + PHASE_120), 
                0.5 + COLOR_GAIN * sin(u_time), 
                0.5 + COLOR_GAIN * sin(u_time - PHASE_120)
            );
        }
    }

    out_color = vec4(color, 1.0);
}
