#version 300 es

#ifdef GL_ES
precision highp float;
#endif

/*
 The shader has two behaviours:
    1. distorts the image on a side like in real cars
        a. with linear + parabolic curvature
        b. with circular curvature
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
// - reverses the thredhold
uniform bool u_reversed;
// - if >0, then the distortios is circular and this parameter is the radius, otherwise it is linear-parabolic
uniform float u_convex_radius;

const vec3 BLANK_COLOR = vec3(0.0, 0.0, 0.2);

// Constants used for colorizing the inspection matrix
const float PI = 3.14159265359;
const float COLOR_GAIN = 1.0 / sin( PI / 3.0 ) / 2.0;
const float PHASE_120 = 2.0 * PI / 3.0;
const vec3 MATRIX_COLOR = vec3(1, 0, 1);

// Constants for image distortion
const float THRESHOLD = 0.30;
const float ZOOM = 1.4;
const float CONVEX_RADIUS = 3.;

// Pipeline attributes
in vec2 v_uv;      // modernGL only
out vec4 out_color;

float get_parabolic_left(float x, float threshold, float zoom) {
    float denom = 1. - threshold;

    // Linear end
    //   Line crosses (1,1) and (T,zoom*T), where T is the threshold:
    //      (x - 1)/(T - 1) = (y - 1)/(zoom*T - 1)
    //      y  = x*(zoom*T - 1)/(T - 1) - T*(1 - zoom)/(T - 1)
    float a = (1. - zoom * threshold) / denom;
    float b = threshold * (zoom - 1.) / denom;

    if (x <= threshold) {
        // Parabolic start. Solving:
        //   a_*T^2 + b_*T + c_ = 0           - parabola crosses the point (0,0)
        //   a_*T^2 + b_*T + c_ = a*T + b     - line and parabola intersect at T (threshold)
        //   2*a_*T + b_ = a                  - line's and parabola's tangentums are same at the intersection point T
        float a_ =  -b / threshold / threshold;
        float b_ =  a + 2. * b / threshold;
        float c_ =  0.;
        return a_ * x * x + b_ * x + c_;
    }
    else {
        return a * x + b;
    }
}

float get_parabolic_right(float x, float threshold, float zoom) {
    float denom = 1. - threshold;

    // Linear start
    //   The line crosses the points (0,0) and (T,T-dy), where T is the threshold, and dy is calcualted from 'zoom' as shown below.
    //   1. Since 'zoom' arg is given for the left-side case (>1), we need to find first what is the difference between
    //      the undistorted point
    //          y(T') = x
    //      and the distorted one 
    //          y'(T') = zoom*x
    //      for the left-side case:
    //          dy(T') = y'(T') - y(T') = zoom*T' - T' = T'*(zoom - 1)
    //      Note that since the 'threshold' arg (T) was given for the right-side case, it was converted the left-side case threshold T':
    //          T' = 1 - T
    //   2. So, as the line crosses (0,0) and (T,T-dy),
    //      y'(T) = T - T'*(zoom - 1)
    //   and as the line also crossed (0,0), then the linear equation is 
    //      (x - 0)/(T - 0) = (y - 0)/(T - T'*(zoom - 1) - 0)
    //      x/T = y/(T - T'*(zoom - 1))
    //      y = x*(T - T'*(zoom - 1))/T = x*(1 - T'/T * (zoom - 1))
    float a = 1. - (1. - threshold) / threshold * (zoom - 1.);
    float b = 0.;

    if (x <= threshold) {
        return a * x + b;
    }
    else {
        // Parabolic end. Solving:
        //   a_*T^2 + b_*T + c_ = a*T + b     - line and parabola intersect at T (threshold)
        //   2*a_*T + b_ = a                  - line's and parabola's tangentums are same at the intersection point T
        //   a_ + b_ + c_ = 1                 - parabola crosses the point (1,1)
        float a_ =  (1. - a - b) / denom / denom;
        float b_ =  a - 2. * threshold * a_;
        float c_ = 1. - a_ - b_;
        return a_ * x * x + b_ * x + c_;
    }
}

float get_circluar(float x, float r) {
    // Circle center
    float a = (1. - sqrt(2. * r * r - 1.)) / 2.;
    float b = 1. - a;   //(1. + sqrt(2. * r * r - 1.)) / 2.;

    float dx = u_reversed ? x - a : x - b;
    float k = sqrt(r * r - dx * dx);
    return u_reversed ? -k + b : k + a;
}

void main() {
    vec2 uv;

    // Choose the source of UV coordinates
    uv = v_uv == vec2(0) ? gl_FragCoord.xy / u_resolution : v_uv;

    // Choose the source of the parameters of the image distortion algorithm
    float zoom = u_zoom == 0. ? ZOOM : u_zoom;
    float radius = u_convex_radius < 1. ? CONVEX_RADIUS : u_convex_radius;

    float threshold = u_mouse.x == 0. ?
        (u_reversed ? 1. - THRESHOLD : THRESHOLD) :
        1. - u_mouse.x / u_resolution.x;

    // Apply distortion
    if (threshold < 0. || threshold > 1. || zoom <= 0.) {
        // do nothing
    }
    else {
        uv.x = u_convex_radius != 0. ?
            get_circluar(uv.x, radius) :
            (u_reversed ?
                get_parabolic_right(uv.x, threshold, zoom) :
                get_parabolic_left(uv.x, threshold, zoom)
            );
    }

    // Calculate the output color
    vec3 color;

    if (uv.x > 1. || uv.x < 0. || uv.y > 1. || uv.y < 0.) {
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

    out_color = vec4(color, 1.);
}
