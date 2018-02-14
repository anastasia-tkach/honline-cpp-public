#version 330
/// texture with depth from sensor
uniform usampler2D tex_depth;
uniform float zNear;
uniform float zFar;

uniform mat4 view_projection; ///< for visualization 
uniform mat3 inv_proj_matrix; ///< depth-to-3D conversion

uniform float undistort;

in vec2 vpoint;
in vec2 uv;

out float depth;
out vec2 frag_uv;
out float discardme;
out vec3 f_position;

vec3 undistort_function(vec3 x_d) {

	float k0 = -0.27686;
	float k1 = 0.12105;
	float k2 = -0.035386;
	float k3 = 0;
	float k4 = 0;
	float k5 = 0;
	float cx = -0.0091268;
	float cy = 0.0061785;
	float tx = 0.0021307;
	float ty = 0.00060417;
	float max_r = 0.78384;

	x_d -= vec3(cx, cy, 0);
	vec2 x_n = vec2(x_d[0], x_d[1]);
	float r2, r4, r6;
	for (int iter = 0; iter < 20; iter++) {

		/// Add distortion 
		r2 = x_n[0] * x_n[0] + x_n[1] * x_n[1];
		r4 = r2 * r2;
		r6 = r4 * r2;

		/// Radial distortion :
		float cdist = 1 + k0 * r2 + k1 * r4 + k2 * r6;
		float cdist2 = 1 + k3 * r2 + k4 * r4 + k5 * r6;

		if (cdist == 0) cdist = 1;
		float invCdist = cdist2 / cdist;

		/// Tangential distortion
		vec2 dx = vec2(0, 0);
		dx[0] = 2 * tx * x_n[0] * x_n[1] + ty * (r2 + 2 * x_n[0] * x_n[0]);
		dx[1] = tx * (r2 + 2 * x_n[1] * x_n[1]) + 2 * ty * x_n[0] * x_n[1];

		x_n = invCdist * (vec2(x_d[0], x_d[1]) - dx);
	}

	if (r2 > max_r * max_r) x_n = vec2(0, 0);
	x_n += vec2(cx, cy);
	x_d = vec3(x_n[0], x_n[1], 1);

	return x_d;
}

void main() {

	///--- Tex coords for color fetch
	frag_uv = uv;

	///--- Depth evaluation
	depth = float(texture(tex_depth, uv).x);
	discardme = float(depth<zNear || depth>zFar);

	//vec3 p = inv_proj_matrix * vec3(vpoint[0] * depth, vpoint[1] * depth, depth);

	vec3 p = inv_proj_matrix * vec3(vpoint[0], vpoint[1], 1);
	if (undistort == 1.0f) p = undistort_function(p);
	p = depth * p;

	gl_Position = view_projection * vec4(p[0], -p[1], p[2], 1.0);
	f_position = p;

	///--- Splat size
	gl_PointSize = 2.9; ///< Screen
	//gl_PointSize = 4; ///< RETINA
}

