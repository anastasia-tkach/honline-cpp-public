#version 330 core
//layout(location = 0) out vec3 framebuffer_color;
out vec4 color_with_alpha;
vec3 color;

uniform float window_left;
uniform float window_bottom;
uniform float window_height;
uniform float window_width;
uniform vec3 camera_center;
uniform mat4 MVP;
uniform mat4 invMVP;

uniform mat4 projection;
uniform mat4 model;
uniform mat4 view;

//HModel_old
//const int num_blocks = 29;
//const int num_centers = 36;

//HModel
//const int num_blocks = 28;
//const int num_centers = 34;

//HModel with wrist
const int num_blocks = 37; //30
const int num_centers = 46; // 38

//HTrack
//const int num_blocks = 17;
//const int num_centers = 24; 

uniform vec3 centers[num_centers];
uniform float radii[num_centers];
uniform ivec3 blocks[num_blocks];
uniform vec3 tangents_v1[num_blocks];
uniform vec3 tangents_v2[num_blocks];
uniform vec3 tangents_v3[num_blocks];
uniform vec3 tangents_u1[num_blocks];
uniform vec3 tangents_u2[num_blocks];
uniform vec3 tangents_u3[num_blocks];

// Online Calibration
uniform bool display_certainty;
uniform float center_index_to_length_certainty_map[num_centers];
uniform float center_index_to_x_certainty_map[num_centers];
uniform float center_index_to_y_certainty_map[num_centers];
uniform float center_index_to_radius_certainty_map[num_centers];


// Lighting
uniform vec3 Ia, Id, Is;
uniform vec3 ka, kd, ks;
uniform float p;
uniform vec3 light_pos;

// Texture
in vec2 uv;
uniform sampler2D synthetic_texture;
uniform sampler2D silhouette;
uniform sampler2D real_texture;

const int RAND_MAX = 32767;
const float epsilon = 0.00001;

uniform int wristband_found;
uniform int num_frames_since_calibrated;

vec2 project(vec3 point){
	vec4 point_gl =  MVP * vec4(point, 1.0);
    vec3 point_clip = vec3(point_gl[0], point_gl[1], point_gl[2]) / point_gl[3];
	float f = gl_DepthRange.far; 
	float n = gl_DepthRange.near;
	//int n = 0; int f = 1; 
	float ox = window_left + window_width/2;
	float oy = window_bottom + window_height/2;
	
    float xd = point_clip[0];
    float yd = point_clip[1];
    float zd = point_clip[2];
	vec3 point_window = vec3(0, 0, 0);
    point_window[0] = xd * window_width / 2 + ox;
    point_window[1] = yd * window_height / 2 + oy;
    point_window[2] = zd * (f - n) / 2 + (n + f) / 2;

	//int i1 = int(point_window[0]);
	//int i2 = int(point_window[1]);
    //point_final = ivec2(i1, i2);

	return vec2(point_window);
}

vec3 unproject(float winx, float winy, float winz){
    vec4 a = vec4(0, 0, 0, 0);
    a[0] = (winx - window_left) / window_width * 2.0 - 1.0;
    a[1] = (winy - window_bottom) / window_height * 2.0 - 1.0;
    a[2] = 2.0 * winz - 1.0;
    a[3] = 1.0;
    vec4 b  = invMVP * a;
    if (b[3] == 0.0)
		return vec3(0, 0, 0);
    b[3] = 1.0 / b[3];
    vec3 world_point = vec3(0, 0, 0);
    world_point[0] = b[0] * b[3];
    world_point[1] = b[1] * b[3];
    world_point[2] = b[2] * b[3];
    return world_point;
}

vec3 compute_color_phong(vec3 point, vec3 normal){
    mat4 MV = view * model;
    vec4 point_mv = MV * vec4(point, 1.0);
    vec3 normal_mv = mat3(transpose(inverse(MV))) * normal;
    vec3 light_dir = light_pos - point_mv.xyz;
    vec3 view_dir = -point_mv.xyz;
    color = vec3(0, 0, 0);
    color += Ia*ka;
    vec3 N = normalize(normal_mv);
    vec3 L = normalize(light_dir);
    float lambert = dot(N,L);
    if(lambert > 0.0) {
        color += Id*kd*lambert;
        vec3 V = normalize(view_dir);
        vec3 R = reflect(-L,N);
        color += Is*ks*pow(max(dot(R,V), 0.0), p);
    }
	return color;
}

vec3 compute_color_gouraud(vec3 point, vec3 normal){
    mat4 MV = view * model;
    vec4 point_mv = MV * vec4(point, 1.0);
    vec3 normal_mv = mat3(transpose(inverse(MV))) * normal;
    vec3 light_dir = light_pos - point_mv.xyz;
    vec3 view_dir = -point_mv.xyz;
    color = vec3(0, 0, 0);

    color += Ia*ka;
    vec3 N = normalize(normal_mv);
    vec3 L = normalize(light_dir);
    float lambert = dot(N,L);
    if(lambert > 0.0) {
        color += Id*kd*lambert;
        vec3 V = normalize(view_dir);
        vec3 R = reflect(-L,N);
        color += Is*ks*pow(max(dot(R,V), 0.0), p);
    }
	return color;
}


vec3 ray_sphere_intersection(vec3 c, float r, vec3 p, vec3 v, inout vec3 normal) {
    float A = dot(v, v);
    float B = -2 * dot(c - p, v);
    float C = dot(c - p, c - p) - r*r;
    float D = B*B - 4 * A*C;
    float t1 = RAND_MAX;
    float t2 = RAND_MAX;
    vec3 i1, i2;
    if (D >= 0) {
        t1 = (-B - sqrt(D)) / 2 / A;
        t2 = (-B + sqrt(D)) / 2 / A;
        i1 = p + t1 * v;
        i2 = p + t2 * v;
    }    
    vec3 i = vec3(RAND_MAX, RAND_MAX, RAND_MAX);
    if (abs(t1) < abs(t2)) {
        i = i1;
    }
    if (abs(t1) > abs(t2)) {
        i = i2;
    }	
	normal = normalize(i - c);
    return i;
}

vec3 ray_cylinder_intersection(vec3 pa, vec3 va, float r, vec3 p, vec3 v, inout vec3 normal) {
    vec3 i = vec3(RAND_MAX, RAND_MAX, RAND_MAX);
    vec3 delta_p = p - pa;
    vec3 e = v - dot(v, va) * va;
    float f = dot(v, va);
    vec3 g = delta_p - dot(delta_p, va) * va;
    float h = dot(delta_p, va);
    float A = dot(e, e);
    float B = 2 * dot(e, g);
    float C = dot(g, g) - r * r;
    float D = B*B - 4 * A*C;
    float t1 = RAND_MAX;
    float t2 = RAND_MAX;
    vec3 i1 = vec3(RAND_MAX, RAND_MAX, RAND_MAX);
    vec3 i2 = vec3(RAND_MAX, RAND_MAX, RAND_MAX);
    if (D >= 0) {
        t1 = (-B - sqrt(D)) / 2 / A;
        t2 = (-B + sqrt(D)) / 2 / A;
        i1 = p + t1 * v;
        i2 = p + t2 * v;
        if (dot(va, i1 - pa) > 0) t1 = RAND_MAX;
        if (dot(va, i2 - pa) > 0) t2 = RAND_MAX;
    }
    
    if (length(p - i1) < length(p - i2)) {
        i = i1;
    }

    if (length(p - i2) < length(p - i1)) {
        i = i2;
    }
	// Find normal
	vec3 c = pa + dot(i - pa, va) * va;
	normal = normalize(i - c); 
    return i;
}

vec3 ray_cone_intersection(vec3 pa, vec3 va, float alpha, vec3 p, vec3 v, inout vec3 normal) {
    vec3 i = vec3(RAND_MAX, RAND_MAX, RAND_MAX);
    float cos2 = cos(alpha) * cos(alpha);
    float sin2 = sin(alpha) * sin(alpha);
    vec3 delta_p = p - pa;
    vec3 e = v - dot(v, va) * va;
    float f = dot(v, va);
    vec3 g = delta_p - dot(delta_p, va) * va;
    float h = dot(delta_p, va);
    float A = cos2 * dot(e, e) - sin2 * f * f;
    float B = 2 * cos2 * dot(e, g) - 2 * sin2 * f * h;
    float C = cos2 * dot(g, g) - sin2 * h * h;
    float D = B*B - 4 * A*C;
    float t1 = RAND_MAX;
    float t2 = RAND_MAX;
    vec3 i1 = vec3(RAND_MAX, RAND_MAX, RAND_MAX);
    vec3 i2 = vec3(RAND_MAX, RAND_MAX, RAND_MAX);
    if (D >= 0) {
        t1 = (-B - sqrt(D)) / 2 / A;
        t2 = (-B + sqrt(D)) / 2 / A;
        i1 = p + t1 * v;
        i2 = p + t2 * v;
        if (dot(va, i1 - pa) > 0) t1 = RAND_MAX;
        if (dot(va, i2 - pa) > 0) t2 = RAND_MAX;
    }
    
    if (length(p - i1) < length(p - i2)) {
        i = i1;
    }

    if (length(p - i2) < length(p - i1)) {
        i = i2;
    }

	// Find normal - fix this
	float l = length(pa - i);
	vec3 c = pa - l * va;
	normal = normalize(i - c); 

    return i;
}

vec3 ray_triangle_intersection(vec3 p0, vec3 p1, vec3 p2, vec3 o, vec3 d, inout vec3 normal) {
    vec3 i = vec3(RAND_MAX, RAND_MAX, RAND_MAX);  
    vec3 e1 = p1 - p0;
    vec3 e2 = p2 - p0;
    vec3 q = cross(d, e2);
    float a = dot(e1, q);     

	// the vector is parallel to the plane(the intersection is at infinity)
    if (a > -epsilon && a < epsilon) {  
        return i;		
    }    
    float f = 1 / a;
    vec3 s = o - p0;
    float u = f * dot(s, q);   
	 
	// the intersection is outside of the triangle
    if (u < 0.0) {        
        return i;
    }    
    vec3 r = cross(s, e1);
    float v = f * dot(d, r); 
	   
	// the intersection is outside of the triangle
    if ( v <0.0 || u + v > 1.0) {        
        return i;
    }    
    float t = f * dot(e2, r); 
    i = o + t * d;
	normal = normalize(cross(e1, e2));
    return i;
}

vec3 ray_convsegment_intersection(vec3 c1, vec3 c2, float r1, float r2, vec3 p, vec3 v, inout vec3 normal) {

	// Ray - sphere intersection
	vec3 n1 = vec3(0, 0, 0);
    vec3 i1 = ray_sphere_intersection(c1, r1, p, v, n1);
    
    // Ray - sphere intersection
	vec3 n2 = vec3(0, 0, 0);
    vec3 i2 = ray_sphere_intersection(c2, r2, p, v, n2);
    
	// Ray - cone intersections
    vec3 n = (c2 - c1) / length(c2 - c1);
	vec3 i = vec3(RAND_MAX, RAND_MAX, RAND_MAX);

	vec3 n12 = vec3(0, 0, 0);
	vec3 i12, s1, s2;
	if (r1 - r2 < epsilon) {
		i12 = ray_cylinder_intersection(c2, n, r1, p, v, n12);
		s1 = c1;
		s2 = c2;
	}
	else {	
		float beta = asin((r1 - r2) / length(c1 - c2));
		float eta1 = r1 * sin(beta);
		s1 = c1 + eta1 * n;
		float eta2 = r2 * sin(beta);
		s2 = c2 + eta2 * n;
		vec3 z = c1 + (c2 - c1) * r1 / (r1 - r2);
		float r = r1 * cos(beta);
		float h = length(z - s1);
		float alpha = atan(r / h); 
		i12 = ray_cone_intersection(z, n, alpha, p, v, n12);		
	}   

	if (dot(n, i12 - s1) >= 0 && dot(n, i12 - s2) <= 0 && length(i12) < RAND_MAX) {
		i = i12;
		normal = n12;
	}       
	if (dot(n, i1 - s1) < 0 && length(i1) < RAND_MAX) {
		i = i1;
		normal = n1;
	}        
	if (dot(n, i2 - s2) > 0 && length(i2) < RAND_MAX) {
		i = i2;
		normal = n2;
	}    	

    return i;
}

vec3 ray_convtriangle_intersection(vec3 c1, vec3 c2, vec3 c3, vec3 v1, vec3 v2, vec3 v3,
        vec3 u1, vec3 u2, vec3 u3, float r1, float r2, float r3, vec3 p, vec3 v, inout vec3 normal) {
    
    vec3 n1 = vec3(0, 0, 0);
	vec3 n2 = vec3(0, 0, 0);
	vec3 n3 = vec3(0, 0, 0); 
	vec3 n4 = vec3(0, 0, 0); 
	vec3 n5 = vec3(0, 0, 0);
    vec3 i1 = ray_convsegment_intersection(c1, c2, r1, r2, p, v, n1);
    vec3 i2 = ray_convsegment_intersection(c1, c3, r1, r3, p, v, n2);
    vec3 i3 = ray_convsegment_intersection(c2, c3, r2, r3, p, v, n3);
    vec3 i4 = ray_triangle_intersection(v1, v2, v3, p, v, n4);
    vec3 i5 = ray_triangle_intersection(u1, u2, u3, p, v, n5);
    
    float min_value = distance(p, i1);  
	vec3 i = i1;
	normal = n1;
	if (distance(p, i2) < min_value) {
		min_value = distance(p, i2);
		i = i2;
		normal = n2; 		
	}
	if (distance(p, i3) < min_value) {
		min_value = distance(p, i3);
		i = i3;
		normal = n3; 		
	}
	if (distance(p, i4) < min_value) {
		min_value = distance(p, i4);
		i = i4;
		normal = n4; 		
	}
	if (distance(p, i5) < min_value) {
		min_value = distance(p, i5);
		i = i5;
		normal = n5; 		
	}	
	if (dot(normal, i - c1) < 0) normal = -normal;
	
    return i;
}


vec3 ray_model_intersection(vec3 p, vec3 d, inout vec3 min_normal, inout int block_index) {
    const int RAND_MAX = 32767;
    vec3 i; vec3 normal = vec3(0, 0, 0);
    vec3 min_i = vec3(RAND_MAX, RAND_MAX, RAND_MAX);
    float min_distance = RAND_MAX;
    vec3 c1, c2, c3, v1, v2, v3, u1, u2, u3;
    float r1, r2, r3;

	int j_start = 0;
	int j_end = num_blocks;
	/*if (block_index > 0) {
		j_start = block_index;
		j_end = block_index + 1;
	}*/
	
    for (int j = j_start; j < j_end; j++) {
		//if (j >= 20 && j <= 25) continue;
        ivec3 block = blocks[j];       
        if (block[2] < RAND_MAX) {
            c1 = centers[block[0]]; c2 = centers[block[1]]; c3 = centers[block[2]];
            r1 = radii[block[0]]; r2 = radii[block[1]]; r3 = radii[block[2]];
            v1 = tangents_v1[j]; v2 = tangents_v2[j]; v3 = tangents_v3[j];
            u1 = tangents_u1[j]; u2 = tangents_u2[j]; u3 = tangents_u3[j];
            i = ray_convtriangle_intersection(c1, c2, c3, v1, v2, v3, u1, u2, u3, r1, r2, r3, p, d, normal);
            if (length(p - i) < min_distance) {
                min_distance = length(p - i);
                min_i = i;
				min_normal = normal;
				block_index = j;
            }
        }
        if (block[2] >= RAND_MAX) {
            c1 = centers[block[0]]; c2 = centers[block[1]];
            r1 = radii[block[0]]; r2 = radii[block[1]];
            i = ray_convsegment_intersection(c1, c2, r1, r2, p, d, normal);
            if (length(p - i) < min_distance ) {
                min_distance = length(p - i);
                min_i = i;
				min_normal = normal;
				block_index = j;
            }
        }
    }
    return min_i;
}

vec2 project_point_on_segment(vec2 p, vec2 c1, vec2 c2) {
	vec2 u = c2 - c1;
	vec2 v = p - c1;
	float alpha = dot(u, v) / dot(u, u);
	if (alpha <= 0) return c1;
	if (alpha > 0 && alpha < 1) return c1 + alpha * u;
	if (alpha >= 1) return c2;
}

float size = 12.0;
float alpha = 0.5;
vec3 green = vec3(0.13, 0.70, 0.30);
vec3 gark_green = vec3(0.0, 0.56, 0.25);
vec3 blue = vec3(0.06, 0.70, 0.95);
vec3 dark_blue = vec3(0.0, 0.44, 0.75);
vec3 magenta = vec3(0.8, 0, 0.6);//vec3(0.24, 0.50, 0.47); //vec3(0.7, 0.3, 0.4)

vec4 compute_color(vec4 skin_color, vec4 grey_color, float certainty_value) {
	if (certainty_value > 1) certainty_value = 1;
	if (certainty_value < 0.1) certainty_value = 0.1;
	skin_color = mix(grey_color, skin_color, vec4(0.45 + 0.55 * certainty_value));
	return skin_color;
}

vec4 compute_length_color(vec2 p, int index1, int index2, vec4 model_color, vec4 grey_color) {

	if (index1 < index2) {
		int temp = index1; index1 = index2; index2 = temp;
	}
	vec2 c1 = project(centers[index1]);
	vec2 c2 = project(centers[index2]);
	vec2 s = project_point_on_segment(p, c1, c2);
	float u = length(s - c2) / length(c1 - c2);
	float v = length(s - c1) / length(c1 - c2);

	v = v * v * v;
	u = 1 - v;
	
	float certainty_value = u * center_index_to_length_certainty_map[index1] + v * center_index_to_length_certainty_map[index2];
	vec4 color = compute_color(model_color, grey_color, certainty_value);

	if (index1 == 19 && u == 1) {
		vec2 v1 = project(centers[index1] + radii[index1] * vec3(1, 0, 0));
		float r1 = length(c1 - v1);
		v = length(p - c1) / r1;
		v = v * v * v;
		u = 1 - v;
		float certainty_value = u * center_index_to_length_certainty_map[index1] + v * center_index_to_x_certainty_map[26];
		color = compute_color(model_color, grey_color, certainty_value);	
	}

	return color;
}

void compute_triangle_coordinates(vec3 p, vec3 a, vec3 b, vec3 c, inout float u, inout float v, inout float w) {
	vec3 v0 = b - a;
	vec3 v1 = c - a;
	vec3 v2 = p - a;
	float d00 = dot(v0, v0);
	float d01 = dot(v0, v1);
	float d11 = dot(v1, v1);
	float d20 = dot(v2, v0);
	float d21 = dot(v2, v1);
	float denom = d00 * d11 - d01 * d01;
	
	v = (d11 * d20 - d01 * d21) / denom;
	w = (d00 * d21 - d01 * d20) / denom;
	u = 1.0f - v - w;
}

vec4 compute_uncertainty_color(vec2 pixel, vec3 model_point, int block_index, vec4 color) {



	float mean_color_channel = (color[0] + color[1] + color[2]) / 3;
	vec4 grey_color = vec4(185.0/255, 185.0/255, 255.0/255, 1.0);

	float certainty_value = 0;

	if (block_index == 27 || block_index == 34) { /// thumb additional
		color = compute_length_color(pixel, 17, 16, color, grey_color);
	}
	if (block_index == 33) { /// thumb fold
		color = compute_length_color(pixel, 18, 17, color, grey_color);
	}
	if (block_index == 14) { /// thumb bottom
		color = compute_length_color(pixel, 19, 18, color, grey_color);
	}

	if (block_index == 26 || block_index == 30 || block_index == 31) { /// thumb fold
		vec2 c1 = project(centers[19]);
		vec2 c2 = project(centers[24]);
		vec2 s = project_point_on_segment(pixel, c1, c2);
		float u = length(s - c2) / length(c1 - c2);
		float v = length(s - c1) / length(c1 - c2);		
		float certainty_value = u * center_index_to_length_certainty_map[19] + v * center_index_to_x_certainty_map[23];
		color = compute_color(color, grey_color, certainty_value);
	}

	if (block_index == 15 || block_index == 16 || block_index == 17 || block_index == 18 || block_index == 19) { /// palm
		vec2 c20 = project(centers[20]); vec2 c23 = project(centers[23]); vec2 c26 = project(centers[26]); vec2 c27 = project(centers[27]);
		float u1, v1, w1; compute_triangle_coordinates(vec3(pixel, 1), vec3(c20, 1), vec3(c23, 1), vec3(c27, 1), u1, v1, w1);
		float u2, v2, w2; compute_triangle_coordinates(vec3(pixel, 1), vec3(c20, 1), vec3(c26, 1), vec3(c27, 1), u2, v2, w2);
		
			if (v2 > 0) {
				if (u2 < 0) u2 = 0; if (v2 < 0) v2 = 0; if (w2 < 0) w2 = 0;
				certainty_value = u2 * center_index_to_x_certainty_map[20] + v2 * center_index_to_x_certainty_map[26] + w2 * center_index_to_x_certainty_map[19];				
			}
			else {
				if (u1 < 0) u1 = 0; if (v1 < 0) v1 = 0; if (w1 < 0) w1 = 0;
				certainty_value = u1 * center_index_to_x_certainty_map[20] + v1 * center_index_to_x_certainty_map[23] +  w1 * center_index_to_x_certainty_map[19];
			}	
			color = compute_color(color, grey_color, certainty_value);
	}
	if (block_index == 20 || block_index == 21 || block_index == 22 || block_index == 23 || block_index == 24 || block_index == 25) { /// membranes
		int index1 = blocks[block_index][0]; int index2 = blocks[block_index][1]; int index3 = blocks[block_index][2];
		vec2 c1 = project(centers[index1]); vec2 c2 = project(centers[index2]); vec2 c3 = project(centers[index3]);
		float u, v, w; compute_triangle_coordinates(vec3(pixel, 1), vec3(c1, 1), vec3(c2, 1), vec3(c3, 1), u, v, w);

		float value1, value2, value3;
		value1 = center_index_to_x_certainty_map[index1];
		if (index2 == 20 || index2 == 21 || index2 == 22 || index2 == 23) 
			value2 = center_index_to_x_certainty_map[index2];
		else
			value2 = center_index_to_y_certainty_map[index2];

		if (index1 == 21) value1 = 0.57 * center_index_to_x_certainty_map[20] + 0.43 * center_index_to_x_certainty_map[23];
		if (index1 == 22) value1 = 0.72 * center_index_to_x_certainty_map[23] + 0.28 * center_index_to_x_certainty_map[20];

		if (index2 == 21) value2 = 0.57 * center_index_to_x_certainty_map[20] + 0.43 * center_index_to_x_certainty_map[23];
		if (index2 == 22) value2 = 0.72 * center_index_to_x_certainty_map[23] + 0.28 * center_index_to_x_certainty_map[20];
		
		value3 = center_index_to_y_certainty_map[index3];
		certainty_value = u * value1 + v * value2 + w * value3;
		color = compute_color(color, grey_color, certainty_value);
	}

	if (block_index == 28 || block_index == 29) { ///  wrist
		certainty_value = center_index_to_x_certainty_map[36];
		color = compute_color(color, grey_color, certainty_value);
	}

	if (block_index == 32) { /// thumb fold
		color = compute_color(color, grey_color, center_index_to_length_certainty_map[19]);
	}

	if (blocks[block_index][2] == RAND_MAX) { /// fingers	
		color = compute_length_color(pixel, blocks[block_index][0], blocks[block_index][1], color, grey_color);
	}
	return color;
}

void main() {

	if (texture(silhouette, uv).r == 1.0) {	
		color_with_alpha = vec4(1, 1, 1, 1);
		gl_FragDepth = 1;
		return;
	}
	// retrieve block indices from the texture;
	int block_index = int(texture(silhouette, uv).r * 255);
	float difference = abs(block_index - texture(silhouette, uv).r * 255);
	if (difference > 0.00001 || block_index > num_blocks) block_index = -1;

    const int RAND_MAX = 32767;
	vec2 pixel = vec2(gl_FragCoord.x, gl_FragCoord.y);
    vec3 p1 = unproject(pixel[0], pixel[1], 0); 
    vec3 p2 = unproject(pixel[0], pixel[1], 1);
    vec3 ray_direction = normalize(p2 - p1);
    vec3 normal = vec3(0, 0, 0);

	vec3 i = ray_model_intersection(camera_center, ray_direction, normal, block_index);
	    
	float d = distance(camera_center, i);
	if (d < RAND_MAX / 3) {
		color = compute_color_gouraud(i, normal);
		color = mix(texture(synthetic_texture, uv).rgb, color, vec3(0.315));		

		if (display_certainty) {
			color_with_alpha = compute_uncertainty_color(pixel, i, block_index, vec4(color, 1.0));			
		}
		else color_with_alpha = vec4(color, 1.0);

		int period = 60;
		if (num_frames_since_calibrated >= 0 && num_frames_since_calibrated < 2 * period) {
			float brightess;
			if (num_frames_since_calibrated < period)
				brightess = (1 + 0.25 * (num_frames_since_calibrated % period) / period);
			else
				brightess = (1 + 0.25 * (period - num_frames_since_calibrated % period) / period);
			color = brightess * color;
			color_with_alpha = vec4(color, 1.0);
		}

		

		// find depth
		vec4 point_gl =  MVP * vec4(i, 1.0);
		float z = point_gl[2] / point_gl[3];
		gl_FragDepth = z * (gl_DepthRange.far - gl_DepthRange.near) / 2 + (gl_DepthRange.near + gl_DepthRange.far) / 2;

		if (wristband_found == 0) {
			float mean_color = 1.08 * (color[0] + color[1] + color[2]) / 3;
			color_with_alpha = vec4(mean_color, mean_color, mean_color, 1);
		}
	}
    else {
		color_with_alpha = vec4(1, 1, 1, 1);

		gl_FragDepth = 1;
	}	

	//framebuffer_color = vec3(color_with_alpha);
}



