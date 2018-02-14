#pragma once
#include "util/gl_wrapper.h"
#include "opencv2/core/core.hpp" 


class CustomFrameBuffer {
private:
    int image_width;
    int image_height;
    GLuint framebuffer; // to bind the proper targets    
    GLuint color_tex;   // where we render the face indices
	GLuint normals_tex;
	GLuint depth_tex;
    bool  needs_cleanup;
	bool render_depth;
	GLuint create_framebuffer();

public:

	CustomFrameBuffer();
    
	CustomFrameBuffer(int image_width, int image_height, bool render_block_id);

	~CustomFrameBuffer();

	GLuint silhouette_texture_id() { return color_tex; }
	GLuint depth_texture_id() { return depth_tex; }

	bool ready();

	void bind();

	void unbind();

	void init(int image_width, int image_height, bool render_block_id);

	void cleanup();

	GLuint create_color_attachment(int image_width, int image_height);
	
	GLuint create_depth_attachment(int image_width, int image_height);

	GLuint create_normals_attachment(int image_width, int image_height);

	void fetch_color_attachment(cv::Mat& image);

	void fetch_depth_attachment(cv::Mat& image);

	void fetch_normals_attachment(cv::Mat& image);

	void display_color_attachment();	

	void display_depth_attachment();

	void display_normals_attachment();
	
};
