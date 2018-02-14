#include "OffscreenRenderer.h"

#include "util/tictoc.h"
#include "util/gl_wrapper.h"
#include "util/mylogger.h"
#include "tracker/Data/Camera.h"
#include "tracker/OpenGL/ConvolutionRenderer/ConvolutionRenderer.h"
#include "tracker/OpenGL/CustomFrameBuffer.h"

void OffscreenRenderer::init(Camera* camera, Model * model, std::string data_path, bool render_depth) {
	this->camera = camera;
	this->model = model;
	this->render_depth = render_depth;
	
	frame_buffer = new CustomFrameBuffer(camera->width(), camera->height(), render_depth);
	convolution_renderer = new ConvolutionRenderer(model, ConvolutionRenderer::FRAMEBUFFER, camera->view_projection_matrix(), data_path, camera->height(), camera->width());
}

OffscreenRenderer::~OffscreenRenderer() {
	delete frame_buffer;
	//delete convolution_renderer;
}

void OffscreenRenderer::render_offscreen(bool last_iter, bool fingers_only, bool use_indicator_texture) {
	
	glViewport(0, 0, camera->width(), camera->height());
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glDisable(GL_BLEND); ///< just in case
	glEnable(GL_DEPTH_TEST);

	frame_buffer->bind();
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	convolution_renderer->render_offscreen(fingers_only, use_indicator_texture);
	frame_buffer->unbind();

	bool display = false;
	if (display) {
		frame_buffer->display_color_attachment();
		if (render_depth) frame_buffer->display_depth_attachment();
	}

	try {
		if (last_iter) frame_buffer->fetch_color_attachment(model->silhouette_texture);
	}
	catch (const std::exception& e) { cout << "exception" << endl; }

	glFinish();
}


