#pragma once
#include "tracker/ForwardDeclarations.h"
#include "tracker/Types.h"

class OffscreenRenderer{
protected:
    Camera* camera = NULL;
	Model * model;	
	bool render_depth;
public:
    CustomFrameBuffer* frame_buffer = NULL; 
	ConvolutionRenderer * convolution_renderer;

    void init(Camera *camera, Model * model, std::string data_path, bool render_block_id);
    void render_offscreen(bool last_iter, bool fingers_only, bool use_indicator_texure);
	~OffscreenRenderer();
};

