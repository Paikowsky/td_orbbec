/* Shared Use License: This file is owned by Derivative Inc. (Derivative) and
 * can only be used, and/or modified for use, in conjunction with 
 * Derivative's TouchDesigner software, and only if you are a licensee who has
 * accepted Derivative's TouchDesigner license or assignment agreement (which
 * also govern the use of this file).  You may share a modified version of this
 * file with another authorized licensee of Derivative's TouchDesigner software.
 * Otherwise, no redistribution or sharing of this file, with or without
 * modification, is permitted.
 */

#include "OpenGLTOP.h"

#include <assert.h>
#ifdef __APPLE__
#include <OpenGL/gl3.h>
#include <string.h>
#endif
#include <cstdio>
#include <minmax.h>
#include <algorithm>

#include <iostream>
using namespace std;

 // OpenNI2 headers
#include <OpenNI.h> 
using namespace openni;

OniRGB888Pixel* gl_texture;
VideoStream depthSensor;
//VideoStream selectedSensor;
Device device;

// Create the OpenGL texture
GLuint myTextureHandle;

static const char *vertexShader = "#version 330\n\
uniform mat4 uModelView; \
in vec3 P; \
void main() { \
    gl_Position = vec4(P, 1) * uModelView; \
}";

static const char *fragmentShader = "#version 330\n\
uniform vec4 uColor; \
out vec4 finalColor; \
void main() { \
    finalColor = uColor; \
}";

static const char *uniformError = "A uniform location could not be found.";

// These functions are basic C function, which the DLL loader can find
// much easier than finding a C++ Class.
// The DLLEXPORT prefix is needed so the compile exports these functions from the .dll
// you are creating
extern "C"
{
DLLEXPORT
TOP_PluginInfo
GetTOPPluginInfo(void)
{
	TOP_PluginInfo info;
	// This must always be set to this constant
	info.apiVersion = TOPCPlusPlusAPIVersion;

	// Change this to change the executeMode behavior of this plugin.
	info.executeMode = TOP_ExecuteMode::OpenGL_FBO;

	return info;
}

DLLEXPORT
TOP_CPlusPlusBase*
CreateTOPInstance(const OP_NodeInfo* info, TOP_Context *context)
{
	// Return a new instance of your class every time this is called.
	// It will be called once per TOP that is using the .dll

    // Note we can't do any OpenGL work during instantiation

	return new OpenGLTOP(info, context);
}

DLLEXPORT
void
DestroyTOPInstance(TOP_CPlusPlusBase* instance, TOP_Context *context)
{
	// Delete the instance here, this will be called when
	// Touch is shutting down, when the TOP using that instance is deleted, or
	// if the TOP loads a different DLL

    // We do some OpenGL teardown on destruction, so ask the TOP_Context
    // to set up our OpenGL context
    context->beginGLCommands();

	delete (OpenGLTOP*)instance;

    context->endGLCommands();
	
	depthSensor.destroy();
	device.close();
	OpenNI::shutdown();
}

};




OpenGLTOP::OpenGLTOP(const OP_NodeInfo* info, TOP_Context *context)
: myNodeInfo(info), myExecuteCount(0), myRotation(0.0), myError(nullptr),
    myProgram(), myDidSetup(false), myModelViewUniform(-1), myColorUniform(-1)
{

#ifdef WIN32
	// GLEW is global static function pointers, only needs to be inited once,
	// and only on Windows.
	static bool needGLEWInit = true;
	if (needGLEWInit)
	{
		needGLEWInit = false;
		context->beginGLCommands();
		// Setup all our GL extensions using GLEW
		glewInit();
		context->endGLCommands();
	}
#endif


	// If you wanted to do other GL initialization inside this constructor, you could
	// uncomment these lines and do the work between the begin/end
	//
	//context->beginGLCommands();
	// Custom GL initialization here
	//context->endGLCommands();
}

OpenGLTOP::~OpenGLTOP()
{

}

void
OpenGLTOP::getGeneralInfo(TOP_GeneralInfo* ginfo)
{
	// Setting cookEveryFrame to true causes the TOP to cook every frame even
	// if none of its inputs/parameters are changing. Set it to false if it
    // only needs to cook when inputs/parameters change.
	ginfo->cookEveryFrame = true;
}

bool
OpenGLTOP::getOutputFormat(TOP_OutputFormat* format)
{
	// In this function we could assign variable values to 'format' to specify
	// the pixel format/resolution etc that we want to output to.
	// If we did that, we'd want to return true to tell the TOP to use the settings we've
	// specified.
	// In this example we'll return false and use the TOP's settings
	format->width = 640;
	format->height = 480;
	return true;
}


void
OpenGLTOP::execute(const TOP_OutputFormatSpecs* outputFormat ,
							OP_Inputs* inputs,
							TOP_Context* context)
{
	myExecuteCount++;

	// These functions must be called before
	// beginGLCommands()/endGLCommands() block

    int width = outputFormat->width;
    int height = outputFormat->height;

    float ratio = static_cast<float>(height) / static_cast<float>(width);

    context->beginGLCommands();
    
    setupGL_ONI();

    if (!myError)
    {
        glViewport(0, 0, 640, 480);
        glClearColor(0.0, 0.0, 0.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT);

		if (depthSensor.isValid())
		{
			Status status = STATUS_OK;

			//grab one frame and save as a pointer object
			VideoStream* streamPointer = &depthSensor;
			int streamReadyIndex;

			status = OpenNI::waitForAnyStream(&streamPointer, 1,
				&streamReadyIndex, 500);

			//if you have good status and streamReady == 0 add new frame
			if (status == STATUS_OK && streamReadyIndex == 0)
			{
				VideoFrameRef newFrame;
				status = depthSensor.readFrame(&newFrame);
				
				if (status == STATUS_OK && newFrame.isValid())
				{

					// Clear the OpenGL buffers
					glClear(
						GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

					// Setup the OpenGL viewpoint
					glMatrixMode(GL_PROJECTION);
					glPushMatrix();
					glLoadIdentity();
					glOrtho(0, 640, 480, 0, -1.0, 1.0);

					// UPDATING TEXTURE (DEPTH 1MM TO RGB888)
					//find max depth value and normalize image based on this
					unsigned short maxDepth = 0;
					for (int y = 0; y < newFrame.getHeight(); ++y)
					{
						DepthPixel* depthCell = (DepthPixel*)(
							(char*)newFrame.getData() +
							(y * newFrame.getStrideInBytes())
							);
						for (int x = 0; x < newFrame.getWidth();
							++x, ++depthCell)
						{
							if (maxDepth < *depthCell) {
								maxDepth = *depthCell;
							}
						}
					}

					//global resize to 640 x 480
					double resizeFactor = min(
						(640 / (double)newFrame.getWidth()),
						(480 / (double)newFrame.getHeight()));
					unsigned int texture_x = (unsigned int)(640 -
						(resizeFactor * newFrame.getWidth())) / 2;
					unsigned int texture_y = (unsigned int)(480 -
						(resizeFactor * newFrame.getHeight())) / 2;

					//calculate position, convert to RGB, and copy each pixel to texture buffer
					for (unsigned int y = 0;
						y < (480 - 2 * texture_y); ++y)
					{
						//3 Byte Pixel structure RGB
						OniRGB888Pixel* texturePixel = gl_texture +
							((y + texture_y) * 640) + texture_x;
						for (unsigned int x = 0;
							x < (640 - 2 * texture_x);
							++x)
						{
							DepthPixel* streamPixel =
								(DepthPixel*)(
								(char*)newFrame.getData() +
									((int)(y / resizeFactor) *
										newFrame.getStrideInBytes())
									) + (int)(x / resizeFactor);
							if (*streamPixel != 0) {
								char depthValue = ((float)*streamPixel /
									maxDepth) * 255;
								texturePixel->b = 255 - depthValue;
								texturePixel->g = 255 - depthValue;
								texturePixel->r = 255 - depthValue;
							}
							else
							{
								texturePixel->b = 0;
								texturePixel->g = 0;
								texturePixel->r = 0;
							}
							texturePixel += 1; // Moves variable by 3 bytes
						}
					}
					
					// Bind Texture to use on current render (quad)
					glBindTexture(GL_TEXTURE_2D, myTextureHandle);

					// Replace Data with pixel data
					glTexSubImage2D(GL_TEXTURE_2D,
						0,
						0,
						0,
						640,
						480,
						GL_RGB,
						GL_UNSIGNED_BYTE,
						gl_texture);


					/*  Not sure how to display this, but would love to be able to figure it out - this is reading from the initial texture buffer I wrote to.  
						I think I need to do buffer swapping to have this work maybe?
					
					// 0x8191 = GL_GENERATE_MIPMAP
					glTexParameteri(GL_TEXTURE_2D,
						0x8191,
						GL_TRUE);

					glTexImage2D(GL_TEXTURE_2D, 
						0, 
						GL_RGB,
						640, 480,
						0,
						GL_RGB,
						GL_UNSIGNED_BYTE,
						gl_texture);

					glGetTexImage(GL_TEXTURE_2D, 
						0, 
						GL_RGB, 
						GL_UNSIGNED_BYTE, 
						gl_texture);
					*/

					//enable 2D texturing
					glEnable(GL_TEXTURE_2D);
					glDisable(GL_DEPTH_TEST);
					
					//draw
					glBegin(GL_QUADS);

					glTexCoord2f(0.0f, 0.0f);
					glVertex3f(0.0f, 0.0f, 0.0f);
					
					glTexCoord2f(0.0f, 1.0f);
					glVertex3f(0.0f, (float)480, 0.0f);
					
					glTexCoord2f(1.0f, 1.0f);
					glVertex3f((float)640, (float)480, 0.0f);
					
					glTexCoord2f(1.0f, 0.0f);
					glVertex3f((float)640, 0.0f, 0.0f);
					
					glEnd();
					
					glDisable(GL_TEXTURE_2D);
					//glDisable(GL_DEPTH_TEST);
					

				}
			}
		}

		// Tidy up
        glBindVertexArray(0);

    }

    context->endGLCommands();
}

int32_t
OpenGLTOP::getNumInfoCHOPChans()
{
	// We return the number of channel we want to output to any Info CHOP
	// connected to the TOP. In this example we are just going to send one channel.
	return 2;
}

void
OpenGLTOP::getInfoCHOPChan(int32_t index,
										OP_InfoCHOPChan* chan)
{
	// This function will be called once for each channel we said we'd want to return
	// In this example it'll only be called once.

	if (index == 0)
	{
		chan->name = "executeCount";
		chan->value = (float)myExecuteCount;
	}

	if (index == 1)
	{
		chan->name = "cameraStream";
		chan->name = "depth";
	}
}

bool		
OpenGLTOP::getInfoDATSize(OP_InfoDATSize* infoSize)
{
	infoSize->rows = 2;
	infoSize->cols = 2;
	// Setting this to false means we'll be assigning values to the table
	// one row at a time. True means we'll do it one column at a time.
	infoSize->byColumn = false;
	return true;
}

void
OpenGLTOP::getInfoDATEntries(int32_t index,
										int32_t nEntries,
										OP_InfoDATEntries* entries)
{
	// It's safe to use static buffers here because Touch will make it's own
	// copies of the strings immediately after this call returns
	// (so the buffers can be reuse for each column/row)
	static char tempBuffer1[4096];
	static char tempBuffer2[4096];

	if (index == 0)
	{
		// Set the value for the first column
#ifdef WIN32
		strcpy_s(tempBuffer1, "executeCount");
#else // macOS
        strlcpy(tempBuffer1, "executeCount", sizeof(tempBuffer1));
#endif
		entries->values[0] = tempBuffer1;

		// Set the value for the second column
#ifdef WIN32
		sprintf_s(tempBuffer2, "%d", myExecuteCount);
#else // macOS
        snprintf(tempBuffer2, sizeof(tempBuffer2), "%d", myExecuteCount);
#endif
		entries->values[1] = tempBuffer2;
	}

	if (index == 1)
	{
		// Set the value for the first column
#ifdef WIN32
		strcpy_s(tempBuffer1, "cameraStream");
#else // macOS
		strlcpy(tempBuffer1, "cameraStream", sizeof(tempBuffer1));
#endif
		entries->values[0] = tempBuffer1;

		// Set the value for the second column
#ifdef WIN32
		sprintf_s(tempBuffer2,"depth");
#else // macOS
		snprintf(tempBuffer2, sizeof(tempBuffer2), "%g", "depth");
#endif
		entries->values[1] = tempBuffer2;
	}
}

const char *
OpenGLTOP::getErrorString()
{
    return myError;
}

void
OpenGLTOP::setupParameters(OP_ParameterManager* manager)
{

	// speed
	{
		OP_NumericParameter	np;

		np.name = "Source";
		np.label = "Source";
		np.defaultValues[0] = 0;
		np.minSliders[0] = 0;
		np.maxSliders[0] =  3;
		
		/* TODO add in source switcher for openNI
		float source = (float)inputs->getParDouble("Source", 0);

		if (source == 1)
		{
			openNI::SetActiveSensor(SENSOR_COLOR, &device);
		}
		if (source == 0)
		{
			openNI.SetActiveSensor(SENSOR_IR, &device);
		}

		*/
		OP_ParAppendResult res = manager->appendFloat(np);
		assert(res == OP_ParAppendResult::Success);
	}


	// pulse
	{
		OP_NumericParameter	np;

		np.name = "Reset";
		np.label = "Reset";
		
		OP_ParAppendResult res = manager->appendPulse(np);
		assert(res == OP_ParAppendResult::Success);
	}

}

void
OpenGLTOP::pulsePressed(const char* name)
{
	if (!strcmp(name, "Reset"))
	{
		myRotation = 0.0;
	}
}

void OpenGLTOP::setupGL_ONI()
{

    if (myDidSetup == false)
    {
        myError = myProgram.build(vertexShader, fragmentShader);
		
		std::cout << "starting setup \r\n";
		std::cout << "program built \r\n";

        // If an error occurred creating myProgram, we can't proceed
        if (myError == nullptr)
        {
			std::cout << "setup complete \r\n";

			// setup functions
			Status status = STATUS_OK;
			std::cout << "Scanning machine for devices and loading modules/drivers... \r\n";

			status = OpenNI::initialize();
			std::cout << "OpenNI initialized \r\n";

			std::cout << "Opening first device ...\r\n";
			status = device.open(ANY_DEVICE);

			std::cout << device.getDeviceInfo().getName() << " Opened, Completed.\r\n";

			//error logging for unsupported devices
			printf("Checking if stream is supported ...\r\n");
			if (!device.hasSensor(SENSOR_DEPTH))
			{
				printf("Stream not supported by this device.\r\n");
				return;
			}

			//start depth stream
			printf("Asking device to create a depth stream ...\r\n");
			status = depthSensor.create(device, SENSOR_DEPTH);
			if (!HandleStatus(status)) return;

			std::cout << "Initializing OpenGL ...\r\n";
			gl_texture = (OniRGB888Pixel*)malloc(
				640 * 480 * sizeof(OniRGB888Pixel));

			//video mode
			std::cout << "Setting video mode to 640x480x30 Depth 1MM..\r\n";
			VideoMode vmod;
			vmod.setFps(30);
			vmod.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
			vmod.setResolution(640, 480);
			status = depthSensor.setVideoMode(vmod);
			if (!HandleStatus(status)) return;

			std::cout << "Starting stream..\r\n";
			status = depthSensor.start();
			if (!HandleStatus(status)) return;

			std::cout << "Initializing OpenGL..\r\n";
			gl_texture = (OniRGB888Pixel*)malloc(
				640 * 480 * sizeof(OniRGB888Pixel));

			std::cout << "Creating GL Texture..\r\n";
			// Create GL Texture
			glGenTextures(1, &myTextureHandle);

			// Bind it (make it the active texture)
			glBindTexture(GL_TEXTURE_2D, myTextureHandle);

			// Set some filtering options... you don't need to do this
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

			// Allocate storage for the texture
			glTexImage2D(GL_TEXTURE_2D,         // type of texture
				0,								// pyramid level (for mip-mapping) - 0 is the top level
				GL_RGB,							// internal colour format to convert to
				640,							// image width  i.e. 640 for Kinect in standard mode
				480,							// image height i.e. 480 for Kinect in standard mode
				0,								// border width in pixels (can either be 1 or 0)
				GL_RGB,							// input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
				GL_UNSIGNED_BYTE,				// image data type
				nullptr);						// the actual image data itself
        }

        myDidSetup = true;
    }
}


/*
void OpenGLTOP::setSensor(SensorType sensorType, Device* device)
{
	//Set status for error checking
	Status status = STATUS_OK;

	//depth stream not yet supported
	if (sensorType == SENSOR_COLOR)
	{
		std::cout << "COLOR Not supported yet..\r\n";
		return;
	}

	if (sensorType == SENSOR_IR)
	{
		std::cout << "IR Not supported yet..\r\n";
		return;
	}

	std::cout << "Checking if stream is supported ...\r\n";
	
	if (!device->hasSensor(sensorType))
	{
		std::cout << "Stream not supported by this device.\r\n";
		return;
	}
	
	//if sensor has valid stream, destory old streams before creating a new one
	if (selectedSensor.isValid())
	{
		std::cout << "Stop and destroy old stream.\r\n";
		selectedSensor.stop();
		selectedSensor.destroy();
	}

	//create new stream if no errors
	std::cout << "Asking device to create a stream ...\r\n";
	status = selectedSensor.create(*device, sensorType);
	if (!HandleStatus(status)) return;

	//set video mode
	std::cout << "Setting video mode to 640x480x30 RGB24 ...\r\n";
	VideoMode vmod;
	vmod.setFps(30);
	vmod.setPixelFormat(PIXEL_FORMAT_RGB888);
	vmod.setResolution(640, 480);
	status = selectedSensor.setVideoMode(vmod);
	if (!HandleStatus(status)) return;

	//start new stream
	std::cout << "Starting stream ...\r\n";
	status = selectedSensor.start();
	if (!HandleStatus(status)) return;
	std::cout << "Done.\r\n";
}
*/

bool OpenGLTOP::HandleStatus(Status status)
{
	if (status == STATUS_OK)
		return true;
	std::cout << "ERROR: #%d, %s" << status << OpenNI::getExtendedError();
	ReadLastCharOfLine();
	return false;
}


char OpenGLTOP::ReadLastCharOfLine()
{
	int newChar = 0;
	int lastChar;
	fflush(stdout);
	do
	{
		lastChar = newChar;
		newChar = getchar();
	} while ((newChar != '\n')
		&& (newChar != EOF));
	return (char)lastChar;
}
