/* Shared Use License: This file is owned by Derivative Inc. (Derivative) and
 * can only be used, and/or modified for use, in conjunction with 
 * Derivative's TouchDesigner software, and only if you are a licensee who has
 * accepted Derivative's TouchDesigner license or assignment agreement (which
 * also govern the use of this file).  You may share a modified version of this
 * file with another authorized licensee of Derivative's TouchDesigner software.
 * Otherwise, no redistribution or sharing of this file, with or without
 * modification, is permitted.
 */

#include "CPlusPlusCHOPExample.h"

#include <stdio.h>
#include <string.h>
#include <cmath>
#include <assert.h>
#include <sstream>

#include <iostream>
using namespace std;

VideoStream depthStream;
Device device;
obt::BodyTracker tracker;
//0 is world space (x,y,z) || 1 is depth space (u,v)
int space;
bool floorStaus;

// These functions are basic C function, which the DLL loader can find
// much easier than finding a C++ Class.
// The DLLEXPORT prefix is needed so the compile exports these functions from the .dll
// you are creating
extern "C"
{

DLLEXPORT
int32_t
GetCHOPAPIVersion(void)
{
	// Always return CHOP_CPLUSPLUS_API_VERSION in this function.
	return CHOP_CPLUSPLUS_API_VERSION;
}

DLLEXPORT
CHOP_CPlusPlusBase*
CreateCHOPInstance(const OP_NodeInfo* info)
{
	// Return a new instance of your class every time this is called.
	// It will be called once per CHOP that is using the .dll
	return new CPlusPlusCHOPExample(info);
}

DLLEXPORT
void
DestroyCHOPInstance(CHOP_CPlusPlusBase* instance)
{
	// Delete the instance here, this will be called when
	// Touch is shutting down, when the CHOP using that instance is deleted, or
	// if the CHOP loads a different DLL
	openni::OpenNI::shutdown();
	delete (CPlusPlusCHOPExample*)instance;
}

};


CPlusPlusCHOPExample::CPlusPlusCHOPExample(const OP_NodeInfo* info) : myNodeInfo(info)
{
	myExecuteCount = 0;
	myOffset = 0.0;
	setup_ONI();
}

CPlusPlusCHOPExample::~CPlusPlusCHOPExample()
{

}

void
CPlusPlusCHOPExample::getGeneralInfo(CHOP_GeneralInfo* ginfo)
{
	// This will cause the node to cook every frame
	ginfo->cookEveryFrameIfAsked = true;
	ginfo->timeslice = true;
	ginfo->inputMatchIndex = 0;
}

bool
CPlusPlusCHOPExample::getOutputInfo(CHOP_OutputInfo* info)
{
	// If there is an input connected, we are going to match it's channel names etc
	// otherwise we'll specify our own.
	
	//world space for x,y,z
	if (space == 0)
	{
		info->numChannels = 48;
		info->sampleRate = 120;
		return true;
	}

	//depth and uv space for x,y or u,v 
	else
	{
		info->numChannels = 32;

		// Since we are outputting a timeslice, the system will dictate
		// the numSamples and startIndex of the CHOP data
		//info->numSamples = 1;
		//info->startIndex = 0

		// For illustration we are going to output 120hz data
		info->sampleRate = 120;
		return true;
	}
}

const char*
CPlusPlusCHOPExample::getChannelName(int32_t index, void* reserved)
{
	if (space == 0)
	{
		switch (index)
		{
		case 0: return "p1/head_x";
		case 1: return "p1/head_y";
		case 2: return "p1/head_z";

		case 3: return "p1/spine_shoulder_x";
		case 4: return "p1/spine_shoulder_y";
		case 5: return "p1/spine_shoulder_z";

		case 6: return "p1/shoulder_l_x";
		case 7: return "p1/shoulder_l_y";
		case 8: return "p1/shoulder_l_z";
		
		case 9: return "p1/elbow_l_x";
		case 10: return "p1/elbow_l_y";
		case 11: return "p1/elbow_l_z";

		case 12: return "p1/hand_l_x";
		case 13: return "p1/hand_l_y";
		case 14: return "p1/hand_l_z";
		
		case 15: return "p1/shoulder_r_x";
		case 16: return "p1/shoulder_r_y";
		case 17: return "p1/shoulder_r_z";
		
		case 18: return "p1/elbow_r_x";
		case 19: return "p1/elbow_r_y";
		case 20: return "p1/elbow_r_z";
		
		case 21: return "p1/hand_r_x";
		case 22: return "p1/hand_r_y";
		case 23: return "p1/hand_r_z";

		case 24: return "p1/spine_mid_x";
		case 25: return "p1/spine_mid_y";
		case 26: return "p1/spine_mid_z";

		case 27: return "p1/spine_base_x";
		case 28: return "p1/spine_base_y";
		case 29: return "p1/spine_base_z";

		case 30: return "p1/hip_l_x";
		case 31: return "p1/hip_l_y";
		case 32: return "p1/hip_l_z";

		case 33: return "p1/knee_l_x";
		case 34: return "p1/knee_l_y";
		case 35: return "p1/knee_l_z";

		case 36: return "p1/foot_l_x";
		case 37: return "p1/foot_l_y";
		case 38: return "p1/foot_l_z";

		case 39: return "p1/hip_r_x";
		case 40: return "p1/hip_r_y";
		case 41: return "p1/hip_r_z";

		case 42: return "p1/knee_r_x";
		case 43: return "p1/knee_r_y";
		case 44: return "p1/knee_r_z";

		case 45: return "p1/foot_r_x";
		case 46: return "p1/foot_r_y";
		case 47: return "p1/foot_r_z";

		}
	}

	if (space == 1)
	{
		switch (index)
		{
		case 0: return "p1/head_x";
		case 1: return "p1/head_y";

		case 2: return "p1/spine_shoulder_x";
		case 3: return "p1/spine_shoulder_y";

		case 4: return "p1/shoulder_l_x";
		case 5: return "p1/shoulder_l_y";

		case 6: return "p1/elbow_l_x";
		case 7: return "p1/elbow_l_y";

		case 8: return "p1/hand_l_x";
		case 9: return "p1/hand_l_y";

		case 10: return "p1/shoulder_r_x";
		case 11: return "p1/shoulder_r_y";

		case 12: return "p1/elbow_r_x";
		case 13: return "p1/elbow_r_y";

		case 14: return "p1/hand_r_x";
		case 15: return "p1/hand_r_y";

		case 16: return "p1/spine_mid_x";
		case 17: return "p1/spine_mid_y";

		case 18: return "p1/spine_base_x";
		case 19: return "p1/spine_base_y";

		case 20: return "p1/hip_l_x";
		case 21: return "p1/hip_l_y";

		case 22: return "p1/knee_l_x";
		case 23: return "p1/knee_l_y";

		case 24: return "p1/foot_l_x";
		case 25: return "p1/foot_l_y";

		case 26: return "p1/hip_r_x";
		case 27: return "p1/hip_r_y";

		case 28: return "p1/knee_r_x";
		case 29: return "p1/knee_r_y";

		case 30: return "p1/foot_r_x";
		case 31: return "p1/foot_r_y";
		}
	}

	if (space == 2)
	{
		switch (index)
		{
		case 0: return "p1/head_u";
		case 1: return "p1/head_v";

		case 2: return "p1/spine_shoulder_u";
		case 3: return "p1/spine_shoulder_v";
		
		case 4: return "p1/shoulder_l_u";
		case 5: return "p1/shoulder_l_v";
		
		case 6: return "p1/elbow_l_u";
		case 7: return "p1/elbow_l_v";
		
		case 8: return "p1/hand_l_u";
		case 9: return "p1/hand_l_v";
		
		case 10: return "p1/shoulder_r_u";
		case 11: return "p1/shoulder_r_v";
		
		case 12: return "p1/elbow_r_u";
		case 13: return "p1/elbow_r_v";
		
		case 14: return "p1/hand_r_u";
		case 15: return "p1/hand_r_v";

		case 16: return "p1/spine_mid_u";
		case 17: return "p1/spine_mid_v";

		case 18: return "p1/spine_base_u";
		case 19: return "p1/spine_base_v";
		
		case 20: return "p1/hip_l_u";
		case 21: return "p1/hip_l_v";
		
		case 22: return "p1/knee_l_u";
		case 23: return "p1/knee_l_v";
		
		case 24: return "p1/foot_l_u";
		case 25: return "p1/foot_l_v";
		
		case 26: return "p1/hip_r_u";
		case 27: return "p1/hip_r_v";
		
		case 28: return "p1/knee_r_u";
		case 29: return "p1/knee_r_v";
		
		case 30: return "p1/foot_r_u";
		case 31: return "p1/foot_r_v";
		}
	}
}

void
CPlusPlusCHOPExample::execute(const CHOP_Output* output,
							  OP_Inputs* inputs, 
							  void* reserved)
{
	myExecuteCount++;

	// if inputs added - throw err message
	if (inputs->getNumInputs() > 0)
	{
		std::cout << "not compatible with input chops";
	} 
	
	
	// If no input is connected, do skeletal tracking
	else 
	{
		//enable custom parameters
		space = inputs->getParInt("Space");
		bool floorStaus = inputs->getParInt("Floor");

		//begin reading frames with openni
		openni::VideoFrameRef openniFrame;
		depthStream.readFrame(&openniFrame);

		const obt::DepthMap depthMap((const int16_t*)openniFrame.getData(),
			openniFrame.getWidth(),
			openniFrame.getHeight());
			
		//non-async API:
		auto frameResult = tracker.update(depthMap);
		std::cout << "--- Frame #" << myExecuteCount << std::endl;

		if (frameResult.is_err())
		{
			auto err = frameResult.take_error();
			std::cout << "FrameResult error: " << err.what() << std::endl;
		}

		else
		{
			auto bodyFrame = frameResult.take_value();

			//TODO ADD FLOOR PLANE AND BODY MASKS TO OUTPUT
			if (bodyFrame.floor_detected() && floorStaus == TRUE)
			{
				const auto& p = bodyFrame.floor_plane();
				
				//int numChan = output->numChannels;

				std::cout << "Floor plane: ["
					<< p.a() << ", " << p.b() << ", " << p.c() << ", " << p.d()
					<< "]" << std::endl;
			}

			//You can also access the floor mask with:
			//FloorMask floorMask = bodyFrame.floor_mask();
			
			//Body mask is also available:
			//BodyMask bodymask = bodyFrame.body_mask();

			//use to figure out how many bodies are in frame
			//auto bodyCount = bodyFrame.bodies();


			for (const auto& body : bodyFrame.bodies())
			{
				
				if (body.status() == obt::BodyStatus::NotTracking || body.status() == obt::BodyStatus::TrackingLost) {
					int chans = output->numChannels;
					
					for (int i = 0; i < chans; i++) {
						output->channels[i] = 0;
					}
					
				}
					
				if (body.status() != obt::BodyStatus::TrackingLost)
				{
					updateBody(body, output);
				}
				else
				{
					std::cout << "Body " << +body.id() << " tracking lost." << std::endl;
				}
			}
		}
	}
		
			

		/*
		inputs->enablePar("Speed", 1);
		inputs->enablePar("Reset", 1);

		double speed = inputs->getParDouble("Speed");
		double step = speed * 0.01f;


		// menu items can be evaluated as either an integer menu position, or a string
		int shape = inputs->getParInt("Shape");
		const char *shape_str = inputs->getParString("Shape");

		// keep each channel at a different phase
		double phase = 2.0f * 3.14159f / (float)(output->numChannels);

		// Notice that startIndex and the output->numSamples is used to output a smooth
		// wave by ensuring that we are outputting a value for each sample
		// Since we are outputting at 120, for each frame that has passed we'll be
		// outputing 2 samples (assuming the timeline is running at 60hz).


		for (int i = 0; i < output->numChannels; i++)
		{
			double offset = myOffset + phase*i;


			double v = 0.0f;

			switch(shape)
			{
				case 0:		// sine
					v = sin(offset);
					break;

				case 1:		// square
					v = fabs(fmod(offset, 1.0)) > 0.5;
					break;

				case 2:		// ramp	
					v = fabs(fmod(offset, 1.0));
					break;
			}


			v *= scale;

			for (int j = 0; j < output->numSamples; j++)
			{
				output->channels[i][j] = float(v);
				offset += step;
			}
		}

		myOffset += step * output->numSamples; 
	}
	*/
}

void CPlusPlusCHOPExample::updateJoint(const obt::JointList & joints, obt::JointType j1, int space, const CHOP_Output* output)
{
	const auto& joint = joints[int(j1)];

	int index = int(j1);

	if (joint.status() == obt::TrackingStatus::NotTracked)
	{
		return;
	}

	if (space == 0)
	{
		//world position
		const auto& j1 = joint.world_position();

		for (int i = 0; i < 4; i++)
		{

			switch (i)
			{
			case 0:
				for (int j = 0; j < output->numSamples; j++)
				{
					output->channels[index * 3][j] = float(j1.x);
				}

			case 1:
				for (int j = 0; j < output->numSamples; j++)
				{
					output->channels[(index * 3) + 1][j] = float(j1.y);
				}
			case 2:
				for (int j = 0; j < output->numSamples; j++)
				{
					output->channels[(index * 3) + 2][j] = float(j1.z);
				}
			}
		}

	}


	if (space == 1)
	{
		//depth position
		const auto& j1 = joint.depth_position();

		for (int i = 0; i < 3; i++)
		{

			switch (i)
			{
			case 0:
				for (int j = 0; j < output->numSamples; j++)
				{
					output->channels[index*2][j] = float(j1.x);
				}

			case 1:
				for (int j = 0; j < output->numSamples; j++)
				{
					output->channels[(index*2)+1][j] = float(j1.y);
				}
			}
		}

	}

	if (space == 2)
	{
		//UV position
		const auto& j1 = joint.depth_position();

		for (int i = 0; i < 3; i++)
		{

			switch (i)
			{
			case 0:
				for (int j = 0; j < output->numSamples; j++)
				{
					float normalized_x = j1.x / 640;
					output->channels[index * 2][j] = float(normalized_x);
				}

			case 1:
				for (int j = 0; j < output->numSamples; j++)
				{
					float normalized_y = j1.y / 480;
					output->channels[(index * 2) + 1][j] = float(normalized_y);
				}
			}
		}

	}
}

void CPlusPlusCHOPExample::updateBody(const obt::Body& body, const CHOP_Output* output)
{
	for (auto joint : body.joints())
	{
		const auto& joints = body.joints();

		updateJoint(joints, obt::JointType::Head, space, output);
		updateJoint(joints, obt::JointType::ShoulderSpine, space, output);
		updateJoint(joints, obt::JointType::LeftShoulder, space, output);
		updateJoint(joints, obt::JointType::LeftElbow, space, output);
		updateJoint(joints, obt::JointType::LeftHand, space, output);
		updateJoint(joints, obt::JointType::RightShoulder, space, output);
		updateJoint(joints, obt::JointType::RightElbow, space, output);
		updateJoint(joints, obt::JointType::RightHand, space, output);
		updateJoint(joints, obt::JointType::MidSpine, space, output);
		updateJoint(joints, obt::JointType::BaseSpine, space, output);
		updateJoint(joints, obt::JointType::LeftHip, space, output);
		updateJoint(joints, obt::JointType::LeftKnee, space, output);
		updateJoint(joints, obt::JointType::LeftFoot, space, output);
		updateJoint(joints, obt::JointType::RightHip, space, output);
		updateJoint(joints, obt::JointType::RightKnee, space, output);
		updateJoint(joints, obt::JointType::RightFoot, space, output);
	}
}

int32_t
CPlusPlusCHOPExample::getNumInfoCHOPChans()
{
	// We return the number of channel we want to output to any Info CHOP
	// connected to the CHOP. In this example we are just going to send one channel.
	return 2;
}

void
CPlusPlusCHOPExample::getInfoCHOPChan(int32_t index,
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
		chan->name = "offset";
		chan->value = (float)myOffset;
	}
}

bool		
CPlusPlusCHOPExample::getInfoDATSize(OP_InfoDATSize* infoSize)
{
	infoSize->rows = 2;
	infoSize->cols = 2;
	// Setting this to false means we'll be assigning values to the table
	// one row at a time. True means we'll do it one column at a time.
	infoSize->byColumn = false;
	return true;
}

void
CPlusPlusCHOPExample::getInfoDATEntries(int32_t index,
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
        strcpy_s(tempBuffer1, "offset");
#else // macOS
        strlcpy(tempBuffer1, "offset", sizeof(tempBuffer1));
#endif
		entries->values[0] = tempBuffer1;

		// Set the value for the second column
#ifdef WIN32
        sprintf_s(tempBuffer2, "%g", myOffset);
#else // macOS
        snprintf(tempBuffer2, sizeof(tempBuffer2), "%g", myOffset);
#endif
		entries->values[1] = tempBuffer2;
	}
}

void
CPlusPlusCHOPExample::setupParameters(OP_ParameterManager* manager)
{

	// Floor Plane
	{
		OP_NumericParameter	np;

		np.name = "Floor";
		np.label = "Floor Plane";
		
		OP_ParAppendResult res = manager->appendToggle(np);
		assert(res == OP_ParAppendResult::Success);
	}

	// Space
	{
		OP_StringParameter	sp;

		sp.name = "Space";
		sp.label = "Space";

		sp.defaultValue = "World";

		const char *names[] = { "World", "Depth", "Uv"};
		const char *labels[] = { "World", "Depth (""Projective"" in Pixels of Input Source)", "UV"};

		OP_ParAppendResult res = manager->appendMenu(sp, 3, names, labels);
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
CPlusPlusCHOPExample::pulsePressed(const char* name)
{
	if (!strcmp(name, "Reset"))
	{
		tracker.reset();
	}
}

void CPlusPlusCHOPExample::setup_ONI()
{

	std::cout << "starting setup \r\n";

	// setup functions
	Status status = STATUS_OK;
	std::cout << "Scanning machine for devices and loading modules/drivers... \r\n";

	status = OpenNI::initialize();
	std::cout << "OpenNI initialized \r\n";

	std::cout << "Opening first device ...\r\n";
	status = device.open(ANY_DEVICE);

	std::cout << device.getDeviceInfo().getName() << " Opened, Completed.\r\n";

	//error logging for unsupported devices
	/*printf("Checking if stream is supported ...\r\n");
	if (!device.hasSensor(SENSOR_DEPTH))
	{
		printf("Stream not supported by this device.\r\n");
		return;
	}*/

	//start depth stream
	printf("Asking device to create a depth stream ...\r\n");
	status = depthStream.create(device, SENSOR_DEPTH);
	if (!HandleStatus(status)) return;

	//video mode
	std::cout << "Setting video mode to 640x480x30 Depth 1MM..\r\n";
	VideoMode vmod;
	vmod.setFps(30);
	vmod.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
	vmod.setResolution(640, 480);
	status = depthStream.setVideoMode(vmod);
	if (!HandleStatus(status)) return;

	std::cout << "Starting stream..\r\n";
	status = depthStream.start();
	if (!HandleStatus(status)) return;
	
	
	char* uri = nullptr; //live sensor

	device.open(uri);

	openni::VideoStream depthStream;
	depthStream.create(device, openni::SensorType::SENSOR_DEPTH);
	
	openni::VideoMode mode;
	mode.setResolution(320, 240);
	mode.setFps(30);
	mode.setPixelFormat(openni::PixelFormat::PIXEL_FORMAT_DEPTH_1_MM);
	depthStream.setVideoMode(mode);

	depthStream.start();

	//configure path to look for data file
	const char *path = "C:/OrbbecBodyTracking.data";

	obt::BodyTrackerOptions b_t_config;
	b_t_config.set_data_file_path(path);

	auto trackerResult = obt::BodyTracker::create(b_t_config);

	if (trackerResult.is_err())
	{
		std::cout << "Could not create BodyTracker: ";
		return;
	}

	tracker = trackerResult.take_value();

}

bool 
CPlusPlusCHOPExample::HandleStatus(Status status)
{
	if (status == STATUS_OK)
		return true;
	std::cout << "ERROR: #%d, %s" << status << OpenNI::getExtendedError();
	ReadLastCharOfLine();
	return false;
}


char CPlusPlusCHOPExample::ReadLastCharOfLine()
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

