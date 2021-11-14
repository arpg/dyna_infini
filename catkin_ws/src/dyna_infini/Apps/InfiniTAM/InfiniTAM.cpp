// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include <cstdlib>
#include <iostream>

#include "UIEngine.h"

#include "../../InputSource/OpenNIEngine.h"
#include "../../InputSource/Kinect2Engine.h"
#include "../../InputSource/LibUVCEngine.h"
#include "../../InputSource/PicoFlexxEngine.h"
#include "../../InputSource/RealSenseEngine.h"
#include "../../InputSource/LibUVCEngine.h"
#include "../../InputSource/RealSense2Engine.h"
#include "../../InputSource/FFMPEGReader.h"
#include "../../InputSource/RosImageSourceEngine.h"
#include "../../InputSource/ExternalTrackerEngine.h"
#include "../../ITMLib/ITMLibDefines.h"
#include "../../ITMLib/Core/ITMBasicEngine.h"
#include "../../ITMLib/Core/ITMBasicSurfelEngine.h"
#include "../../ITMLib/Core/ITMMultiEngine.h"


using namespace InfiniTAM::Engine;
using namespace InputSource;
using namespace ITMLib;

/** Create a default source of depth images from a list of command line
    arguments. Typically, @para arg1 would identify the calibration file to
    use, @para arg2 the colour images, @para arg3 the depth images and
    @para arg4 the IMU images. If images are omitted, some live sources will
    be tried.
*/

bool use_ros_image = true;
 
static void CreateDefaultImageSource(ImageSourceEngine* & imageSource, IMUSourceEngine* & imuSource, const char *arg1, const char *arg2, const char *arg3, const char *arg4, ros::NodeHandle& nh)
{
	const char *calibFile = arg1;
	const char *filename1 = arg2;
	const char *filename2 = arg3;
	const char *filename_imu = arg4;

	if (strcmp(calibFile, "viewer") == 0)
	{
		imageSource = new BlankImageGenerator("", Vector2i(640, 480));
		printf("starting in viewer mode: make sure to press n first to initiliase the views ... \n");
		return;
	}

	printf("using calibration file: %s\n", calibFile);

	if(filename1 == NULL && filename2 == NULL && filename_imu == NULL && use_ros_image){
		printf("Default use ROS to subsribe images..... \n");
		imageSource = new InputSource::RosImageSourceEngine(nh,calibFile);
	}
	else
		use_ros_image = false;

	if ((imageSource == NULL) && (filename2 != NULL))
	{
		printf("using rgb images: %s\nusing depth images: %s\n", filename1, filename2);
		if (filename_imu == NULL)
		{
			ImageMaskPathGenerator pathGenerator(filename1, filename2);
			imageSource = new ImageFileReader<ImageMaskPathGenerator>(calibFile, pathGenerator);
		}
		else
		{
			printf("using imu data: %s\n", filename_imu);
			imageSource = new RawFileReader(calibFile, filename1, filename2, Vector2i(320, 240), 0.5f);
			imuSource = new IMUSourceEngine(filename_imu);
		}

		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			if (imuSource != NULL) delete imuSource;
			imuSource = NULL;
			imageSource = NULL;
		}
	}

	if ((imageSource == NULL) && (filename1 != NULL) && (filename_imu == NULL))
	{
		imageSource = new InputSource::FFMPEGReader(calibFile, filename1, filename2);
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}

	if (imageSource == NULL)
	{
		// If no calibration file specified, use the factory default calibration
		bool useInternalCalibration = !calibFile || strlen(calibFile) == 0;

		printf("trying OpenNI device: %s - calibration: %s\n",
				filename1 ? filename1 : "<OpenNI default device>",
				useInternalCalibration ? "internal" : "from file");
		//imageSource = new OpenNIEngine(calibFile, filename1, useInternalCalibration);
        // Set useInternalCalibration to true if ITMVoxel::hasColorInformation is true
        imageSource = new OpenNIEngine(calibFile, filename1, ITMVoxel::hasColorInformation);
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}

	if (imageSource == NULL)
	{
		printf("trying UVC device\n");
		imageSource = new LibUVCEngine(calibFile);
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}

	if (imageSource == NULL)
	{
		printf("trying RealSense device\n");
		imageSource = new RealSenseEngine(calibFile);
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}

    if (imageSource == NULL)
    {
        printf("trying RealSense device with SDK 2.X (librealsense2)\n");
        imageSource = new RealSense2Engine(calibFile);
        if (imageSource->getDepthImageSize().x == 0)
        {
            delete imageSource;
            imageSource = NULL;
        }
    }

    if (imageSource == NULL)
	{
		printf("trying MS Kinect 2 device\n");
		imageSource = new Kinect2Engine(calibFile);
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}

	if (imageSource == NULL)
	{
		printf("trying PMD PicoFlexx device\n");
		imageSource = new PicoFlexxEngine(calibFile);
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}
}

int main(int argc, char** argv)
try
{
	const char *arg1 = "";
	const char *arg2 = NULL;
	const char *arg3 = NULL;
	const char *arg4 = NULL;

	ros::init(argc,argv, "Infinitam_node");
	ros::NodeHandle nh;

	std::string calib_address;
	if(nh.getParam("calib_address", calib_address)){
		ROS_INFO("Calibration file address %s \n", calib_address.c_str());
		arg1 = calib_address.c_str();
	}
	else{
		ROS_INFO("Not providing calibration file address in the launch file. Make sure you provide 4 args to run the Infinitam without ROS %s \n", calib_address.c_str());
	}

	bool use_external_pose_estimation = false;
	if(nh.getParam("use_external_pose_estimation", use_external_pose_estimation)){
		ROS_INFO("Use_external_pose_estimation or not ? %d \n", use_external_pose_estimation);
	}
	else{
		ROS_INFO("Not providing if we need to use external pose estimation. Default not use.  \n");
	}

	bool dynamic_objects = false;
	if(nh.getParam("dynamic_objects", dynamic_objects)){
		ROS_INFO("has dynamic_objects or not ? %d \n", dynamic_objects);
	}
	else{
		ROS_INFO("Not providing if we need to consider dynamic objects. Default not.  \n");
	}

	bool mask_out = false;
	if(nh.getParam("mask_out", mask_out)){
		ROS_INFO("reconstruct the dynamic object ? %d \n", !mask_out);
	}
	else{
		ROS_INFO("Not providing if we need to reconstruct the dynamic object, default we reconstruct everything. \n");
	}

	// int arg = 1;
	// do {
	// 	if (argv[arg] != NULL) arg1 = argv[arg]; else break;
	// 	++arg;
	// 	if (argv[arg] != NULL) arg2 = argv[arg]; else break;
	// 	++arg;
	// 	if (argv[arg] != NULL) arg3 = argv[arg]; else break;
	// 	++arg;
	// 	if (argv[arg] != NULL) arg4 = argv[arg]; else break;
	// } while (false);

	// if (arg == 1) {
	// 	printf("usage: %s [<calibfile> [<imagesource>] ]\n"
	// 	       "  <calibfile>   : path to a file containing intrinsic calibration parameters\n"
	// 	       "  <imagesource> : either one argument to specify OpenNI device ID\n"
	// 	       "                  or two arguments specifying rgb and depth file masks\n"
	// 	       "\n"
	// 	       "examples:\n"
	// 	       "  %s ./Files/Teddy/calib.txt ./Files/Teddy/Frames/%%04i.ppm ./Files/Teddy/Frames/%%04i.pgm\n"
	// 	       "  %s ./Files/Teddy/calib.txt\n\n", argv[0], argv[0], argv[0]);
	// }

	printf("initialising ...\n");
	ImageSourceEngine *imageSource = NULL;
	IMUSourceEngine *imuSource = NULL;
	ExternalTrackerEngine *ExternalTrackerSource = NULL;

	CreateDefaultImageSource(imageSource, imuSource, arg1, arg2, arg3, arg4, nh);

	if(use_external_pose_estimation){
		ExternalTrackerSource = new ExternalTrackerEngine(nh);
		//generally you dont want to start this spin because in the ROSImageSourceEngine we have started the ROS Spin
		//However, if you don want to subscribe ROS image but just want to use external pose from ROS, we need to start ros spin here
		//If you just want to use external pose from ROS, you need to MAKE SURE your none-ROS image have "timestamp" as a member so that the program can compare if the external pose matches that image
		if(!use_ros_image)
			ExternalTrackerSource->StartRosSpinThread();
	}

	if (imageSource==NULL)
	{
		std::cout << "failed to open any image stream" << std::endl;
		return -1;
	}

	ITMLibSettings *internalSettings = new ITMLibSettings();
	internalSettings->mask_out_ = mask_out;

	ITMMainEngine *mainEngine = NULL;
	switch (internalSettings->libMode)
	{
	case ITMLibSettings::LIBMODE_BASIC:
		mainEngine = new ITMBasicEngine<ITMVoxel, ITMVoxelIndex>(internalSettings, imageSource->getCalib(), imageSource->getRGBImageSize(), imageSource->getDepthImageSize());
		break;
	case ITMLibSettings::LIBMODE_BASIC_SURFELS:
		mainEngine = new ITMBasicSurfelEngine<ITMSurfelT>(internalSettings, imageSource->getCalib(), imageSource->getRGBImageSize(), imageSource->getDepthImageSize());
		break;
	case ITMLibSettings::LIBMODE_LOOPCLOSURE:
		mainEngine = new ITMMultiEngine<ITMVoxel, ITMVoxelIndex>(internalSettings, imageSource->getCalib(), imageSource->getRGBImageSize(), imageSource->getDepthImageSize());
		break;
	default: 
		throw std::runtime_error("Unsupported library mode!");
		break;
	}

	UIEngine::Instance()->Initialise(argc, argv, imageSource, imuSource, ExternalTrackerSource, mainEngine, "./Files/Out", internalSettings->deviceType, use_ros_image, dynamic_objects);
	UIEngine::Instance()->Run();
	UIEngine::Instance()->Shutdown();

	delete mainEngine;
	delete internalSettings;
	delete imageSource;
	delete ExternalTrackerSource;
	if (imuSource != NULL) delete imuSource;
	return 0;
}
catch(std::exception& e)
{
	std::cerr << e.what() << '\n';
	return EXIT_FAILURE;
}

