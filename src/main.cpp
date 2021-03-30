#include <iostream>
// ZED includes
#include <sl/Camera.hpp>

// PCL includes
// Undef on Win32 min/max for PCL
#ifdef _WIN32
#undef max
#undef min
#endif
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>
#include <opencv2/highgui/highgui.hpp>

// Sample includes
#include <thread>
#include <mutex>
#include <typeinfo>
#include <string>

// Namespace
using namespace sl;
using namespace std;

// Global instance (ZED, Mat, callback)
Camera zed;
Mat data_cloud;
Mat RGB_image;
std::thread zed_callback;
std::mutex mutex_input;
bool stop_signal;
bool has_data;

// Sample functions
void startZED();
void run();
void closeZED();
shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void);
inline float convertColor(float colorIn);

sl::Resolution cloud_res;
sl::Resolution rgb_res;

// set save_state. If save_state=0, it won't save.
int save_state = 0;

// Main process
int main(int argc, char **argv) {

    if (argc > 2) {
        cout << "Only the path of a SVO can be passed in arg" << endl;
        return -1;
    }

    // Set configuration parameters
    InitParameters init_params;
    if (argc == 2)
        init_params.input.setFromSVOFile(argv[1]);
    else {
        init_params.camera_resolution = RESOLUTION::HD1080;
        init_params.camera_fps = 30;
    }
    init_params.coordinate_units = UNIT::METER;
    init_params.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    init_params.depth_mode = DEPTH_MODE::ULTRA;

    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != ERROR_CODE::SUCCESS) {
        cout << toString(err) << endl;
        zed.close();
        return 1;
    }

    cloud_res = sl::Resolution(640, 360);
	rgb_res = sl::Resolution(1920, 1080);

    // Allocate PCL point cloud at the resolution
	pcl::PointCloud<pcl::PointXYZ>::Ptr p_pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    p_pcl_point_cloud->points.resize(cloud_res.area());

    // Create the PCL point cloud visualizer
    //shared_ptr<pcl::visualization::PCLVisualizer> viewer = createRGBVisualizer(p_pcl_point_cloud);
	shared_ptr<pcl::visualization::PCLVisualizer> viewer = simpleVis(p_pcl_point_cloud);

    // Start ZED callback
    startZED();

    // Set Viewer initial position
    viewer->setCameraPosition(0, 0, 5,    0, 0, 1,   0, 1, 0);
    viewer->setCameraClipDistances(0.1,1000);

	// Set RGB Viewer(not point cloud)
	cv::String win_name = "Camera Remote Control";
	cv::namedWindow(win_name);

	// Set path
	cout << "please input save Dir. (!!!Need to add '/' at end!!!)" << endl;
	string save_path;
	cin >> save_path;

	//set RGB video path
	string video_name = "RGB_video.svo";
	const char* video_save_path = NULL;
	string temp = save_path + video_name;
	video_save_path = temp.c_str();
	sl::String sl_video_save_path(video_save_path);

	//set ZED 2 video recording
	RecordingParameters recordingParameters;
	recordingParameters.compression_mode = SVO_COMPRESSION_MODE::H265;
	recordingParameters.video_filename = sl_video_save_path;
	err = zed.enableRecording(recordingParameters);

    // Loop until viewer catches the stop signal
	int frame = 0;

	// print information
	cout << "capture start." << endl;
	cout << "press R to record." << endl;
	cout << "press Q to exit." << endl;

	char key = ' ';
	//while (1){
    while (!viewer->wasStopped()) {

            //Lock to use the point cloud
            mutex_input.lock();
            float *p_data_cloud = data_cloud.getPtr<float>();
            int index = 0;
			
            // Check and adjust points for PCL format
            for (auto &it : p_pcl_point_cloud->points) {
                float X = p_data_cloud[index];
                if (!isValidMeasure(X)) // Checking if it's a valid point
					it.x = it.y = it.z = 0;
                    //it.x = it.y = it.z = it.rgb = 0;
                else {
                    it.x = X;
                    it.y = p_data_cloud[index + 1];
                    it.z = p_data_cloud[index + 2];
                    //it.rgb = convertColor(p_data_cloud[index + 3]); // Convert a 32bits float into a pcl .rgb format
                }
                index += 4;
            }

            // Unlock data and update Point cloud
            mutex_input.unlock();
            viewer->updatePointCloud(p_pcl_point_cloud);
            viewer->spinOnce(10);

			// get and show RGB_image
			cv::Mat cvImage(RGB_image.getHeight(), RGB_image.getWidth(), (RGB_image.getChannels() == 1) ? CV_8UC1 : CV_8UC4, RGB_image.getPtr<sl::uchar1>(sl::MEM::CPU));
			cv::imshow(win_name, cvImage);

			// set pcd and png save path
			auto str_frame = std::to_string(frame);
			//cout << "frame:" + str_frame << endl;
			std::string point_cloud_path = save_path + str_frame + ".pcd";
			std::string RGB_image_path = save_path + str_frame + ".png";

			// save point cloud as .pcd file
			if (save_state == 1) {
				int save;
				save = pcl::io::savePCDFile<pcl::PointXYZ>(point_cloud_path, *p_pcl_point_cloud, true);
				//cv::imwrite(RGB_image_path, cvImage);
			}		
			//int save;
			//save = pcl::io::savePCDFile<pcl::PointXYZ>(point_cloud_path, *p_pcl_point_cloud, true);

			frame++;
			key = cv::waitKey(1);
    }

    // Close the viewer
    viewer->close();

	// Close video recorder
	zed.disableRecording();

    // Close the zed
    closeZED();

    return 0;
}

/**
 *  This functions start the ZED's thread that grab images and data.
 **/
void startZED() {
    // Start the thread for grabbing ZED data
    stop_signal = false;
    has_data = false;
    zed_callback = std::thread(run);

    //Wait for data to be grabbed
    while (!has_data)
        sleep_ms(1);
}

/**
 *  This function loops to get the point cloud from the ZED. It can be considered as a callback.
 **/
void run() {
    while (!stop_signal) {
        if (zed.grab(SENSING_MODE::STANDARD) == ERROR_CODE::SUCCESS) {
		//if (true) {
            mutex_input.lock(); // To prevent from data corruption
            zed.retrieveMeasure(data_cloud, MEASURE::XYZRGBA, MEM::CPU, cloud_res);
			zed.retrieveImage(RGB_image, VIEW::LEFT, MEM::CPU, rgb_res);
            mutex_input.unlock();
            has_data = true;
        } else
            sleep_ms(1);
    }
}

/**
 *  This function frees and close the ZED, its callback(thread) and the viewer
 **/
void closeZED() {
    // Stop the thread
    stop_signal = true;
    zed_callback.join();
    zed.close();
}

/**
 *  This function creates a PCL visualizer
 **/
shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    // Open 3D viewer and add point cloud
    shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL ZED 3D Viewer"));
    viewer->setBackgroundColor(0.12, 0.12, 0.12);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());
    return (viewer);
}

shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0.12, 0.12, 0.12);
	viewer->addPointCloud<pcl::PointXYZ>(cloud);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());
	return (viewer);
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
	void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getKeySym() == "r" && event.keyDown())
	{
		std::cout << "r was pressed => start recording..." << std::endl;
		std::cout << "press Q to exit." << std::endl;

		save_state = 1;
	}
}

/**
 *  This function convert a RGBA color packed into a packed RGBA PCL compatible format
 **/
inline float convertColor(float colorIn) {
    uint32_t color_uint = *(uint32_t *) & colorIn;
    unsigned char *color_uchar = (unsigned char *) &color_uint;
    color_uint = ((uint32_t) color_uchar[0] << 16 | (uint32_t) color_uchar[1] << 8 | (uint32_t) color_uchar[2]);
    return *reinterpret_cast<float *> (&color_uint);
}
