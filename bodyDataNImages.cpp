#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <k4a/k4a.h>
#include <k4abt.h>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>
#include <sys/time.h>
#include <math.h>
#include <vector>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

template<typename T>
inline void ConvertToGrayScaleImage(const T* imgDat, const int size, const int vmin, const int vmax, uint8_t* img)
{
    for (int i = 0; i < size; i++)
    {
        T v = imgDat[i];
        float colorValue = 0.0f;
        if (v <= vmin)
        {
            colorValue = 0.0f;
        }
        else if (v >= vmax)
        {
            colorValue = 1.0f;
        }
        else
        {
            colorValue = (float)(v - vmin) / (float)(vmax - vmin);
        }
        img[i] = (uint8_t)(colorValue * 255);
    }
}

#define PI 3.141592654

#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }                                                                                                    \

double Pitch(k4a_quaternion_t quaternion);
double Yaw(k4a_quaternion_t quaternion);
double Roll(k4a_quaternion_t quaternion);

int main()
{
    k4a_device_t device = NULL;
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_2160P;
    deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

    k4a_calibration_t sensor_calibration;
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensor_calibration),
        "Get depth camera calibration failed!");

    k4abt_tracker_t tracker = NULL;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    VERIFY(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker), "Body tracker initialization failed!");

    int frame_count = 0;
    do
    {
        k4a_capture_t sensor_capture;
        k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);
        if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            frame_count++;
            k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);
            k4a_capture_release(sensor_capture); // Remember to release the sensor capture once you finish using it
            if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
            {
                // It should never hit timeout when K4A_WAIT_INFINITE is set.
                printf("Error! Add capture to tracker process queue timeout!\n");
                break;
            }
            else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
            {
                printf("Error! Add capture to tracker process queue failed!\n");
                break;
            }

            k4abt_frame_t body_frame = NULL;
            k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
            FILE *fp;
	    if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
            {
                // Successfully popped the body tracking result. Start your processing
                size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
                printf("%zu bodies are detected!\n", num_bodies);
                k4a_capture_t input_capture = k4abt_frame_get_capture(body_frame);
                // Access the color image
                k4a_image_t colorImage;
                k4a_image_t depthImage;
                k4a_image_t irImage;
                switch (k4a_device_get_capture(device, &input_capture,1000))
                {
                case K4A_WAIT_RESULT_SUCCEEDED:
                    break;
                case K4A_WAIT_RESULT_TIMEOUT:
                    printf("Timed out waiting for a capture\n");
                    continue;
                    break;
                case K4A_WAIT_RESULT_FAILED:
                    printf("Failed to read a capture\n");
                    continue;
                    break;
                }
                // Probe for a color image
                colorImage = k4a_capture_get_color_image(input_capture);
                if (colorImage)
                {
                    printf(" | Color res:%4dx%4d stride:%5d ",
                           k4a_image_get_height_pixels(colorImage),
                           k4a_image_get_width_pixels(colorImage),
                           k4a_image_get_stride_bytes(colorImage));

                    // you can check the format with this function
                    k4a_image_format_t format = k4a_image_get_format(colorImage); // K4A_IMAGE_FORMAT_COLOR_BGRA32 

                    // get raw buffer
                    uint8_t* buffer = k4a_image_get_buffer(colorImage);

                    // convert the raw buffer to cv::Mat
                    int rows = k4a_image_get_height_pixels(colorImage);
                    int cols = k4a_image_get_width_pixels(colorImage);
                    cv::Mat colorMat(rows , cols, CV_8UC4, (void*)buffer, cv::Mat::AUTO_STEP);
                    cv::namedWindow("Color");
                    cv::imshow("Color", colorMat);
                    cv::waitKey(500);
                    cv::destroyWindow("Color");
                    k4a_image_release(colorImage);
                }
                else
                {
                    printf(" | Color None                       ");
                }

                // probe for a IR16 image
                irImage = k4a_capture_get_ir_image(input_capture);
                if (irImage != NULL)
                {
                    printf(" | Ir16 res:%4dx%4d stride:%5d ",
                        k4a_image_get_height_pixels(irImage),
                        k4a_image_get_width_pixels(irImage),
                        k4a_image_get_stride_bytes(irImage));
                    k4a_image_format_t format = k4a_image_get_format(irImage); // K4A_IMAGE_FORMAT_DEPTH16 
                    int height = k4a_image_get_height_pixels(irImage);
                    int width = k4a_image_get_width_pixels(irImage);
                    int strides = k4a_image_get_stride_bytes(irImage);
                    printf(" height: %d , %d ", height, width);
                    printf("stride: %d", strides);

                    // One way to convert 16bit to 8bit with user specified min/max dynamic range
                    uint8_t* imgData = k4a_image_get_buffer(irImage);
                    uint16_t* irImg = reinterpret_cast<uint16_t*>(imgData);
                    std::vector<uint8_t> grayScaleImg(width * height);
                    int irMinValue = 0;
                    int irMaxValue = 1000;
                    ConvertToGrayScaleImage(irImg, width * height, irMinValue, irMaxValue, grayScaleImg.data());
                    const cv::Mat irMat(cv::Size(width, height), CV_8U, grayScaleImg.data());
                    cv::namedWindow("IR");
                    cv::imshow("IR", irMat);
                    cv::waitKey(500);
                    cv::destroyWindow("IR");
                    k4a_image_release(irImage);

                }
                else
                {
                    printf(" | Ir16 None                       ");
                }

                // Probe for a depth16 image
                depthImage = k4a_capture_get_depth_image(input_capture);
                if (depthImage != NULL)
                {
                    printf(" | Depth16 res:%4dx%4d stride:%5d\n",
                        k4a_image_get_height_pixels(depthImage),
                        k4a_image_get_width_pixels(depthImage),
                        k4a_image_get_stride_bytes(depthImage));
                        // you can check the format with this function
                    k4a_image_format_t format = k4a_image_get_format(depthImage); // K4A_IMAGE_FORMAT_DEPTH16 

                    // get raw buffer
                    uint8_t* buffer = k4a_image_get_buffer(depthImage);

                    // convert the raw buffer to cv::Mat
                    int rows = k4a_image_get_height_pixels(depthImage);
                    int cols = k4a_image_get_width_pixels(depthImage);
                    cv::Mat depthMat(rows, cols, CV_16U, (void*)buffer, cv::Mat::AUTO_STEP);
                    cv::namedWindow("Depth");
                    cv::imshow("Depth", depthMat);
                    cv::waitKey(500);
                    cv::destroyWindow("Depth");
                    k4a_image_release(depthImage);
                }
                else
                {
                    printf(" | Depth16 None\n");
                }

                k4a_capture_release(input_capture);                
                k4a_image_t body_index_map = k4abt_frame_get_body_index_map(body_frame);
                // Do your work with the body index map
                k4a_image_release(body_index_map);


                for (size_t i = 0; i < num_bodies; i++)
                {
                    k4abt_skeleton_t skeleton;
                    k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
                    uint32_t id = k4abt_frame_get_body_id(body_frame, i);
                    k4a_float3_t pelvis = skeleton.joints[K4ABT_JOINT_PELVIS].position;
                    k4a_float3_t naval = skeleton.joints[K4ABT_JOINT_SPINE_NAVAL].position;
                    k4a_float3_t chest = skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position;
                    k4a_float3_t neck = skeleton.joints[K4ABT_JOINT_NECK].position;
                    k4a_float3_t clavicle_left = skeleton.joints[K4ABT_JOINT_CLAVICLE_LEFT].position;
                    k4a_float3_t shoulder_left = skeleton.joints[K4ABT_JOINT_SHOULDER_LEFT].position;
                    k4a_float3_t elbow_left = skeleton.joints[K4ABT_JOINT_ELBOW_LEFT].position;
                    k4a_float3_t wrist_left = skeleton.joints[K4ABT_JOINT_WRIST_LEFT].position;
                    k4a_float3_t hand_left = skeleton.joints[K4ABT_JOINT_HAND_LEFT].position;
                    k4a_float3_t handtip_left = skeleton.joints[K4ABT_JOINT_HANDTIP_LEFT].position;
                    k4a_float3_t thumb_left = skeleton.joints[K4ABT_JOINT_THUMB_LEFT].position;
                    k4a_float3_t clavicle_right = skeleton.joints[K4ABT_JOINT_CLAVICLE_RIGHT].position;
                    k4a_float3_t shoulder_right = skeleton.joints[K4ABT_JOINT_SHOULDER_RIGHT].position;
                    k4a_float3_t elbow_right  = skeleton.joints[K4ABT_JOINT_ELBOW_RIGHT].position;
                    k4a_float3_t wrist_right = skeleton.joints[K4ABT_JOINT_WRIST_RIGHT].position;
                    k4a_float3_t hand_right = skeleton.joints[K4ABT_JOINT_HAND_RIGHT].position;
                    k4a_float3_t handtip_right = skeleton.joints[K4ABT_JOINT_HANDTIP_RIGHT].position;
                    k4a_float3_t thumb_right = skeleton.joints[K4ABT_JOINT_THUMB_RIGHT].position;
                    k4a_float3_t hip_left = skeleton.joints[K4ABT_JOINT_HIP_LEFT].position;
                    k4a_float3_t knee_left = skeleton.joints[K4ABT_JOINT_KNEE_LEFT].position;
                    k4a_float3_t ankle_left = skeleton.joints[K4ABT_JOINT_ANKLE_LEFT].position;
                    k4a_float3_t foot_left = skeleton.joints[K4ABT_JOINT_FOOT_LEFT].position;
                    k4a_float3_t hip_right = skeleton.joints[K4ABT_JOINT_HIP_RIGHT].position;
                    k4a_float3_t knee_right = skeleton.joints[K4ABT_JOINT_KNEE_RIGHT].position;
                    k4a_float3_t ankle_right = skeleton.joints[K4ABT_JOINT_ANKLE_RIGHT].position;
                    k4a_float3_t foot_right = skeleton.joints[K4ABT_JOINT_FOOT_RIGHT].position;
                    k4a_float3_t head = skeleton.joints[K4ABT_JOINT_HEAD].position;
                    k4a_float3_t nose = skeleton.joints[K4ABT_JOINT_NOSE].position;
                    k4a_float3_t eye_left = skeleton.joints[K4ABT_JOINT_EYE_LEFT].position;
                    k4a_float3_t ear_left = skeleton.joints[K4ABT_JOINT_EAR_LEFT].position;
                    k4a_float3_t eye_right = skeleton.joints[K4ABT_JOINT_EYE_RIGHT].position;
                    k4a_float3_t ear_right = skeleton.joints[K4ABT_JOINT_EAR_RIGHT].position;

                    k4a_quaternion_t pelvis_orientation = skeleton.joints[K4ABT_JOINT_PELVIS].orientation;
                    k4a_quaternion_t naval_orientation = skeleton.joints[K4ABT_JOINT_SPINE_NAVAL].orientation;
                    k4a_quaternion_t chest_orientation = skeleton.joints[K4ABT_JOINT_SPINE_CHEST].orientation;
                    k4a_quaternion_t neck_orientation = skeleton.joints[K4ABT_JOINT_NECK].orientation;
                    k4a_quaternion_t clavicle_left_orientation = skeleton.joints[K4ABT_JOINT_CLAVICLE_LEFT].orientation;
                    k4a_quaternion_t shoulder_left_orientation = skeleton.joints[K4ABT_JOINT_SHOULDER_LEFT].orientation;
                    k4a_quaternion_t elbow_left_orientation = skeleton.joints[K4ABT_JOINT_ELBOW_LEFT].orientation;
                    k4a_quaternion_t wrist_left_orientation = skeleton.joints[K4ABT_JOINT_WRIST_LEFT].orientation;
                    k4a_quaternion_t hand_left_orientation = skeleton.joints[K4ABT_JOINT_HAND_LEFT].orientation;
                    k4a_quaternion_t handtip_left_orientation = skeleton.joints[K4ABT_JOINT_HANDTIP_LEFT].orientation;
                    k4a_quaternion_t thumb_left_orientation = skeleton.joints[K4ABT_JOINT_THUMB_LEFT].orientation;
                    k4a_quaternion_t clavicle_right_orientation = skeleton.joints[K4ABT_JOINT_CLAVICLE_RIGHT].orientation;
                    k4a_quaternion_t shoulder_right_orientation = skeleton.joints[K4ABT_JOINT_SHOULDER_RIGHT].orientation;
                    k4a_quaternion_t elbow_right_orientation = skeleton.joints[K4ABT_JOINT_ELBOW_RIGHT].orientation;
                    k4a_quaternion_t wrist_right_orientation = skeleton.joints[K4ABT_JOINT_WRIST_RIGHT].orientation;
                    k4a_quaternion_t hand_right_orientation = skeleton.joints[K4ABT_JOINT_HAND_RIGHT].orientation;
                    k4a_quaternion_t handtip_right_orientation = skeleton.joints[K4ABT_JOINT_HANDTIP_RIGHT].orientation;
                    k4a_quaternion_t thumb_right_orientation = skeleton.joints[K4ABT_JOINT_THUMB_RIGHT].orientation;
                    k4a_quaternion_t hip_left_orientation = skeleton.joints[K4ABT_JOINT_HIP_LEFT].orientation;
                    k4a_quaternion_t knee_left_orientation = skeleton.joints[K4ABT_JOINT_KNEE_LEFT].orientation;
                    k4a_quaternion_t ankle_left_orientation = skeleton.joints[K4ABT_JOINT_ANKLE_LEFT].orientation;
                    k4a_quaternion_t foot_left_orientation = skeleton.joints[K4ABT_JOINT_FOOT_LEFT].orientation;
                    k4a_quaternion_t hip_right_orientation = skeleton.joints[K4ABT_JOINT_HIP_RIGHT].orientation;
                    k4a_quaternion_t knee_right_orientation = skeleton.joints[K4ABT_JOINT_KNEE_RIGHT].orientation;
                    k4a_quaternion_t ankle_right_orientation = skeleton.joints[K4ABT_JOINT_ANKLE_RIGHT].orientation;
                    k4a_quaternion_t foot_right_orientation = skeleton.joints[K4ABT_JOINT_FOOT_RIGHT].orientation;
                    k4a_quaternion_t head_orientation = skeleton.joints[K4ABT_JOINT_HEAD].orientation;
                    k4a_quaternion_t nose_orientation = skeleton.joints[K4ABT_JOINT_NOSE].orientation;
                    k4a_quaternion_t eye_left_orientation = skeleton.joints[K4ABT_JOINT_EYE_LEFT].orientation;
                    k4a_quaternion_t ear_left_orientation = skeleton.joints[K4ABT_JOINT_EAR_LEFT].orientation;
                    k4a_quaternion_t eye_right_orientation = skeleton.joints[K4ABT_JOINT_EYE_RIGHT].orientation;
                    k4a_quaternion_t ear_right_orientation = skeleton.joints[K4ABT_JOINT_EAR_RIGHT].orientation;



		    fp=fopen("./bodyData.csv", "a+");
		    time_t t = time(NULL);
  		    struct tm tm = *localtime(&t);
            struct timeval currentTime;
            gettimeofday(&currentTime, NULL);
            long micro = currentTime.tv_sec * (int)1e6 + currentTime.tv_usec;

  		    fprintf(fp,"\n%d-%02d-%02d_%02d-%02d-%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
            fprintf(fp,",%03u", micro * 1000);
                fprintf(fp,",%zu", id);
                    fprintf(fp,",%lf", pelvis.xyz.x / pelvis_orientation.wxyz.w);
                    fprintf(fp,",%lf", pelvis.xyz.y / pelvis_orientation.wxyz.w);
                    fprintf(fp,",%lf", pelvis.xyz.z / pelvis_orientation.wxyz.w);
                    fprintf(fp,",%lf", Pitch(pelvis_orientation));
                    fprintf(fp,",%lf", Yaw(pelvis_orientation));
                    fprintf(fp,",%lf", Roll(pelvis_orientation));
                    fprintf(fp,",%lf", naval.xyz.x);
                    fprintf(fp,",%lf", naval.xyz.y);
                    fprintf(fp,",%lf", naval.xyz.z);
                    fprintf(fp,",%lf", Pitch(naval_orientation));
                    fprintf(fp,",%lf", Yaw(naval_orientation));
                    fprintf(fp,",%lf", Roll(naval_orientation));
                    fprintf(fp,",%lf", chest.xyz.x);
                    fprintf(fp,",%lf", chest.xyz.y);
                    fprintf(fp,",%lf", chest.xyz.z);
                    fprintf(fp,",%lf", Pitch(chest_orientation));
                    fprintf(fp,",%lf", Yaw(chest_orientation));
                    fprintf(fp,",%lf", Roll(chest_orientation));
                    fprintf(fp,",%lf", neck.xyz.x);
                    fprintf(fp,",%lf", neck.xyz.y);
                    fprintf(fp,",%lf", neck.xyz.z);
                    fprintf(fp,",%lf", Pitch(neck_orientation));
                    fprintf(fp,",%lf", Yaw(neck_orientation));
                    fprintf(fp,",%lf", Roll(neck_orientation));
                    fprintf(fp,",%lf", clavicle_left.xyz.x);
                    fprintf(fp,",%lf", clavicle_left.xyz.y);
                    fprintf(fp,",%lf", clavicle_left.xyz.z);
                    fprintf(fp,",%lf", Pitch(clavicle_left_orientation));
                    fprintf(fp,",%lf", Yaw(clavicle_left_orientation));
                    fprintf(fp,",%lf", Roll(clavicle_left_orientation));
                    fprintf(fp,",%lf", shoulder_left.xyz.x);
                    fprintf(fp,",%lf", shoulder_left.xyz.y);
                    fprintf(fp,",%lf", shoulder_left.xyz.z);
                    fprintf(fp,",%lf", Pitch(shoulder_left_orientation));
                    fprintf(fp,",%lf", Yaw(shoulder_left_orientation));
                    fprintf(fp,",%lf", Roll(shoulder_left_orientation));
                    fprintf(fp,",%lf", elbow_left.xyz.x);
                    fprintf(fp,",%lf", elbow_left.xyz.y);
                    fprintf(fp,",%lf", elbow_left.xyz.z);
                    fprintf(fp,",%lf", Pitch(elbow_left_orientation));
                    fprintf(fp,",%lf", Yaw(elbow_left_orientation));
                    fprintf(fp,",%lf", Roll(elbow_left_orientation));
                    fprintf(fp,",%lf", wrist_left.xyz.x);
                    fprintf(fp,",%lf", wrist_left.xyz.y);
                    fprintf(fp,",%lf", wrist_left.xyz.z);
                    fprintf(fp,",%lf", Pitch(wrist_left_orientation));
                    fprintf(fp,",%lf", Yaw(wrist_left_orientation));
                    fprintf(fp,",%lf", Roll(wrist_left_orientation));
                    fprintf(fp,",%lf", hand_left.xyz.x);
                    fprintf(fp,",%lf", hand_left.xyz.y);
                    fprintf(fp,",%lf", hand_left.xyz.z);
                    fprintf(fp,",%lf", Pitch(hand_left_orientation));
                    fprintf(fp,",%lf", Yaw(hand_left_orientation));
                    fprintf(fp,",%lf", Roll(hand_left_orientation));
                    fprintf(fp,",%lf", handtip_left.xyz.x);
                    fprintf(fp,",%lf", handtip_left.xyz.y);
                    fprintf(fp,",%lf", handtip_left.xyz.z);
                    fprintf(fp,",%lf", Pitch(handtip_left_orientation));
                    fprintf(fp,",%lf", Yaw(handtip_left_orientation));
                    fprintf(fp,",%lf", Roll(handtip_left_orientation));
                    fprintf(fp,",%lf", thumb_left.xyz.x);
                    fprintf(fp,",%lf", thumb_left.xyz.y);
                    fprintf(fp,",%lf", thumb_left.xyz.z);
                    fprintf(fp,",%lf", Pitch(thumb_left_orientation));
                    fprintf(fp,",%lf", Yaw(thumb_left_orientation));
                    fprintf(fp,",%lf", Roll(thumb_left_orientation));
                    fprintf(fp,",%lf", clavicle_right.xyz.x);
                    fprintf(fp,",%lf", clavicle_right.xyz.y);
                    fprintf(fp,",%lf", clavicle_right.xyz.z);
                    fprintf(fp,",%lf", Pitch(clavicle_right_orientation));
                    fprintf(fp,",%lf", Yaw(clavicle_right_orientation));
                    fprintf(fp,",%lf", Roll(clavicle_right_orientation));
                    fprintf(fp,",%lf", shoulder_right.xyz.x);
                    fprintf(fp,",%lf", shoulder_right.xyz.y);
                    fprintf(fp,",%lf", shoulder_right.xyz.z);
                    fprintf(fp,",%lf", Pitch(shoulder_right_orientation));
                    fprintf(fp,",%lf", Yaw(shoulder_right_orientation));
                    fprintf(fp,",%lf", Roll(shoulder_right_orientation));
                    fprintf(fp,",%lf", elbow_right.xyz.x);
                    fprintf(fp,",%lf", elbow_right.xyz.y);
                    fprintf(fp,",%lf", elbow_right.xyz.z);
                    fprintf(fp,",%lf", Pitch(elbow_right_orientation));
                    fprintf(fp,",%lf", Yaw(elbow_right_orientation));
                    fprintf(fp,",%lf", Roll(elbow_right_orientation));
                    fprintf(fp,",%lf", wrist_right.xyz.x);
                    fprintf(fp,",%lf", wrist_right.xyz.y);
                    fprintf(fp,",%lf", wrist_right.xyz.z);
                    fprintf(fp,",%lf", Pitch(wrist_right_orientation));
                    fprintf(fp,",%lf", Yaw(wrist_right_orientation));
                    fprintf(fp,",%lf", Roll(wrist_right_orientation));
                    fprintf(fp,",%lf", hand_right.xyz.x);
                    fprintf(fp,",%lf", hand_right.xyz.y);
                    fprintf(fp,",%lf", hand_right.xyz.z);
                    fprintf(fp,",%lf", Pitch(hand_right_orientation));
                    fprintf(fp,",%lf", Yaw(hand_right_orientation));
                    fprintf(fp,",%lf", Roll(hand_right_orientation));
                    fprintf(fp,",%lf", handtip_right.xyz.x);
                    fprintf(fp,",%lf", handtip_right.xyz.y);
                    fprintf(fp,",%lf", handtip_right.xyz.z);
                    fprintf(fp,",%lf", Pitch(handtip_right_orientation));
                    fprintf(fp,",%lf", Yaw(handtip_right_orientation));
                    fprintf(fp,",%lf", Roll(handtip_right_orientation));
                    fprintf(fp,",%lf", thumb_right.xyz.x);
                    fprintf(fp,",%lf", thumb_right.xyz.y);
                    fprintf(fp,",%lf", thumb_right.xyz.z);
                    fprintf(fp,",%lf", Pitch(thumb_right_orientation));
                    fprintf(fp,",%lf", Yaw(thumb_right_orientation));
                    fprintf(fp,",%lf", Roll(thumb_right_orientation));
                    fprintf(fp,",%lf", hip_left.xyz.x);
                    fprintf(fp,",%lf", hip_left.xyz.y);
                    fprintf(fp,",%lf", hip_left.xyz.z);
                    fprintf(fp,",%lf", Pitch(hip_left_orientation));
                    fprintf(fp,",%lf", Yaw(hip_left_orientation));
                    fprintf(fp,",%lf", Roll(hip_left_orientation));
                    fprintf(fp,",%lf", knee_left.xyz.x);
                    fprintf(fp,",%lf", knee_left.xyz.y);
                    fprintf(fp,",%lf", knee_left.xyz.z);
                    fprintf(fp,",%lf", Pitch(knee_left_orientation));
                    fprintf(fp,",%lf", Yaw(knee_left_orientation));
                    fprintf(fp,",%lf", Roll(knee_left_orientation));
                    fprintf(fp,",%lf", ankle_left.xyz.x);
                    fprintf(fp,",%lf", ankle_left.xyz.y);
                    fprintf(fp,",%lf", ankle_left.xyz.z);
                    fprintf(fp,",%lf", Pitch(ankle_left_orientation));
                    fprintf(fp,",%lf", Yaw(ankle_left_orientation));
                    fprintf(fp,",%lf", Roll(ankle_left_orientation));
                    fprintf(fp,",%lf", foot_left.xyz.x);
                    fprintf(fp,",%lf", foot_left.xyz.y);
                    fprintf(fp,",%lf", foot_left.xyz.z);
                    fprintf(fp,",%lf", Pitch(foot_left_orientation));
                    fprintf(fp,",%lf", Yaw(foot_left_orientation));
                    fprintf(fp,",%lf", Roll(foot_left_orientation));
                    fprintf(fp,",%lf", hip_right.xyz.x);
                    fprintf(fp,",%lf", hip_right.xyz.y);
                    fprintf(fp,",%lf", hip_right.xyz.z);
                    fprintf(fp,",%lf", Pitch(hip_right_orientation));
                    fprintf(fp,",%lf", Yaw(hip_right_orientation));
                    fprintf(fp,",%lf", Roll(hip_right_orientation));
                    fprintf(fp,",%lf", knee_right.xyz.x);
                    fprintf(fp,",%lf", knee_right.xyz.y);
                    fprintf(fp,",%lf", knee_right.xyz.z);
                    fprintf(fp,",%lf", Pitch(knee_right_orientation));
                    fprintf(fp,",%lf", Yaw(knee_right_orientation));
                    fprintf(fp,",%lf", Roll(knee_right_orientation));
                    fprintf(fp,",%lf", ankle_right.xyz.x);
                    fprintf(fp,",%lf", ankle_right.xyz.y);
                    fprintf(fp,",%lf", ankle_right.xyz.z);
                    fprintf(fp,",%lf", Pitch(ankle_right_orientation));
                    fprintf(fp,",%lf", Yaw(ankle_right_orientation));
                    fprintf(fp,",%lf", Roll(ankle_right_orientation));
                    fprintf(fp,",%lf", foot_right.xyz.x);
                    fprintf(fp,",%lf", foot_right.xyz.y);
                    fprintf(fp,",%lf", foot_right.xyz.z);
                    fprintf(fp,",%lf", Pitch(foot_right_orientation));
                    fprintf(fp,",%lf", Yaw(foot_right_orientation));
                    fprintf(fp,",%lf", Roll(foot_right_orientation));
                    fprintf(fp,",%lf", head.xyz.x);
                    fprintf(fp,",%lf", head.xyz.y);
                    fprintf(fp,",%lf", head.xyz.z);
                    fprintf(fp,",%lf", Pitch(head_orientation));
                    fprintf(fp,",%lf", Yaw(head_orientation));
                    fprintf(fp,",%lf", Roll(head_orientation));
                    fprintf(fp,",%lf", nose.xyz.x);
                    fprintf(fp,",%lf", nose.xyz.y);
                    fprintf(fp,",%lf", nose.xyz.z);
                    fprintf(fp,",%lf", Pitch(nose_orientation));
                    fprintf(fp,",%lf", Yaw(nose_orientation));
                    fprintf(fp,",%lf", Roll(nose_orientation));
                    fprintf(fp,",%lf", eye_left.xyz.x);
                    fprintf(fp,",%lf", eye_left.xyz.y);
                    fprintf(fp,",%lf", eye_left.xyz.z);
                    fprintf(fp,",%lf", Pitch(eye_left_orientation));
                    fprintf(fp,",%lf", Yaw(eye_left_orientation));
                    fprintf(fp,",%lf", Roll(eye_left_orientation));
                    fprintf(fp,",%lf", ear_left.xyz.x);
                    fprintf(fp,",%lf", ear_left.xyz.y);
                    fprintf(fp,",%lf", ear_left.xyz.z);
                    fprintf(fp,",%lf", Pitch(ear_left_orientation));
                    fprintf(fp,",%lf", Yaw(ear_left_orientation));
                    fprintf(fp,",%lf", Roll(ear_left_orientation));
                    fprintf(fp,",%lf", eye_right.xyz.x);
                    fprintf(fp,",%lf", eye_right.xyz.y);
                    fprintf(fp,",%lf", eye_right.xyz.z);
                    fprintf(fp,",%lf", Pitch(eye_right_orientation));
                    fprintf(fp,",%lf", Yaw(eye_right_orientation));
                    fprintf(fp,",%lf", Roll(eye_right_orientation));
                    fprintf(fp,",%lf", ear_right.xyz.x);
                    fprintf(fp,",%lf", ear_right.xyz.y);
                    fprintf(fp,",%lf", ear_right.xyz.z);
                    fprintf(fp,",%lf", Pitch(ear_right_orientation));
                    fprintf(fp,",%lf", Yaw(ear_right_orientation));
                    fprintf(fp,",%lf", Roll(ear_right_orientation));                    
		    fclose(fp);
                }

                k4abt_frame_release(body_frame); // Remember to release the body frame once you finish using it
            }
            else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
            {
                //  It should never hit timeout when K4A_WAIT_INFINITE is set.
                printf("Error! Pop body frame result timeout!\n");
                break;
            }
            else
            {
                printf("Pop body frame result failed!\n");
                break;
            }
        }
        else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT)
        {
            // It should never hit time out when K4A_WAIT_INFINITE is set.
            printf("Error! Get depth frame time out!\n");
            break;
        }
        else
        {
            printf("Get depth capture returned error: %d\n", get_capture_result);
            break;
        }

    } while (frame_count < 100);

    printf("Finished body tracking processing!\n");

    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);
    k4a_device_stop_cameras(device);
    k4a_device_close(device);

    return 0;
}

double Pitch(k4a_quaternion_t quaternion) {
    double value1 = - 2.0 * (quaternion.wxyz.w * quaternion.wxyz.x + quaternion.wxyz.y * quaternion.wxyz.z);
    double value2 = 1.0 - 2.0 * (quaternion.wxyz.x * quaternion.wxyz.x + quaternion.wxyz.y * quaternion.wxyz.y);

    double roll = atan2(value1, value2);

    return roll * (180.0 / PI)+ 180;
}  

double Yaw(k4a_quaternion_t quaternion) {
    double value = - 2.0 * (quaternion.wxyz.w * quaternion.wxyz.y - quaternion.wxyz.z * quaternion.wxyz.x);
    value = value > 1.0 ? 1.0 : value;
    value = value < -1.0 ? -1.0 : value;

    double pitch = asin(value);

    return pitch * (180.0 / PI)+ 180;
}

double Roll(k4a_quaternion_t quaternion) {
    double value1 = 2.0 * (quaternion.wxyz.w * quaternion.wxyz.z + quaternion.wxyz.x * quaternion.wxyz.y);
    double value2 = 1.0 - 2.0 * (quaternion.wxyz.y * quaternion.wxyz.y + quaternion.wxyz.z * quaternion.wxyz.z);

    double yaw = atan2(value1, value2);

    return yaw * (180.0 / PI) + 180;
}