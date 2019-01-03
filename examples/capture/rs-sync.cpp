// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.
/*
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

// Capture Example demonstrates how to
// capture depth and color video streams and render them to the screen
int main(int argc, char * argv[]) try
{
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Capture Example");

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    // Declare rates printer for showing streaming rates of the enabled streams.
    rs2::rates_printer printer;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Start streaming with default recommended configuration
    // The default video configuration contains Depth and Color streams
    // If a device is capable to stream IMU data, both Gyro and Accelerometer are enabled by default 
    pipe.start();

    while (app) // Application still alive?
    {
        rs2::frameset data = pipe.wait_for_frames().    // Wait for next set of frames from the camera
                             apply_filter(printer).     // Print each enabled stream frame rate
                             apply_filter(color_map);   // Find and colorize the depth data

        // The show method, when applied on frameset, break it to frames and upload each frame into a gl textures
        // Each texture is displayed on different viewport according to it's stream unique id
        app.show(data);
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
*/

#include <librealsense2/rs.hpp>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// IMPORTANT
// Replace this with the correct value.
//static const std::string MASTER_SERIAL = "701512070108";

static const std::vector<uint8_t> master_command{
	0x14, 0x00, 0xab, 0xcd, 0x64, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const std::vector<uint8_t> slave_command{
	0x14, 0x00, 0xab, 0xcd, 0x64, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const std::vector<uint8_t> master_gpio_dir_command{
	0x18, 0x00, 0xab, 0xcd, 0x02, 0x00, 0x00, 0x00, 0x30, 0x20, 0x01, 0x00,
	0x34, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x44, 0x4C, 0x21, 0x00 };
static const std::vector<uint8_t> master_gpio_toggle_command{
	0x18, 0x00, 0xab, 0xcd, 0x02, 0x00, 0x00, 0x00, 0x34, 0x20, 0x01, 0x00,
	0x38, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x40, 0x44, 0x21, 0x00 };
static const std::vector<uint8_t> master_gpio_dir_chk_command{
	0x14, 0x00, 0xab, 0xcd, 0x02, 0x00, 0x00, 0x00, 0x30, 0x20, 0x01, 0x00,
	0x34, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
struct Realsense {
	rs2::device device;
	rs2::pipeline pipeline;
	rs2::config config;
	std::thread thread;
	rs2::frameset frameset;
};

void processFrames(std::shared_ptr<Realsense> rs) {
	while (1) {
		if (!rs->pipeline.try_wait_for_frames(&rs->frameset, 100)) {
			fprintf(stderr, "Wait for frames timed out\n");
			continue;
		}
		// Try to read the exposure field.
		// Sometimes, this throws on each frameset for the master camera.
		try {
			for (auto&& frame : rs->frameset) {
				auto exp = frame.get_frame_metadata(
					RS2_FRAME_METADATA_ACTUAL_EXPOSURE);

				auto ts_bkend = frame.get_frame_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP);
				auto ts_toa = frame.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL);
				auto ts_frame = frame.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP);
				auto ts_sensor = frame.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP);
				auto frm_id = frame.get_frame_number();
				auto frm_cnt = frame.get_frame_metadata(RS2_FRAME_METADATA_FRAME_COUNTER);
				printf("%lld %lld %lld %lld %lld %lld", ts_toa, ts_bkend, ts_frame, ts_sensor,frm_id,frm_cnt);
				printf("sn: %s, %s, ae=%lld\n", rs->device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), rs2_stream_to_string(frame.get_profile().stream_type()), exp);

			}
			printf("\n");
		}
		catch (const rs2::error& e) {
			fprintf(stderr, "Frame metadata / exposure is invalid\n");
		}
	}
}

int main() try {
	rs2::context context;
	std::vector<std::shared_ptr<Realsense>> rs_vec;
	std::string MASTER_SERIAL = context.query_devices()[0].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
	for (auto&& dev : context.query_devices()) {
		const std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
		fprintf(stderr, "Starting serial %s\n", serial.c_str());
		rs_vec.emplace_back(std::make_shared<Realsense>());
		auto rs = rs_vec.back();
		rs->device = dev;
		rs->config.enable_device(serial);
		rs->config.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
		rs->config.enable_stream(RS2_STREAM_INFRARED, 1280, 720, RS2_FORMAT_Y8, 30);
		//rs->config.enable_stream(RS2_STREAM_FISHEYE, 640, 480, RS2_FORMAT_RAW8, 15);
		rs->config.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
		
		auto depth = dev.first<rs2::depth_sensor>();
		if (depth.supports(RS2_OPTION_INTER_CAM_SYNC_MODE)) {
			int mode = (serial == MASTER_SERIAL ? 1 : 2);
			printf("Sync mode = %d\n", mode);
			depth.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, mode);
			//if (mode == 1) {
			//	auto debug = dev.as<rs2::debug_protocol>();
			//	debug.send_and_receive_raw_data(master_gpio_dir_command);
			//	debug.send_and_receive_raw_data(master_gpio_toggle_command);
			//}

		}
		//else {
		//	auto debug = dev.as<rs2::debug_protocol>();
		//	if (serial == MASTER_SERIAL) {
		//		printf("Old Sync mode = master\n");
		//		debug.send_and_receive_raw_data(master_command);
		//	}
		//	else {
		//		debug.send_and_receive_raw_data(slave_command);
		//	}
		//}
		


		//Set manual exp and disable laser
		for (rs2::sensor& sensor : dev.query_sensors()) {
			if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>()) {

				if (dpt.supports(RS2_OPTION_EXPOSURE)) {

					dpt.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.0f);
					dpt.set_option(RS2_OPTION_EXPOSURE, 10000.0f);
				}
				if (dpt.supports(RS2_OPTION_EMITTER_ENABLED)) {

					dpt.set_option(RS2_OPTION_EMITTER_ENABLED, 0.0f);
				}

			}
			else if (sensor.supports(RS2_OPTION_EXPOSURE)) {

				sensor.set_option(RS2_OPTION_EXPOSURE, 10000.0f);
			}
		}

		rs->pipeline.start(rs->config);

		rs->thread = std::thread(&processFrames, rs);
	}

	for (auto&& rs : rs_vec) {
   		rs->thread.join();
	}
}
catch (const rs2::error& e) {
	fprintf(stderr, "Caught rs2::error when calling %s [%s]\n",
		e.get_failed_function().c_str(), e.what());
}
