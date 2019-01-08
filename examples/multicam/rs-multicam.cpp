// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include "example.hpp"              // Include short list of convenience functions for rendering
#include <librealsense2/hpp/rs_frame.hpp>

#include <thread>
#include <string>
#include <map>
#include <algorithm>
#include <mutex>                    // std::mutex, std::lock_guard
#include <cmath>                    // std::ceil
#include <fstream>              // File IO
#include <iostream>             // Terminal IO
#include <sstream>              // Stringstreams
#include <librealsense2/rs_advanced_mode.hpp>
#include <librealsense2/rsutil.h>

// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

using namespace rs400;

const std::string no_camera_message = "No camera connected, please connect 1 or more";
const std::string platform_camera_name = "Platform Camera";

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


class device_container
{
    // Helper struct per pipeline
    struct view_port
    {
		std::string dev;
        std::map<int, rs2::frame> frames_per_stream;
        rs2::colorizer colorize_frame;
        texture tex;
        rs2::pipeline pipe;
        rs2::pipeline_profile profile;
    };

public:

	//std::vector<rs2::frameset> list;
	std::vector<std::pair<std::string,rs2::frame>> list;

    void enable_device(rs2::device dev)
    {
        std::string serial_number(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        std::lock_guard<std::mutex> lock(_mutex);

        if (_devices.find(serial_number) != _devices.end())
        {
            return; //already in
        }

        // Ignoring platform cameras (webcams, etc..)
        if (platform_camera_name == dev.get_info(RS2_CAMERA_INFO_NAME))
        {
            return;
        }
        // Create a pipeline from the given device
        rs2::pipeline p;
        rs2::config c;
        c.enable_device(serial_number);
        // Start the pipeline with the configuration
        rs2::pipeline_profile profile = p.start(c);
        // Hold it internally
        _devices.emplace(serial_number, view_port{ serial_number,{},{},{}, p, profile });

    }

	void set_masterslave(rs2::context ctx)
	{
		std::string MASTER_SERIAL = ctx.query_devices()[0].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
		for (auto&& dev : ctx.query_devices()) {
			const std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
			fprintf(stderr, "Starting serial %s\n", serial.c_str());

			auto depth = dev.first<rs2::depth_sensor>();
			if (depth.supports(RS2_OPTION_INTER_CAM_SYNC_MODE)) {
				int mode = (serial == MASTER_SERIAL ? 1 : 2);
				printf("Set sync mode = %d\n", mode);
				depth.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, mode);
				std::cout<<"Read sync mode ="<<depth.get_option(RS2_OPTION_INTER_CAM_SYNC_MODE)<<std::endl;

				////Set manual exp and disable laser
				//for (rs2::sensor& sensor : dev.query_sensors()) {
				//	if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>()) {

				//		if (dpt.supports(RS2_OPTION_EXPOSURE)) {

				//			dpt.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.0f);
				//			dpt.set_option(RS2_OPTION_EXPOSURE, 30000.0f);
				//		}
				//		if (dpt.supports(RS2_OPTION_EMITTER_ENABLED)) {

				//			dpt.set_option(RS2_OPTION_EMITTER_ENABLED, 0.0f);
				//		}

				//	}
				//	else if (sensor.supports(RS2_OPTION_EXPOSURE)) {

				//		sensor.set_option(RS2_OPTION_EXPOSURE, 30000.0f);
				//	}
				//}
			}
		}
	}

    void remove_devices(const rs2::event_information& info)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        // Go over the list of devices and check if it was disconnected
        auto itr = _devices.begin();
        while(itr != _devices.end())
        {
            if (info.was_removed(itr->second.profile.get_device()))
            {
                itr = _devices.erase(itr);
            }
            else
            {
                ++itr;
            }
        }
    }

    size_t device_count()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _devices.size();
    }

    int stream_count()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        int count = 0;
        for (auto&& sn_to_dev : _devices)
        {
            for (auto&& stream : sn_to_dev.second.frames_per_stream)
            {
                if (stream.second)
                {
                    count++;
                }
            }
        }
        return count;
    }

	//int save_to_raw(uint16_t *image, int imageWidth, int imageHeight, const char *filename)
	//{
	//	char fname_bmp[128];
	//	sprintf(fname_bmp, "%s.raw", filename);

	//	FILE *fp;
	//	if (!(fp = fopen(fname_bmp, "wb")))
	//		return -1;
	//	fwrite(image, sizeof(uint16_t), (size_t)(long)imageWidth*imageHeight, fp);
	//	fclose(fp);
	//	return 0;
	//}
	
	bool save_frame_raw_data(const std::string& filename, rs2::frame frame)
	{
		bool ret = false;
		auto image = frame.as<rs2::video_frame>();
		if (image)
		{
			std::ofstream outfile(filename.data(), std::ofstream::binary);
			outfile.write(static_cast<const char*>(image.get_data()), image.get_height()*image.get_stride_in_bytes());

			outfile.close();
			ret = true;
		}

		return ret;
	}

    void poll_frames()
    {
        std::lock_guard<std::mutex> lock(_mutex);

		// Declare depth colorizer for pretty visualization of depth data
		rs2::colorizer color_map;

		skip_frame_num++;

        // Go over all device

			for (auto&& view : _devices)
			{
				printf("sn: %s\n", view.second.dev.c_str());
				sn_frame.first = view.second.dev;
				// Ask each pipeline if there are new frames available
				rs2::frameset frameset;
				if (view.second.pipe.poll_for_frames(&frameset))
				{
					////keep frame
					//auto raw_data = frameset.get_depth_frame();
					//raw_data.keep();
					//list.push_back(raw_data);
					//std::_Count_pr << list.size << std::endl;

					//printf("%s, ae=%lld\n", rs2_stream_to_string(frame.get_profile().stream_type()), exp);

					for (int i = 0; i < frameset.size(); i++)
					{
						rs2::frame frame = frameset[i];
						//keep frame
						//auto raw_data = frameset.get_depth_frame();
						//raw_data.keep();
						if(skip_frame_num>0)
						{ 
						frame.keep();
						sn_frame.second = frame;
						list.emplace_back(sn_frame);
						}

						//auto exp = frame.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
						auto ts_bkend = frame.get_frame_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP);
						auto ts_toa = frame.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL);
						auto ts_frame = frame.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP);
						auto ts_sensor = frame.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP);
						auto frm_id = frame.get_frame_number();
						auto frm_cnt = frame.get_frame_metadata(RS2_FRAME_METADATA_FRAME_COUNTER);
						auto frm_tp = rs2_stream_to_string(frame.get_profile().stream_type());
						printf("                %s id=%lld cnt=%lld tsbk=%lld tstoa=%lld tsf=%lld tss=%lld\n", frm_tp, frm_id, frm_cnt, ts_bkend, ts_toa, ts_frame, ts_sensor);

						int stream_id = frame.get_profile().unique_id();
						view.second.frames_per_stream[stream_id] = view.second.colorize_frame.process(frame); //update view port with the new stream


					}
				}
			}
		
    }

	void rank_frames()
	{







	}

	void save_frames()
	{
		//std::lock_guard<std::mutex> lock(_mutex);

		std::cout << "list size = " << list.size() << std::endl;
		for (auto frame : list)
		{

				//auto exp = frame.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
				auto ts_bkend = frame.second.get_frame_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP);
				//auto ts_toa = frame.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL);
				//auto ts_frame = frame.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP);
				//auto ts_sensor = frame.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP);
				//auto frm_id = frame.get_frame_number();
				auto frm_cnt = frame.second.get_frame_metadata(RS2_FRAME_METADATA_FRAME_COUNTER);
				auto frm_tp = rs2_stream_to_string(frame.second.get_profile().stream_type());
				//printf("                %s id=%lld cnt=%lld tsbk=%lld tstoa=%lld tsf=%lld tss=%lld\n", frm_tp, frm_id, frm_cnt, ts_bkend, ts_toa, ts_frame, ts_sensor);

				//save image to disk 
				// We can only save video frames as pngs, so we skip the rest
				if (auto vf = frame.second.as<rs2::video_frame>())
				{

					auto stream = frame.second.get_profile().stream_type();

					// Use the colorizer to get an rgb image for the depth stream
					if (vf.is<rs2::depth_frame>()) //vf = color_map.process(frame);
					{
						//save depth to raw
						std::stringstream raw_file;
						std::string raw_filename;
						//raw_file <<"sn"<<view.second.dev <<"_ts"<<ts_bkend<<"_cnt"<<frm_cnt<< "-" << vf.get_profile().stream_name() << ".raw";
						//raw_file << vf.get_profile().stream_name()<<"_sn"<<view.second.dev << "_cnt" <<frm_cnt<<"_ts"<<ts_bkend<< ".raw";
						raw_file << "fc" << frm_cnt << "_ts" << ts_bkend << "_sn" << frame.first << "_" << vf.get_profile().stream_name() << ".raw";
						//raw_file << "fc" << frm_cnt << "_ts" << ts_bkend << vf.get_profile().stream_name() << ".raw";

						raw_file >> raw_filename;
						raw_filename = ".\\images\\" + raw_filename;
						if (save_frame_raw_data(raw_filename, frame.second))
							std::cout << "Raw data is captured into " << raw_filename << std::endl;
					}
 
					else
					{
						//cause delay
						// Write images to disk
						std::stringstream png_file;
						std::string png_filename;
						png_file << "fc" << frm_cnt << "_ts" << ts_bkend << "_sn" << frame.first << "_" << vf.get_profile().stream_name() << ".png";
						png_file >> png_filename;
						png_filename = ".\\images\\" + png_filename;
						stbi_write_png(png_filename.c_str(), vf.get_width(), vf.get_height(),
							vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
						std::cout << "Saved " << png_filename << std::endl;
# if(0)
						//// Record per-frame metadata for UVC streams
						//std::stringstream csv_file;
						//csv_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name()
						//	<< "-metadata.csv";
						//metadata_to_csv(vf, csv_file.str());

					
# endif
					}
				}

				
			
		}
	}



	void clear_frames()
	{
		list.clear();
	}

	void metadata_to_csv(const rs2::frame& frm, const std::string& filename)
	{
		std::ofstream csv;

		csv.open(filename);

		//    std::cout << "Writing metadata to " << filename << endl;
		csv << "Stream," << rs2_stream_to_string(frm.get_profile().stream_type()) << "\nMetadata Attribute,Value\n";

		// Record all the available metadata attributes
		for (size_t i = 0; i < RS2_FRAME_METADATA_COUNT; i++)
		{
			if (frm.supports_frame_metadata((rs2_frame_metadata_value)i))
			{
				csv << rs2_frame_metadata_to_string((rs2_frame_metadata_value)i) << ","
					<< frm.get_frame_metadata((rs2_frame_metadata_value)i) << "\n";
			}
		}

		csv.close();
	}

    void render_textures(int cols, int rows, float view_width, float view_height)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        int stream_no = 0;
        for (auto&& view : _devices)
        {
            // For each device get its frames
            for (auto&& id_to_frame : view.second.frames_per_stream)
            {
                rect frame_location{ view_width * (stream_no % cols), view_height * (stream_no / cols), view_width, view_height };
                if (rs2::video_frame vid_frame = id_to_frame.second.as<rs2::video_frame>())
                {
                    view.second.tex.render(vid_frame, frame_location);
                    stream_no++;
                }
            }
        }
    }
private:
    std::mutex _mutex;
    std::map<std::string, view_port> _devices;
	std::pair<std::string, rs2::frame> sn_frame;
	int skip_frame_num = -10;
};


int main(int argc, char * argv[]) try
{
    // Create a simple OpenGL window for rendering:
    window app(1280, 960, "CPP Multi-Camera Example");

    device_container connected_devices;

    rs2::context ctx;    // Create librealsense context for managing devices

                         // Register callback for tracking which devices are currently connected
    ctx.set_devices_changed_callback([&](rs2::event_information& info)
    {
        connected_devices.remove_devices(info);
        for (auto&& dev : info.get_new_devices())
        {
            connected_devices.enable_device(dev);
        }
    });

    // Initial population of the device list
    for (auto&& dev : ctx.query_devices()) // Query the list of connected RealSense devices
    {
        connected_devices.enable_device(dev);
    }
	connected_devices.set_masterslave(ctx);

	int index = 0;
    while (index<30) // Application still alive?
    {
        connected_devices.poll_frames();
		index++;

		//save raw when got frameset num = 10 * camera num
		printf("index=%d\n", index);



        auto total_number_of_streams = connected_devices.stream_count();
        if (total_number_of_streams == 0)
        {
            draw_text(int(std::max(0.f, (app.width() / 2) - no_camera_message.length() * 3)),
                      int(app.height() / 2), no_camera_message.c_str());
            continue;
        }
        if (connected_devices.device_count() == 1)
        {
            draw_text(0, 10, "Please connect another camera");
        }
        int cols = int(std::ceil(std::sqrt(total_number_of_streams)));
        int rows = int(std::ceil(total_number_of_streams / static_cast<float>(cols)));

        float view_width = (app.width() / cols);
        float view_height = (app.height() / rows);

        connected_devices.render_textures(cols, rows, view_width, view_height);
    }
	//connected_devices.rank_frames();
	connected_devices.save_frames();
	connected_devices.clear_frames();
	//index = 0;
	
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
