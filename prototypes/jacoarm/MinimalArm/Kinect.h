#ifndef KINECT_H
#define KINECT_H

#include <mutex>
#include <vector>

class MyFreenectDevice : public Freenect::FreenectDevice {
	public:
		MyFreenectDevice(freenect_context *_ctx, int _index)
	 		: Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(FREENECT_DEPTH_11BIT),
			m_buffer_rgb(FREENECT_VIDEO_RGB), m_gamma(2048), m_new_rgb_frame(false),
			m_new_depth_frame(false), depthMat(Size(640,480),CV_16UC1),
			rgbMat(Size(640,480), CV_8UC3, Scalar(0)),
			ownMat(Size(640,480),CV_8UC3,Scalar(0)) {
			
			for( unsigned int i = 0 ; i < 2048 ; ++i) {
				float v = i/2048.0;
				v = std::pow(v, 3)* 6;
				m_gamma[i] = v*6*256;
			}
		}
		
		// Do not call directly even in child
		void VideoCallback(void* _rgb, uint32_t timestamp) {
			std::cout << "RGB callback" << std::endl;
			
			{
				std::lock_guard<std::mutex> lock(m_rgb_mutex);
			
				uint8_t* rgb = static_cast<uint8_t*>(_rgb);
				rgbMat.data = rgb;
				m_new_rgb_frame = true;
			}
		};
		
		// Do not call directly even in child
		void DepthCallback(void* _depth, uint32_t timestamp) {
			std::cout << "Depth callback" << std::endl;
			{
				std::lock_guard<std::mutex> lock(m_depth_mutex);
				
				uint16_t* depth = static_cast<uint16_t*>(_depth);
				depthMat.data = (uchar*) depth;
				m_new_depth_frame = true;
			}
		}
		
		bool getVideo(Mat& output) {
			std::lock_guard<std::mutex> lock(m_rgb_mutex);
			
			if(m_new_rgb_frame) {
				cv::cvtColor(rgbMat, output, CV_RGB2BGR);
				m_new_rgb_frame = false;
				return true;
			} else {
				return false;
			}
		}
		
		bool getDepth(Mat& output) {
			std::lock_guard<std::mutex> lock(m_depth_mutex);
			
				if(m_new_depth_frame) {
					depthMat.copyTo(output);
					m_new_depth_frame = false;
					return true;
				} else {
					return false;
				}
			}
	private:
		std::vector<uint8_t> m_buffer_depth;
		std::vector<uint8_t> m_buffer_rgb;
		std::vector<uint16_t> m_gamma;
		Mat depthMat;
		Mat rgbMat;
		Mat ownMat;
		std::mutex m_rgb_mutex;
		std::mutex m_depth_mutex;
		bool m_new_rgb_frame;
		bool m_new_depth_frame;
};

#define
