#include "libfreenect.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <pthread.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#define PI 3.14159265

using namespace cv;
using namespace std;

class Mutex {
public:
	Mutex() {
		pthread_mutex_init( &m_mutex, NULL );
	}
	void lock() {
		pthread_mutex_lock( &m_mutex );
	}
	void unlock() {
		pthread_mutex_unlock( &m_mutex );
	}
private:
	pthread_mutex_t m_mutex;
};

class MyFreenectDevice : public Freenect::FreenectDevice {
public:
	MyFreenectDevice(freenect_context *_ctx, int _index)
	: Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(2000),m_buffer_rgb(2000), m_gamma(2048), m_new_rgb_frame(false), m_new_depth_frame(false),
	depthMat(Size(640,480),CV_16UC1), rgbMat(Size(640,480),CV_8UC3,Scalar(0)), ownMat(Size(640,480),CV_8UC3,Scalar(0))
	{
		for( unsigned int i = 0 ; i < 2048 ; i++) {
			float v = i/2048.0;
			v = std::pow(v, 3)* 6;
			m_gamma[i] = v*6*256;
		}
	}
	// Do not call directly even in child
	void VideoCallback(void* _rgb, uint32_t timestamp) {
		//std::cout << "RGB callback" << std::endl;
		m_rgb_mutex.lock();
		uint8_t* rgb = static_cast<uint8_t*>(_rgb);
		rgbMat.data = rgb;
		m_new_rgb_frame = true;
		m_rgb_mutex.unlock();
	};
	// Do not call directly even in child
	void DepthCallback(void* _depth, uint32_t timestamp) {
		//std::cout << "Depth callback" << std::endl;
		m_depth_mutex.lock();
		uint16_t* depth = static_cast<uint16_t*>(_depth);
		depthMat.data = (uchar*) depth;
		m_new_depth_frame = true;
		m_depth_mutex.unlock();
	}
	
	bool getVideo(Mat& output) {
		m_rgb_mutex.lock();
		if(m_new_rgb_frame) {
			cv::cvtColor(rgbMat, output, CV_RGB2BGR);
			m_new_rgb_frame = false;
			m_rgb_mutex.unlock();
			return true;
		} else {
			m_rgb_mutex.unlock();
			return false;
		}
	}
	
	bool getDepth(Mat& output) {
		m_depth_mutex.lock();
		if(m_new_depth_frame) {
			depthMat.copyTo(output);
			m_new_depth_frame = false;
			m_depth_mutex.unlock();
			return true;
		} else {
			m_depth_mutex.unlock();
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
	Mutex m_rgb_mutex;
	Mutex m_depth_mutex;
	bool m_new_rgb_frame;
	bool m_new_depth_frame;
};

CvScalar hsv2rgb( float hue )
{
	int rgb[3], p, sector;
	static const int sector_data[][3]=
        {{0,2,1}, {1,2,0}, {1,0,2}, {2,0,1}, {2,1,0}, {0,1,2}};
	hue *= 0.033333333333333333333333333333333f;
	sector = cvFloor(hue);
	p = cvRound(255*(hue - sector));
	p ^= sector & 1 ? 255 : 0;
	
	rgb[sector_data[sector][0]] = 255;
	rgb[sector_data[sector][1]] = 0;
	rgb[sector_data[sector][2]] = p;
	
	return cvScalar(rgb[2], rgb[1], rgb[0],0);
}

void OverlayImage(IplImage* src, IplImage* overlay, CvPoint location, CvScalar S, CvScalar D)
{
	for(int x=0;x<480;x++)
	{
		if(x+location.x>=640) continue;
		for(int y=0;y<118;y++)
		{
			if(y+location.y>=480) continue;
			CvScalar source = cvGet2D(src, y+location.y, x+location.x);
			CvScalar over = cvGet2D(overlay, y, x);
			CvScalar merged;
			for(int i=0;i<4;i++)
				merged.val[i] = (S.val[i]*source.val[i]+D.val[i]*over.val[i]);
			cvSet2D(src, y+location.y, x+location.x, merged);
		}
	}
}

int main(int argc, char **argv) {
	bool die(false);
	string filename("snapshot");
	string suffix(".png");
	int i_snap(0),iter(0);
	
	CvHistogram *hist = 0;
	int hdims = 16 ;
	float hranges_arr[] = {0,180};
	float* hranges = hranges_arr;
	
	hist = cvCreateHist(1,&hdims, CV_HIST_ARRAY, &hranges,1);
	
	IplImage* histimg = cvCreateImage (cvSize(640,480),8,3);
	cvZero(histimg);
	
	bool Nobody = TRUE, Arrived = FALSE, Playing = FALSE;
	int Wait = 4;
	Mat depthMat(Size(640,480),CV_16UC1);
	Mat depthf  (Size(640,480),CV_8UC1);
	Mat rgbf(Size(640,480),CV_8UC3);
	Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));
	Mat depthf_dest (Size(640,480),CV_8UC1);
	
        Freenect::Freenect freenect;
        MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);
	
	namedWindow("rgb",CV_WINDOW_AUTOSIZE);
	
	device.startVideo();
	device.startDepth();
	
	//Number of points in the game
	int points_game = 0;
	double minVal = 1000, maxVal= 0, old_minVal =0;
	//To create bubble movement we use the sin and its value is val_sin
	double val_sin = 0;
	// The bubbles are circles with the coordinates x_circle[i] and y_circle[i]
	int x_circle[8] , y_circle[8] ;
	
	
	bool order = TRUE; //to create disorder
	srand ( time(NULL) );
	
	for (int i = 0; i<8; i++) {
		x_circle[i] = rand()%600 + 30;
		y_circle[i] = 500 + rand()%90;
	}
	
	
	
	
	std::ostringstream oss, timer_t;
	
	//Idle is used when nobody is in front of the camera this way the progam goes idle
	int idle = 1000;
	//timer is the time per game
	int timer = 30;
	std::string result, temps;
	time_t sec, prev_sec;
	prev_sec = time(&prev_sec);
	bool fini = false;
	
	
	while (!die) {
		device.getVideo(rgbMat);
		device.getDepth(depthMat);
		depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);
		
		Point minLoc, maxLoc;
		if (minVal != 1000) {
			old_minVal = minVal;
		}
		/*cout << "minVal : ";
		 cout << minVal;
		 cout << "\n";
		 cout << "old : ";
		 cout << old_minVal;
		 cout << "\n";*/
		
		cv::minMaxLoc(depthf, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
		
		
		threshold(depthf, depthf_dest, minVal+3, 255, THRESH_TOZERO);
		
		
		//Find the middle of the blob (defined by zeros in the matrix)
		int sum_x =0, sum_y=0, count=0, x_avg=0, y_avg=0;
		
		for(int i = 0; i < depthf_dest.rows; i++)	
		{
			for(int j = 0; j < depthf_dest.cols; j++) {
				if (depthf_dest.data[640*i+j] == 0) {
					sum_x += j;
					sum_y += i;
					count++;
				}
			}
		}
		
		//Middle of the blob 
		x_avg = sum_x/count;
		y_avg = sum_y/count;
		
		
		//Create circle as bubble burster for the game centered on the middle of the blob -- Use open NI to take only hand
		cv::circle(rgbMat, cv::Point(x_avg, y_avg), 0, cv::Scalar(0,250,0), 2, 0,0);
		
		
		/*****************
		 *               *
		 *  Bubble Game  *
		 *               *
		 *****************/
		rgbMat.convertTo(rgbf, CV_8UC1, 255.0/2048.0);
		IplImage *img_rgb = new IplImage(rgbMat);
		IplImage *img1 = cvLoadImage("./soda_can.png");
		CvPoint pt1;
		pt1.x=10;pt1.y=10;
		CvScalar S = cvScalar(0.5, 0.5, 0.5, 0.5);
		CvScalar D = cvScalar(0.5, 0.5, 0.5, 0.5);
		OverlayImage(img_rgb, img1, pt1 , S,D);
		
		cout << "DEPTH OF BLOB : ";
		cout << (int)depthf.data[y_avg*640+x_avg];
		cout << "\n";
		if (depthf.data[y_avg*640+x_avg] < 75 && Nobody && Wait==0) {
			Nobody = FALSE, Arrived = TRUE, Playing = FALSE;
			timer = 30;
			points_game = 0;
		} 
		
		//Allows to avoid minVal = 0 at initialization
		if (Wait != 0) {
			Wait--;
		}
		
		
		if (Arrived) 
			cv::putText(rgbMat, "Pop a bubble !", Point(60,200), FONT_HERSHEY_SIMPLEX, 2,  cv::Scalar(0,0,250), 6, 6, false);
		
		
		
		
		//Each bubble of the game
		for (int k = 0; k<8; k++) {
			
			cv::circle(rgbMat, cv::Point(x_circle[k],y_circle[k]), 30, cv::Scalar(250,180,0), 2.5, 8, 0);
			y_circle[k]= y_circle[k] - 1;
			
			//If the bubble goes to the top of screen
			if (y_circle[k] < -40) {
				y_circle[k] = 500 + rand()%60;
			}
			val_sin = val_sin + 0.04;
			//Avoid overflow
			if (val_sin > 300) {
				val_sin = 0;
			}
			order = !order;
			if (order) {
				x_circle[k] = x_circle[k] + cos(val_sin)*3.4 +0.6;
				
			} else {
				x_circle[k] = x_circle[k] + sin(val_sin+13)*3.7 +0.6;
			}
			
			
			
			if ( !Nobody ){
				//Sets the timer and idle
				sec = time(&sec);
				
				if (sec-prev_sec >0 && (timer !=0)) {
					timer--;
					prev_sec = sec;
					
				}
				if (Playing){
					if (timer > 26) {
						cv::putText(rgbMat, "Pop as many", Point(90,200), FONT_HERSHEY_SIMPLEX, 2,  cv::Scalar(0,0,250), 5, 5, false);
						cv::putText(rgbMat, "as possible !", Point(70,300), FONT_HERSHEY_SIMPLEX, 2,  cv::Scalar(0,0,250), 5, 5, false);
						
					}
				}
				
				//Detects when bubble popped
				if (x_avg < x_circle[k]+30 && x_avg > x_circle[k]-30 && y_avg > y_circle[k]-30 && y_avg < y_circle[k]+30 && old_minVal-minVal > 1) 
				{
					if (!Playing) {
						Playing = TRUE;
						Arrived = FALSE;
						timer = 30;
						points_game = 0;
					}
					
					x_circle[k] = rand()%640;
					y_circle[k] = 500+ rand()%60;
					points_game++;
					oss.str("");
					oss << points_game;
					result = oss.str();
				}
				
				
				if (!Arrived) {
					cv::putText(rgbMat, "score", Point(450,445), FONT_HERSHEY_SCRIPT_SIMPLEX, 2,  cv::Scalar(20,0,200), 2, 2, false);					
					cv::putText(rgbMat, result, Point(565,445), FONT_HERSHEY_SIMPLEX, 2,  cv::Scalar(20,0,200), 2, 2, false);					
					timer_t.str("");
					timer_t << timer;
					temps = timer_t.str();
					cv::putText(rgbMat, temps, Point(500,60), FONT_HERSHEY_SIMPLEX, 2,  cv::Scalar(250,250,250), 8, 8, false);
				}
			}
			
			if (timer == 0 && !Nobody) {
				
				
				if (idle !=0) {
					idle--;
					
					if (points_game > 7) {
						cv::putText(rgbMat, "Nice score!", Point(130,200), FONT_HERSHEY_SCRIPT_SIMPLEX, 2,  cv::Scalar(250,0,0), 5, 5, false);
						cv::putText(rgbMat, "You could use a drink", Point(30,300), FONT_HERSHEY_SCRIPT_SIMPLEX, 2,  cv::Scalar(250,0,0), 5, 5, false);					
					} else if (points_game < 7) {
						cv::putText(rgbMat, "Humm... are", Point(30,200), FONT_HERSHEY_SCRIPT_SIMPLEX, 2,  cv::Scalar(250,0,0), 3, 3, false);
						cv::putText(rgbMat, "you awake ?", Point(60,260), FONT_HERSHEY_SCRIPT_SIMPLEX, 2,  cv::Scalar(250,0,0), 3, 3, false);
						cv::putText(rgbMat, "Maybe you need a drink !", Point(10,320), FONT_HERSHEY_SCRIPT_SIMPLEX, 2,  cv::Scalar(250,0,0), 3, 3, false);
					}
				}
				if (idle == 0 ) {
					Playing = FALSE;
					Arrived = FALSE;
					Nobody  = TRUE;
					timer = 30;
					points_game = 0;
					idle = 1000;
					cv::putText(rgbMat, "", Point(40,200), FONT_HERSHEY_SIMPLEX, 3,  cv::Scalar(0,0,250), 4, 4, false);
					
				}
				
			}
			
			
			
			//while (timer == 0) {
			//			cv::putText(rgbMat, "Come closer!", Point(40,200), FONT_HERSHEY_SIMPLEX, 3,  cv::Scalar(0,0,250), 8, 8, false);
			//			//	if (mouvement) {
			//			timer = 20;
			//			cv::putText(rgbMat, "Make a T with your body to start playing!", Point(40,200), FONT_HERSHEY_SIMPLEX, 3,  cv::Scalar(0,0,250), 8, 8, false);
			//			//}
			//		}
			
			
		}
		
		
		cout << "idle = ";
		cout << idle;
		cout << "\n";
		cout << "points : ";
		cout << points_game;
		cout << "\n";
		cout << "timer : ";
		cout << timer;
		cout << "\n";
		cout << "val sin == ";
		cout << val_sin;
		cout << "\n";
		
		cvShowImage("rgb", img_rgb);
		
		char k = cvWaitKey(5);
		if( k == 27 ){
			cvDestroyWindow("rgb");
			cvDestroyWindow("depth");
			break;
		}
		if( k == 8 ) {
			std::ostringstream file;
			file << filename << i_snap << suffix;
			cv::imwrite(file.str(),rgbMat);
			i_snap++;
		}
		//if(iter >= 1000) break;
		iter++;
		
	}
	
	device.stopVideo();
	device.stopDepth();
	return 0;
}

