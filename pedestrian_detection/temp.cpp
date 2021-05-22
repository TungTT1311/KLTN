#include "object_detection.h"
#include "camera_tracking.h"
#include "robot_move.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <opencv4/opencv2/tracking/tracker.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/face.hpp>
#include <math.h>
#include <dlib/image_processing.h>
// #include <dlib/gui_widgets.h>
#include <dlib/dir_nav.h>
#include <dlib/image_io.h>
#include <dlib/opencv.h>
#include <fstream>
#include <sstream>
#include <chrono>
#include <iostream>
using namespace std;
using namespace cv;
using namespace dlib;
using namespace cv::face;
// NEW

CascadeClassifier face_cascade;


int pan = 90;
int serial_port;

cv::Rect2d r;


int main( int argc, char** argv )
{
  // Config to Camera tracking
  // Open(serial_port);
  // Config_SerialPort(serial_port); 
  // LobotSerialServoMove(1, pan, 0, serial_port);

  //-----------------------------DETECT FACE-------------------------------
  //face recognizer 


  bool RUNFACE_DETECTION=false;



  Ptr<FaceRecognizer> model = EigenFaceRecognizer::create();
  model->read("eigenface.yml");
  int img_width = 128;
	int img_height = 128;
  if (!face_cascade.load("haarcascade_frontalface_alt.xml")) {
		cout << " Error loading file" << endl;
		return -1;
	}
  //-----------------------------DETECT FACE-------------------------------END
  std::string pipeline = "v4l2src device=/dev/video0 ! image/jpeg, width=(int)640, height=(int)480, framerate=30/1 ! jpegdec ! videoconvert ! appsink";
  std::string wriweb= "appsrc ! videoconvert ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! jpegenc ! rtpjpegpay ! udpsink host=127.0.0.1 port=5000";
  cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
  cv::Mat frame;
  cv::VideoWriter out(wriweb, 0,	 
  30,
  cv::Size(640, 480), 
  true);
  ObjectDetection det("../../pedestrian_detection/");
  
  auto m_StartTime = std::chrono::system_clock::now();

  //correlation_tracker tracker(int filter_size = 6);

  cv::Mat cimg;

  bool isInitTracking = false;
  int startX, startY, endX, endY;

  Ptr<TrackerCSRT> tracker = TrackerCSRT::create();


  std::string Pname= "";
  // image_window win;

  while(true)
  {
    std::vector<cv::Rect> faces;
		Mat frame;
		Mat graySacleFrame;
		Mat original;
    cap >> frame;

    //cv_image<rgb_pixel> cimg(frame);

    double fps = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - m_StartTime).count();
    m_StartTime = std::chrono::system_clock::now();
    cv::putText(frame, to_string(static_cast<int>(1000/fps)) + " FPS", cv::Point(10, 30), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 255), 1, false);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    //out.write(frame);

    //-----------------------------------Detect face -----------------------------
    
    if (RUNFACE_DETECTION == true) {
      std::cout << "Run face detection ...................." <<std::endl;
			//clone from original frame
			original = frame.clone();

			//convert image to gray scale and equalize
			cvtColor(original, graySacleFrame, COLOR_BGR2GRAY);
			//equalizeHist(graySacleFrame, graySacleFrame);

			//detect face in gray image
			face_cascade.detectMultiScale(graySacleFrame, faces, 1.1, 3, 0, cv::Size(90, 90));

			//number of faces detected
			//cout << faces.size() << " faces detected" << endl;
			std::string faceset = std::to_string(faces.size());

			int width = 0, height = 0;

			//region of interest
			//cv::Rect roi;

			for (int i = 0; i < faces.size(); i++)
			{
				//region of interest
				Rect face_i = faces[i];

				//crop the roi from grya image
				Mat face = graySacleFrame(face_i);

				//resizing the cropped image to suit to database image sizes
				Mat face_resized;
				cv::resize(face, face_resized, Size(img_width, img_height), 1.0, 1.0, INTER_CUBIC);

				//recognizing what faces detected
				int label=-1 ; double conf = 0;
				cv::imshow("ds", face_resized);
				model->predict(face_resized,label,conf);
				
				cout << " Label: " << label << "confident"<<conf<<endl;
				int temp = label / 10;
				if ( temp!= 77 )
				{
					Pname = "unknow";
				}
				/*else if (conf > 2000)
				{
					Pname = "unknow";
				}*/
				else
				{
					Pname ="object";
				}
				
				
				//drawing green rectagle in recognize face
				cv::rectangle(original, face_i, CV_RGB(0, 255, 0), 1);
				
				std::string text = Pname;

				int pos_x = std::max(face_i.tl().x - 10, 0);
				int pos_y = std::max(face_i.tl().y - 10, 0);
				cout << "x " << face_i.tl().x  <<"y "<< face_i.tl().y;
				//name the person who is in the image
				putText(original, text, Point(pos_x, pos_y), FONT_HERSHEY_COMPLEX_SMALL, 1.0, CV_RGB(0, 255, 0), 1.0);
				//cv::imwrite("E:/FDB/"+frameset+".jpg", cropImg);
				/*if (temp==77 ) {
					break;
				}*/
			}
			// cv::imshow("window", original);
			/*if (Pname == "object") {
				RUNFACE_DETECTION = false;
				// destroyWindow("window");
			}*/

			//putText(original, "Frames: " + frameset, Point(30, 60), FONT_HERSHEY_COMPLEX_SMALL, 1.0, CV_RGB(0, 255, 0), 1.0);
			//putText(original, "No. of Persons detected: " + to_string(faces.size()), Point(30, 90), FONT_HERSHEY_COMPLEX_SMALL, 1.0, CV_RGB(0, 255, 0), 1.0);
			//display to the winodw
			
			
				
			//cout << "model infor " << model->getDouble("threshold") << endl;
		}
    else if (RUNFACE_DETECTION == false && isInitTracking == false){
      std::cout << "Run person detection ...................." <<std::endl;
      auto recs = det.detectObject(frame);
      if (recs.empty()){
        // std:: cout<<"no person to detect"<<std::endl;
      }
      else{
      }	

    //-----------------------------------Detect face -----------------------------end

      //std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
      //std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
      //std::cout<<"Optical flow time: " << time_used.count() << "seconds" << std::endl;

      
      
      for(auto rec:recs)
      {
        // int w=cap.get(CV_CAP_PROP_FRAME_WIDTH);
        // std::cout<<W<<std::endl;
        //std:: cout<<rec.width<<std::endl;
        r = static_cast<Rect2d>(rec);
        
        // -----------------------------------TRACKING -----------------------------
        if(isInitTracking==false)   
          {
            //tracker.start_track(cimg, centered_rect(point(rec.x,rec.y), rec.width, rec.height));
            if(tracker->init(frame,r)==true)
              std::cout << "Init ...................."  <<std::endl;
            
            //cv::rectangle(frame, rec, cv::Scalar(0, 255, 0), 2);
            isInitTracking = true;
          }

        
        //-----------------------------------TRACKING -----------------------------END

        // cv::rectangle(frame, rec, cv::Scalar(0, 255, 0), 2, 1);

        //-----------------------------------DISTANCE -----------------------------
        // Distance(rec.width);
        
        //-----------------------------------CAMERA TRACKING -----------------------------
        // CameraTracking(rec.x, rec.width, pan, serial_port);
        // break;
        //-----------------------------------CAMERA TRACKING -----------------------------END
      }
    }
    if (isInitTracking == true){
  // std::cout << "Update "  <<std::endl;
  // auto started = std::chrono::high_resolution_clock::now();
  // tracker.update(cimg);
  // auto done = std::chrono::high_resolution_clock::now();
  // std::cout << "Time in milliseconds: "<<std::chrono::duration_cast<std::chrono::milliseconds>(done-started).count() << std::endl;
  // // std::cout << tracker.get_position() <<std::endl;
  // auto r = tracker.get_position();
  // startX = int(r.left());
  // startY = int(r.top());
  // endX = int(r.right());
  // endY = int(r.bottom());

  // win.set_image(cimg);
  // win.clear_overlay();
  // win.add_overlay(tracker.get_position());

  // cv::rectangle(frame, Point(startX, startY), Point(endX, endY), cv::Scalar(0, 255, 0), 2);

    // Mat frame_tracking;
    // cap >> frame_tracking;
    if(tracker->update(frame,r)==true)
      std::cout << "Update ...................."<< r.x <<std::endl;
    cv::rectangle(frame, r, cv::Scalar(0, 255, 0), 2, 1);
    // cv::imshow("tracking", frame_tracking);
    
  }
  
    cv::imshow("detection", frame);
    if(cv::waitKey(1) == 27)
    {
        break;
    }
  }
  cap.release();
  cv::destroyAllWindows() ;
  return 0;
}

