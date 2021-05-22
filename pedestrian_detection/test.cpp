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
// lib for face recognize
#include "src/config.h"
#include "src/anchor_creator.h"
#include "src/utils.h"
#include "benchmark.h"
using namespace std;
using namespace cv;
using namespace dlib;
using namespace cv::face;
// NEW

//***************************funtion  face detect and recognize****************************//
static int init_retinaface(ncnn::Net* retinaface, const int target_size)
{
    int ret = 0;
    // gpu
    
    ncnn::create_gpu_instance();
    retinaface->opt.use_vulkan_compute = 1;
    retinaface->opt.num_threads = 8;
    retinaface->opt.use_winograd_convolution = true;
    retinaface->opt.use_sgemm_convolution = true;

    const char* model_param = "../models/retinaface.param";
    const char* model_model = "../models/retinaface.bin";
    
    ret = retinaface->load_param(model_param);
    if(ret)
    {
        return ret;
    }
    ret = retinaface->load_model(model_model);
    if(ret)
    {
        return ret;
    }

    return 0;
}
static void deinit(ncnn::Net* retinaface,ncnn::Net* mbv2facenet){ 
    retinaface->clear();
    ncnn::destroy_gpu_instance();
    mbv2facenet->clear();
}
static int init_mbv2facenet(ncnn::Net* mbv2facenet, const int target_size)
{
    int ret = 0;
    // use gpu vulkan
    ncnn::create_gpu_instance();
    mbv2facenet->opt.use_vulkan_compute = 1;
    
    mbv2facenet->opt.num_threads = 8;
    mbv2facenet->opt.use_sgemm_convolution = 1;
    mbv2facenet->opt.use_winograd_convolution = 1;

    const char* model_param = "../models/mbv2facenet.param";
    const char* model_bin = "../models/mbv2facenet.bin";

    ret = mbv2facenet->load_param(model_param);
    if(ret)
    {
        return ret;
    }

    ret = mbv2facenet->load_model(model_bin);
    if(ret)
    {
        return ret;
    }

    return 0;
}

std::vector<cv::Rect> detect_retinaface(ncnn::Net* retinaface, cv::Mat img, const int target_size, std::vector<cv::Mat>& face_det)
{
    int img_w = img.cols;
    int img_h = img.rows;
    float tempo1= img_w*1.0 /target_size;
    //cout <<tempo1<<endl;
    float tempo2= img_h*1.0 /target_size;
    //cout<<img_w <<" "<<img_h<<endl;
    cv::Mat img1;
    std::vector<cv::Rect> bor;
    const float mean_vals[3] = {0, 0, 0};
    const float norm_vals[3] = {1, 1, 1};

    //ncnn::Mat input = ncnn::Mat::from_pixels(img.data, ncnn::Mat::PIXEL_BGR2RGB, img_w, img_h);
    ncnn::Mat input = ncnn::Mat::from_pixels_resize(img.data, ncnn::Mat::PIXEL_BGR2RGB,img_w, img_h, target_size, target_size);
    input.substract_mean_normalize(mean_vals, norm_vals);

    ncnn::Extractor ex = retinaface->create_extractor();
    ex.set_vulkan_compute(true);
    std::vector<AnchorCreator> ac(_feat_stride_fpn.size());
    for(size_t i = 0; i < _feat_stride_fpn.size(); i++)
    {
        int stride = _feat_stride_fpn[i];
        ac[i].init(stride, anchor_config[stride], false);
    }
    
    ex.input("data", input);

    std::vector<Anchor> proposals;

    for(size_t i = 0; i < _feat_stride_fpn.size(); i++)
    {
        ncnn::Mat cls;
        ncnn::Mat reg;
        ncnn::Mat pts;
        char cls_name[100];
        char reg_name[100];
        char pts_name[100];
        sprintf(cls_name, "face_rpn_cls_prob_reshape_stride%d", _feat_stride_fpn[i]);
        sprintf(reg_name, "face_rpn_bbox_pred_stride%d", _feat_stride_fpn[i]);
        sprintf(pts_name, "face_rpn_landmark_pred_stride%d", _feat_stride_fpn[i]);
        /*cls_name=(int)_feat_stride_fpn[i];
        reg_name = (int)_feat_stride_fpn[i];
        pts_name = (int)_feat_stride_fpn[i];*/
        //cout << _feat_stride_fpn[i]<<endl;
        ex.extract(cls_name,cls);
        ex.extract(reg_name,reg);
        ex.extract(pts_name,pts);

        /*printf("cls: %d %d %d\n", cls.c, cls.h, cls.w);
        printf("reg: %d %d %d\n", reg.c, reg.h, reg.w);
        printf("pts: %d %d %d\n", pts.c, pts.h, pts.w);*/

        ac[i].FilterAnchor(cls, reg, pts, proposals);

        /*for(size_t p = 0; p < proposals.size(); ++p)
        {
            proposals[p].print();
        }*/
    }

    std::vector<Anchor> finalres;
    box_nms_cpu(proposals, nms_threshold, finalres, target_size);
    cv::resize(img, img, cv::Size(target_size, target_size));
    for(size_t i = 0; i < finalres.size(); ++i)
    {
        finalres[i].print();//in thong so khuon mat xy-width-height-threadshold 300x300
        cv::Mat face = img(cv::Range((int)finalres[i].finalbox.y, (int)finalres[i].finalbox.height),cv::Range((int)finalres[i].finalbox.x, (int)finalres[i].finalbox.width)).clone();
        face_det.push_back(face);
        
        /*cv::rectangle(img, cv::Point((int)finalres[i].finalbox.x, (int)finalres[i].finalbox.y), cv::Point((int)finalres[i].finalbox.width, (int)finalres[i].finalbox.height),cv::Scalar(255,255,0), 2, 8, 0);*/
        float x=(float)finalres[i].finalbox.x*tempo1;
        float y= (float)finalres[i].finalbox.y*tempo2;
        float width =(float)finalres[i].finalbox.width*tempo1-x;
        float heigh =(float)finalres[i].finalbox.height*tempo2-y;
        //cout << "x,y "<<x<< " "<<y<<"width-height "<<width<<" "<<heigh<<endl;
        bor.push_back(cv::Rect((int)x,(int)y,(int)width,(int)heigh));
        /*for(size_t l = 0; l < finalres[i].pts.size(); ++l)
        {
            cv::circle(img, cv::Point((int)finalres[i].pts[l].x, (int)finalres[i].pts[l].y), 1, cv::Scalar(255, 255, 0), 2, 8, 0);
        }*/
    }
    return bor;
}

void run_mbv2facenet(ncnn::Net* mbv2facenet, std::vector<cv::Mat>& img, int target_size, std::vector<std::vector<float>>& res)
{
    for(size_t i = 0; i < img.size(); ++i)
    {
        ncnn::Extractor ex = mbv2facenet->create_extractor();
        //网络结构中的前两层已经做了归一化和均值处理， 在输入的时候不用处理了
        ex.set_vulkan_compute(true);
        ncnn::Mat input = ncnn::Mat::from_pixels_resize(img[i].data, ncnn::Mat::PIXEL_BGR2RGB, img[i].cols, img[i].rows, target_size, target_size);
        ex.input("data", input);
        
        ncnn::Mat feat;

        ex.extract("fc1", feat);

        //printf("c: %d h: %d w: %d\n", feat.c, feat.h, feat.w);
        std::vector<float> tmp;
        for(int i = 0; i < feat.w; ++i)
        {
            //printf("%f ", feat.channel(0)[i]);
            tmp.push_back(feat.channel(0)[i]);
        }
        res.push_back(tmp);
        //printf("\n");
    }
}   

//****************************************end func **************************************
int pan = 90;
int serial_port;

cv::Rect2d r;

cv::Point CalCenterObject(float x, float y, float width, float height)
{
  cv::Point point;
  point.x = x + width/2;
  point.y = y + height/2;
  return point;
}

int main( int argc, char** argv )
{
  // Config to Camera tracking
  // Open(serial_port);
  // Config_SerialPort(serial_port); 
  // LobotSerialServoMove(1, pan, 0, serial_port);

  //-----------------------------DETECT FACE-------------------------------
  //face recognizer 

  int target_size = 300;
  int facenet_size = 112;
  ncnn::Net retinaface;
  ncnn::Net mbv2facenet;  
  bool RUNFACE_DETECTION=true;
  int ret = 0;

  cv::Mat img1 = cv::imread("../test_pic/774.jpg", 1);
  ret = init_retinaface(&retinaface, target_size);
  if(ret)
  {
      cout<<"loi load model init_retinaface() "<<endl;
      return -1;
  }

  ret = init_mbv2facenet(&mbv2facenet, facenet_size);
  if(ret)
  {
      cout<<"loi load model nit_mbv2facenet() "<<endl;
      return -1;
  }
  std::vector<cv::Mat> face_det1;
  std::vector<std::vector<float>> feature_face1;
  auto box1=detect_retinaface(&retinaface, img1, target_size, face_det1); //pic1 dect
  //cv::imshow("out put detect",face_det1);
  run_mbv2facenet(&mbv2facenet, face_det1, facenet_size, feature_face1);
  //-----------------------------DETECT FACE-------------------------------END
  std::string pipeline = "v4l2src device=/dev/video0 ! image/jpeg, width=(int)640, height=(int)480, framerate=30/1 ! jpegdec ! videoconvert ! appsink";
  cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
  cv::Mat frame;

  ObjectDetection det("../../pedestrian_detection/");
  
  auto m_StartTime = std::chrono::system_clock::now();

  //correlation_tracker tracker(int filter_size = 6);

  cv::Mat cimg;

  bool isInitTracking = false;
  int startX, startY, endX, endY;
  
  Ptr<TrackerCSRT> tracker = TrackerCSRT::create();

  cv::Point center_face_detect;
  cv::Point center_person_detect;
  cv::Rect BoudBox_Target;

  // image_window win;

  while(true)
  {
    auto started = std::chrono::high_resolution_clock::now();
    cap >> frame;

    //cv_image<rgb_pixel> cimg(frame);

    double fps = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - m_StartTime).count();
    m_StartTime = std::chrono::system_clock::now();
    cv::putText(frame, to_string(static_cast<int>(1000/fps)) + " FPS", cv::Point(10, 30), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 255), 1, false);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    //out.write(frame);

    //-----------------------------------Detect face -----------------------------
    
    if (RUNFACE_DETECTION == true) {
      cv::Mat img2 = frame.clone();
      std::vector<cv::Mat> face_det;
      std::vector<std::vector<float>> feature_face;
      
      auto box2=detect_retinaface(&retinaface, img2, target_size, face_det);//pic2 dect
      if(face_det.size() >= 1) 
      {
        run_mbv2facenet(&mbv2facenet, face_det, facenet_size, feature_face);
        for(int i=0; i<int(feature_face.size());i++){
          float sim = calc_similarity_with_cos(feature_face1[0], feature_face[i]);
          cv::rectangle(frame,box2[i],cv::Scalar(255,255,0), 2, 8, 0);


          string text;
          if(sim >= 0.3)
          {
            text ="object";
            cv::putText(frame, text, cv::Point(box2[i].x - 10, box2[i].y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            
            // center_face_detect = CalCenterObject(box2[i].x, box2[i].y, box2[i].width, box2[i].height);
            // RUNFACE_DETECTION = false;
           
            // break;
          }
          else
          {
            text ="unknow";
            cv::putText(frame, text, cv::Point(box2[i].x - 10, box2[i].y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);
          }
          //string text1 = to_string(sim);
          
          //cv::imwrite("../output_pic/des1.jpg", img2);
            cout<<"Same ratio of Face: "<< sim<<endl;
        }
      }


			//putText(original, "Frames: " + frameset, Point(30, 60), FONT_HERSHEY_COMPLEX_SMALL, 1.0, CV_RGB(0, 255, 0), 1.0);
			//putText(original, "No. of Persons detected: " + to_string(faces.size()), Point(30, 90), FONT_HERSHEY_COMPLEX_SMALL, 1.0, CV_RGB(0, 255, 0), 1.0);
			//display to the winod
			
		}
    else if (RUNFACE_DETECTION == false && isInitTracking == false){
      std::cout << "Run person detection ...................." <<std::endl;
      auto recs = det.detectObject(frame);
      if (recs.empty()){
        // std:: cout<<"no person to detect"<<std::endl;
      }
      else{
        
      //std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
      //std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
      //std::cout<<"Optical flow time: " << time_used.count() << "seconds" << std::endl;

      float dis_Face_Person = 1000;
      
      for(auto rec:recs)
      {
        center_person_detect = CalCenterObject(rec.x, rec.y, rec.width, rec.height);
        if (abs(center_person_detect.x - center_face_detect.x) < dis_Face_Person){
          dis_Face_Person = abs(center_person_detect.x - center_face_detect.x);
          BoudBox_Target = rec;
          
        }
        r = static_cast<Rect2d>(rec);
        
        // -----------------------------------TRACKING -----------------------------
        // if(isInitTracking==false)   
        //   {
        //     //tracker.start_track(cimg, centered_rect(point(rec.x,rec.y), rec.width, rec.height));
        //     if(tracker->init(frame,r)==true)
        //       std::cout << "Init ...................."  <<std::endl;
            
        //     //cv::rectangle(frame, rec, cv::Scalar(0, 255, 0), 2);
        //     isInitTracking = true;
        //   }

        
        //-----------------------------------TRACKING -----------------------------END

        // cv::rectangle(frame, rec, cv::Scalar(0, 255, 0), 2, 1);

        //-----------------------------------DISTANCE -----------------------------
        // Distance(rec.width);
        
        //-----------------------------------CAMERA TRACKING -----------------------------
        // CameraTracking(rec.x, rec.width, pan, serial_port);
        // break;
        //-----------------------------------CAMERA TRACKING -----------------------------END
      }
        cv::rectangle(frame, BoudBox_Target, cv::Scalar(0, 255, 0), 2);

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
   
    //tracker->update(frame,r);
    
    cv::rectangle(frame, r, cv::Scalar(0, 255, 0), 2, 1);
    // cv::imshow("tracking", frame_tracking);
    
  }
  
    cv::imshow("detection", frame);
    auto done = std::chrono::high_resolution_clock::now();
    //std::cout << "Time in milliseconds: "<<std::chrono::duration_cast<std::chrono::milliseconds>(done-started).count() << std::endl;
    if(cv::waitKey(1) == 27)
    {
        break;
    }
  }
  cap.release();
  cv::destroyAllWindows() ;
  deinit(&retinaface,&mbv2facenet);
  return 0;
}
