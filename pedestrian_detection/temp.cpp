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
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include <fstream>
#include <sstream>
#include <chrono>
#include <iostream>
// lib for face recognize
#include "src/config.h"
#include "src/anchor_creator.h"
#include "src/utils.h"
#include "benchmark.h"
// lib for tracker
#include "src/kcftracker.hpp"
#include <dirent.h>
using namespace std;
using namespace cv;
using namespace cv::face;
using namespace cv::xfeatures2d;
// move
#define MoveForward GPIO::LOW
#define MoveBackward GPIO::HIGH

#define MaxRight 100
#define MaxLeft 95

const int right_output_pwm_pin1 = 33; //pwm
const int right_output_pin2 = 31; //


const int left_output_pin_1 = 29; // 
const int left_output_pwm_pin_2 = 32; // pwm
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
        //?????????????????????????????????????????????????????????????????? ?????????????????????????????????
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

cv::Point CalCenterObject(cv::Rect r)
{
  cv::Point point;
  point.x = r.x + r.width/2;
  point.y = r.y + r.height/2;
  return point;
}

int main( int argc, char** argv )
{
  // Config to Camera tracking
  //Open(serial_port);
  //Config_SerialPort(serial_port); 
  //LobotSerialServoMove(1, pan, 0, serial_port);
  //Move

  GPIO::setwarnings(false);
	// Pin Setup.
	// Board pin-numbering scheme
	GPIO::setmode(GPIO::BOARD);

	// set pin as an output pin with optional initial state of HIGH
	GPIO::setup(right_output_pwm_pin1, GPIO::OUT, GPIO::HIGH);
	GPIO::setup(right_output_pin2, GPIO::OUT, MoveForward);
	GPIO::PWM Right_pwm(right_output_pwm_pin1, 50);

	GPIO::setup(left_output_pwm_pin_2, GPIO::OUT, GPIO::HIGH);
	GPIO::setup(left_output_pin_1, GPIO::OUT, MoveForward);
  GPIO::PWM Left_pwm(left_output_pwm_pin_2, 50);
  //-----------------------------DETECT FACE-------------------------------
  //face recognizer 

  int target_size = 300;
  int facenet_size = 112;
  ncnn::Net retinaface;
  ncnn::Net mbv2facenet;  
  bool RUNFACE_DETECTION=true;
  int ret = 0;

  cv::Mat img1 = cv::imread("../test_pic/336.jpg", 1);
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

  bool isInitTracking = false;

  //config tracker
  bool HOG = true;
	bool FIXEDWINDOW = false;
	bool MULTISCALE = true;
	bool LAB = false;
  // Create KCFTracker object
	KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

  cv::Point center_face_detect;
  cv::Point center_person_detect;
  cv::Rect BoudBox_Target;
  cv::Rect BoudBox_Tracking;
  cv::Mat cropTarget_1;

  int nFrame = 0;

  int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
  int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  // Define the codec and create VideoWriter object.The output is stored in 'outcpp.avi' file.
  VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, Size(frame_width,frame_height));
  while(true)
  {

    cap >> frame;
    


    double fps = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - m_StartTime).count();
    m_StartTime = std::chrono::system_clock::now();
    cv::putText(frame, to_string(static_cast<int>(1000/fps)) + " FPS", cv::Point(10, 30), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 255), 1, false);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
   

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
            center_face_detect = CalCenterObject(box2[i]);
            RUNFACE_DETECTION = false;

            break;
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
        // cv::rectangle(frame, rec, cv::Scalar(0, 255, 0), 2);

        center_person_detect = CalCenterObject(rec);
        if (abs(center_person_detect.x - center_face_detect.x) < dis_Face_Person){
          dis_Face_Person = abs(center_person_detect.x - center_face_detect.x);
          BoudBox_Target = rec;
        }
       

        // cv::rectangle(frame, rec, cv::Scalar(0, 255, 0), 2, 1);

        //-----------------------------------DISTANCE -----------------------------
        // Distance(rec.width, frame_width);
        

      }
        if(!BoudBox_Target.empty())
          {
            // cout<<BoudBox_Target.size()<<endl;
            tracker.init(BoudBox_Target, frame);
          }
        // cv::rectangle(frame, BoudBox_Target, cv::Scalar(0, 255, 0), 2);
        if(0 <= BoudBox_Target.x
              && 0 <= BoudBox_Target.width
              && BoudBox_Target.x + BoudBox_Target.width <= frame.cols
              && 0 <= BoudBox_Target.y
              && 0 <= BoudBox_Target.height
              && BoudBox_Target.y + BoudBox_Target.height <= frame.rows)
        cropTarget_1 = frame(BoudBox_Target);

        isInitTracking = true;
    }	
    
    }
    if(isInitTracking == true && nFrame == 10)
    {
      nFrame = 0;
      auto recs = det.detectObject(frame);

      if (recs.empty()){
        std:: cout<<"no person to detect"<<std::endl;
      }
      else{
        // std::cout << "Update target ...................." <<std::endl;
        float dis_person = 1000;
        float cen_box_ex = (CalCenterObject(BoudBox_Target)).x;
        for(auto rec:recs){
          // if (0 <= rec.x
          //     && 0 <= rec.width
          //     && rec.x + rec.width <= frame.cols
          //     && 0 <= rec.y
          //     && 0 <= rec.height
          //     && rec.y + rec.height <= frame.rows && !cropTarget_1.empty()){
          // cv::Mat imageTest = frame(rec);
          // if(!imageTest.empty()){
          // //frame(rec).copyTo(imageTest);
          // //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
          // int minHessian = 400;
          // Ptr<SURF> detector = SURF::create( minHessian );
          // std::vector<KeyPoint> keypoints1, keypoints2;
          // Mat descriptors1, descriptors2;
          // detector->detectAndCompute( cropTarget_1, noArray(), keypoints1, descriptors1 );
          // detector->detectAndCompute( imageTest, noArray(), keypoints2, descriptors2 );
          // //-- Step 2: Matching descriptor vectors with a FLANN based matcher
          // // Since SURF is a floating-point descriptor NORM_L2 is used
          // Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
          // std::vector< std::vector<DMatch> > knn_matches;
          // matcher->knnMatch( descriptors1, descriptors2, knn_matches, 2 );
          // //-- Filter matches using the Lowe's ratio test
          // const float ratio_thresh = 0.7f;
          // std::vector<DMatch> good_matches;
          // for (size_t i = 0; i < knn_matches.size(); i++)
          // {
          // if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
          //   {
          //     good_matches.push_back(knn_matches[i][0]);
          //   }
          // } 
          // cout<<"Diem giong: "<<good_matches.size()<<endl;
          // if(good_matches.size() > 30){
          //   BoudBox_Target = rec;
          //   tracker.init(BoudBox_Target, frame);
          // }
          // string a = std::to_string(good_matches.size());
          //cv::putText(frame, a, cv::Point(BoudBox_Target.x, BoudBox_Target.y),
          //cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);

          // }
          float dis_cal = abs((CalCenterObject(rec)).x - cen_box_ex);
          if(dis_cal < dis_person){
            dis_person = dis_cal;
            BoudBox_Target = rec;
          }
          // std::cout << "Update done ...................." <<std::endl;
          tracker.init(BoudBox_Target, frame);
          // std::cout << "Update done ...................." <<std::endl;
          BoudBox_Tracking = tracker.update(frame);
          cv::rectangle(frame, BoudBox_Tracking, cv::Scalar(0, 255, 0), 2, 1);
        //}
        }
      }
    }

    else if (isInitTracking == true){
      // std::cout << "Tracking............." <<std::endl;
      nFrame = nFrame + 1;
      BoudBox_Tracking = tracker.update(frame);
      // std::cout << "Tracking done............." <<std::endl;
      cv::rectangle(frame, BoudBox_Tracking, cv::Scalar(0, 0, 255), 2, 1);

      // if(nFrame == 14 && 0 <= BoudBox_Tracking.x
      //         && 0 <= BoudBox_Tracking.width
      //         && BoudBox_Tracking.x + BoudBox_Tracking.width <= frame.cols
      //         && 0 <= BoudBox_Tracking.y
      //         && 0 <= BoudBox_Tracking.height
      //         && BoudBox_Tracking.y + BoudBox_Tracking.height <= frame.rows)
      // cropTarget_1 = frame(BoudBox_Tracking);
      //-----------------------------------CAMERA TRACKING -----------------------------
      //CameraTracking(frame_width, BoudBox_Tracking.x, BoudBox_Tracking.width, pan, serial_port);
      // break;
      //-----------------------------------CAMERA TRACKING -----------------------------END
      
      int distance = Distance(BoudBox_Tracking.width, BoudBox_Tracking.height, frame_width);
      string distancee = std::to_string(distance);
      cv::putText(frame, distancee, cv::Point(BoudBox_Target.x, BoudBox_Target.y),
      cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);
      int objX =  BoudBox_Tracking.x + BoudBox_Tracking.width/2; 
      int errorPan = objX - frame_width/2;
      if(distance > 220)
      {
      if(abs(errorPan) > 100)
      {
        if(errorPan > 0) //Taget in right of frame 
          {
            Right_pwm.stop();
            GPIO::output(left_output_pin_1, MoveForward);

            Left_pwm.start(MaxLeft-50);
          }
        else       //Taget in left of frame
          {
            Left_pwm.stop();
            GPIO::output(right_output_pin2, MoveForward);
            Right_pwm.start(MaxRight-50);
          }
      }
      else {

        // if(distance < 180)
        //   {
        //     GPIO::output(left_output_pin_1, MoveBackward);
        //     GPIO::output(right_output_pin2, MoveBackward);
        //     Right_pwm.start(MaxRight);
        //     Left_pwm.start(MaxLeft);
        //   }
        // if(distance < 220)
        // {
        //   Right_pwm.stop();
        //   Left_pwm.stop();
        // }
        // else{
          GPIO::output(left_output_pin_1, GPIO::HIGH);
          GPIO::output(right_output_pin2, GPIO::HIGH);
          Right_pwm.start(MaxRight);
          Left_pwm.start(MaxLeft - 10);
        // }
      }
      }
      else if(180 < distance && distance < 220)
      {
        Right_pwm.stop();
        Left_pwm.stop();
      }
      else
      {
        cout<<"Lui"<<endl;
        GPIO::output(left_output_pin_1, GPIO::LOW);
        Left_pwm.start(MaxLeft);
        GPIO::output(right_output_pin2, GPIO::LOW);
        Right_pwm.start(MaxRight);
        
      }
      // if(distance < 220){
      // //CameraTracking(frame_width, BoudBox_Tracking.x, BoudBox_Tracking.width, pan, serial_port);

      //   // Right_pwm.stop();
      //   // Left_pwm.stop();
      // }
      // else{
      //   //Move-------------------------------------------------------------------------------

	
      // if(abs(errorPan) > 100)
      // {
      //   if(errorPan > 0) //Taget in right of frame 
      //     {
      //       Right_pwm.stop();
      //       GPIO::output(left_output_pin_1, MoveForward);

      //       Left_pwm.start(MaxLeft-50);
      //     }
      //   else       //Taget in left of frame
      //     {
      //       Left_pwm.stop();
      //       GPIO::output(right_output_pin2, MoveForward);
      //       Right_pwm.start(MaxRight-50);
      //     }
      // }
      // else{
      //   Right_pwm.start(MaxRight-50);
      //   Left_pwm.start(MaxRight-50);
      // }
      //  //Move end------------------------------------------------------------------------
      // }
    }
  
    cv::imshow("detection", frame);
    // video.write(frame);
    if(cv::waitKey(1) == 27)
    {
        break;
    }
  }
  Right_pwm.stop();
  Left_pwm.stop();

  GPIO::cleanup(29);
	GPIO::cleanup(31);
	GPIO::cleanup(32);
	GPIO::cleanup(33);

  cap.release();
  // video.release();
  cv::destroyAllWindows();
  deinit(&retinaface,&mbv2facenet);
  return 0;
}

