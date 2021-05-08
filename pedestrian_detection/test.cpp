#include "/home/jetson/Desktop/person_detection/pedestrian_detection/object_detection.h"
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <opencv4/opencv2/tracking/tracker.hpp>

#include <math.h>

#include <chrono>
#include <iostream>
using namespace std;
using namespace cv;
// NEW

/// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <stdint.h>
#define GET_LOW_BYTE(A) (uint8_t)((A))
//Macro function  get lower 8 bits of A
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
//Macro function  get higher 8 bits of A
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//put A as higher 8 bits   B as lower 8 bits   which amalgamated into 16 bits integer

#define LOBOT_SERVO_FRAME_HEADER         0x55
#define LOBOT_SERVO_MOVE_TIME_WRITE      1
#define LOBOT_SERVO_MOVE_TIME_READ       2
#define LOBOT_SERVO_MOVE_TIME_WAIT_WRITE 7
#define LOBOT_SERVO_MOVE_TIME_WAIT_READ  8
#define LOBOT_SERVO_MOVE_START           11
#define LOBOT_SERVO_MOVE_STOP            12
#define LOBOT_SERVO_ID_WRITE             13
#define LOBOT_SERVO_ID_READ              14
#define LOBOT_SERVO_ANGLE_OFFSET_ADJUST  17
#define LOBOT_SERVO_ANGLE_OFFSET_WRITE   18
#define LOBOT_SERVO_ANGLE_OFFSET_READ    19
#define LOBOT_SERVO_ANGLE_LIMIT_WRITE    20
#define LOBOT_SERVO_ANGLE_LIMIT_READ     21
#define LOBOT_SERVO_VIN_LIMIT_WRITE      22
#define LOBOT_SERVO_VIN_LIMIT_READ       23
#define LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define LOBOT_SERVO_TEMP_MAX_LIMIT_READ  25
#define LOBOT_SERVO_TEMP_READ            26
#define LOBOT_SERVO_VIN_READ             27
#define LOBOT_SERVO_POS_READ             28
#define LOBOT_SERVO_OR_MOTOR_MODE_WRITE  29
#define LOBOT_SERVO_OR_MOTOR_MODE_READ   30
#define LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define LOBOT_SERVO_LOAD_OR_UNLOAD_READ  32
#define LOBOT_SERVO_LED_CTRL_WRITE       33
#define LOBOT_SERVO_LED_CTRL_READ        34
#define LOBOT_SERVO_LED_ERROR_WRITE      35
#define LOBOT_SERVO_LED_ERROR_READ       36
#define Baud_rate B115200
const char *address_tty ="/dev/ttyUSB0";
typedef unsigned char   byte;
struct termios tty;
int serial_port;

byte LobotCheckSum(byte buf[])
{
  byte i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  temp = ~temp;
  i = (byte)temp;
  return i;
}

void LobotSerialServoMove(uint8_t id, int16_t angle, uint16_t time, int serial_port)
{
  int position = int(angle*25/6);
  byte buf[10];
  if(position < 0)
    position = 0;
  if(position > 1000)
    position = 1000;
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(position);
  buf[6] = GET_HIGH_BYTE(position);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = LobotCheckSum(buf);
  write(serial_port, buf, 10);
}

// Open
void Open(){
  serial_port = open(address_tty, O_RDWR);
  // Check for errors
  if (serial_port < 0) {
      printf("Error %i from open: %s\n", errno, strerror(errno));
  }
}

void Config_SerialPort(int serial_port){
  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      // return 1;
      exit(1);
  }
  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, Baud_rate);
  cfsetospeed(&tty, Baud_rate);
  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      exit(1);
  }
}
// 
void Distance(int knowWidth)
{
  int focalLength = (864/2)*sqrt(3);
  // int realheight = 1750;
  // int imageheight = 720;

  // int objectheight = rec.height;
  // int sensorheight = 10;
  
  // cout << "Distance = " << (focalLength*realheight*imageheight)/(objectheight*sensorheight) << endl;
  
  cout << "Distance = " << 60*focalLength/knowWidth << endl;
  cout << endl;
}


void CameraTracking(int recX, int recWidth, int &pan)
{
  int objX = recX + recWidth/2; 
  int errorPan=objX-1280/2;
  if(abs(errorPan)>15)
    pan = pan - errorPan/75;
  
  if(pan>180)
  {
    pan = 180;
    // std:: cout<<"pan out of range "<<std::endl;
  }
  if(pan<0)
  {
    pan = 0;
    // std:: cout<<"pan out of range "<<std::endl;
  }
  std:: cout<<"pan"<<pan<<std::endl;
  LobotSerialServoMove(1, pan, 0, serial_port);
}

int pan = 90;

int main( int argc, char** argv )
{
  // Config to Camera tracking
  // Open();
  // Config_SerialPort(serial_port); 
  // LobotSerialServoMove(1, pan, 0, serial_port);

  std::string pipeline = "v4l2src device=/dev/video0 ! image/jpeg, width=(int)1280, height=(int)720, framerate=30/1 ! jpegdec ! videoconvert ! appsink";
  std::string wriweb= "appsrc ! videoconvert ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! jpegenc ! rtpjpegpay ! udpsink host=127.0.0.1 port=5000";
  cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
  cv::Mat frame;
  cv::VideoWriter out(wriweb, 0,	 
  30,
  cv::Size(640, 480), 
  true);
  ObjectDetection det("../../pedestrian_detection/");

  cv::Ptr<cv::Tracker> tracker = cv::TrackerMOSSE::create();
  
  auto m_StartTime = std::chrono::system_clock::now();
  
  // Setup the termination criteria, either 10 iteration or move by atleast 1 pt
  TermCriteria term_crit(TermCriteria::EPS | TermCriteria::COUNT, 10, 1);


  cv::Rect2d r;

  while(true)
  {
    cap >> frame;
    double fps = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - m_StartTime).count();
    m_StartTime = std::chrono::system_clock::now();
    cv::putText(frame, to_string(static_cast<int>(1000/fps)) + " FPS", cv::Point(10, 30), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 255), 1, false);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    //out.write(frame);       
    auto recs = det.detectObject(frame);
    if (recs.empty()){
      // std:: cout<<"no person to detect"<<std::endl;
    }
    else{
    }	
    //std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    //std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
    //std::cout<<"Optical flow time: " << time_used.count() << "seconds" << std::endl;

    bool isInit = false;
    
    for(auto rec:recs)
    {
      // int w=cap.get(CV_CAP_PROP_FRAME_WIDTH);
      // std::cout<<W<<std::endl;
      //std:: cout<<rec.width<<std::endl;

      r = static_cast<cv::Rect2d>(rec);

      


      // if(isInit==false)   

      
      //   if(tracker->init(frame, r)==true)
      //   {
      //     std::cout << "Init ...................."  <<std::endl;
      //     isInit = true;

          
      //     // try{ 
      //     //   if(tracker->update(frame, r)==true)
      //     //     std::cout << "Tracking... "  <<std::endl;
      //     //     throw 20;
      //     //     }
      //     // catch(int e){
      //     //   std::cout << "Tracking... faile "  <<std::endl;
      //     // }
      //   }
      // else{
      //   std::cout << "Update "  <<std::endl;
      //   std::cout<<frame.channels()<<std::endl;
      //   if(tracker->update(frame, r)==true)
      //     std::cout << "Tracking... "  <<std::endl;
      // }
      
      

      cv::rectangle(frame, rec, cv::Scalar(0, 255, 0), 2, 1);

      // Distance(rec.width);
      

      //Tracking
      // CameraTracking(rec.x, rec.width, pan);
      // break;
      //..........
    }
    std::cout << "Exit "  <<std::endl;
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
