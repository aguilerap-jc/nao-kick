/**
 * Copyright (c) 2011 Aldebaran Robotics
 */

#include "bumper.h"

#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <qi/log.hpp>
#include <althread/alcriticalsection.h>

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alproxies/alledsproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <alproxies/allandmarkdetectionproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/albehaviormanagerproxy.h>


using namespace std;
using namespace cv;
using namespace AL;

int contL=0;
int contR=0;
bool BUMPERIZQUIERDO=false;

void kickCatarino();
void rutinaApagado();

//AL::ALVideoDeviceProxy camProxy;
//const std::string clientName = camProxy.subscribe("test", AL::kQVGA, AL::kBGRColorSpace, 30);


Bumper::Bumper(
  boost::shared_ptr<AL::ALBroker> broker,
  const std::string& name): AL::ALModule(broker, name),
    fCallbackMutex(AL::ALMutex::createALMutex())
{
  setModuleDescription("This module presents how to subscribe to a simple event (here RightBumperPressed) and use a callback method.");

  functionName("onRightBumperPressed", getName(), "Method called when the right bumper is pressed. Makes a LED animation.");
  BIND_METHOD(Bumper::onRightBumperPressed)

  functionName("onLeftBumperPressed", getName(), "Method called when the left bumper is pressed. Makes a LED animation.");
  BIND_METHOD(Bumper::onLeftBumperPressed)
}

Bumper::~Bumper() {
  fMemoryProxy.unsubscribeToEvent("onRightBumperPressed", "Bumper");
  fMemoryProxy2.unsubscribeToEvent("onLeftBumperPressed", "Bumper");
}

void Bumper::init() {
  try {
    /** Create a proxy to ALMemory.
    */
    fMemoryProxy = AL::ALMemoryProxy(getParentBroker());
    fMemoryProxy2= AL::ALMemoryProxy(getParentBroker());

    fState = fMemoryProxy.getData("RightBumperPressed");
    fState2= fMemoryProxy2.getData("LeftBumperPressed");

    /** Subscribe to event LeftBumperPressed
    * Arguments:
    * - name of the event
    * - name of the module to be called for the callback
    * - name of the bound method to be called on event
    */
    fMemoryProxy.subscribeToEvent("RightBumperPressed", "Bumper",
                                   "onRightBumperPressed");
    fMemoryProxy2.subscribeToEvent("LeftBumperPressed", "Bumper",
                                   "onLeftBumperPressed");
    }
  catch (const AL::ALError& e) {
    qiLogError("module.example") << e.what() << std::endl;
  }
}

void Bumper::onRightBumperPressed() {
  qiLogInfo("module.example") << "Executing callback method on right bumper event" << std::endl;
  /**
  * As long as this is defined, the code is thread-safe.
  */
  //AL::ALCriticalSection section(fCallbackMutex);

  /**
  * Check that the bumper is pressed.
  */
  fState =  fMemoryProxy.getData("RightBumperPressed");

  if (fState  > 0.5f) {
    //return;
  }
  try {
    fTtsProxy = AL::ALTextToSpeechProxy(getParentBroker());
    fTtsProxy.say("Right bumper pressed");
    contR++;
    if(BUMPERIZQUIERDO==false && contR==1){
    kickCatarino();
    }
  }
  catch (const AL::ALError& e) {
    qiLogError("module.example") << e.what() << std::endl;
  }
}

void Bumper::onLeftBumperPressed(){

    qiLogInfo("module.example") << "Executing callback method on left bumper event" << std::endl;
    /**
    * As long as this is defined, the code is thread-safe.
    */
    //AL::ALCriticalSection section(fCallbackMutex);

    /**
    * Check that the bumper is pressed.
    */

    fState2=  fMemoryProxy2.getData("LeftBumperPressed");
    cout<<fMemoryProxy2.getData("LeftBumperPressed");
    if (fState2 >0.5f){
      //return;
    }
    try {
      fTtsProxy = AL::ALTextToSpeechProxy(getParentBroker());
      //fTtsProxy.say("Left bumper pressed");
      //cout<<"AL::ALModule::exit();"<<endl;
      contL++;
      cout<<"Contador = "<<contL<<endl;
      if(contL==10){
          BUMPERIZQUIERDO=true;
          rutinaApagado();
          fMemoryProxy.unsubscribeToEvent("onRightBumperPressed","Bumper");
          fMemoryProxy2.unsubscribeToEvent("onLeftBumperPressed","Bumper");
          cout<<"Unsubscribed"<<endl;
        }
    }
    catch (const AL::ALError& e) {
      qiLogError("module.example") << e.what() << std::endl;
    }
}

void kickCatarino(){
        AL::ALRobotPostureProxy posture;
        AL::ALMotionProxy motion;
        ALTextToSpeechProxy say;
        std::vector<std::string> names;
        AL::ALValue times, keys;
        AL::ALVideoDeviceProxy camProxy;
        const std::string clientName = camProxy.subscribe("test2", AL::kQVGA, AL::kBGRColorSpace, 30);

        bool flag = true;
        string postura;
        posture.goToPosture("StandInit", 0.5);
        //get robot Frame
        camProxy.setActiveCamera(1); //conect to bottom camera
        camProxy.setResolution("test2", 1);
        //Color para el filtro
        int iLowH = 0;
        int iHighH = 10;
        int iLowS = 10;
        int iHighS = 255;
        int iLowV =10;
        int iHighV = 255;

        int contFrames = 0;
        char key = 'a';
        Mat frame;
        //int umbralRosa = 220;
        bool robotDetectedIzq = false;
        bool robotDetectedDer = false;

        Mat imgHSV;
        Mat imgThresholded;
        int posX = 0;
        int posY = 0;
        motion.angleInterpolation("HeadPitch", (20.5*M_PI)/180, 1, true);
        motion.moveTo(0.10,0,0);
        while (key != 27 && flag)
        {
            //Obtienes la imagen de la camara
            Mat imgHeader = cv::Mat(cv::Size(320, 240), CV_8UC3);

            ALValue img = camProxy.getImageRemote(clientName);

            imgHeader.data = (uchar*) img[6].GetBinary();
            camProxy.releaseImage(clientName);
            frame = imgHeader.clone();

            if(contFrames == 4) {
                robotDetectedIzq = false;
                robotDetectedDer = false;
                contFrames = 0;
                    //motion.angleInterpolation("HeadYaw", (0*M_PI)/180, 1, true);
                    cvtColor(frame, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
                    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
                    //morphological opening (removes small objects from the foreground)
                    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
                    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

                    //morphological closing (removes small holes from the foreground)
                    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
                    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

                    //imshow("Thresholded Image", imgThresholded); //show the thresholded image
                    //imshow("Src", frame); //show the original image
                    //waitKey(50);

                    Moments oMoments = moments(imgThresholded);
                    double dM01 = oMoments.m01;
                    double dM10 = oMoments.m10;
                    double dArea = oMoments.m00;
                    if (dArea > 10000)
                    {
                    //calculate the position of the ball
                    posX = dM10 / dArea;
                    posY = dM01 / dArea;

                    cout << "pos X = " << posX << " pos Y = " << posY << endl;
                    if(posY < 160) { // catarino y sheldon con 165, lisa con 160
                        motion.moveTo(0.05,0,0); // catarino y sheldon con .05 y lisa con .04
                    }
                    if(posX > 190){
                        motion.moveTo(0, -0.03, 0);
                    }
                    if(posX < 170){ // 170 c y s, 175 a.
                        motion.moveTo(0, 0.03, 0);
                    }
                    if(posY> 160 && posX < 190 && posX > 170){
                        motion.moveTo(0,0.02,-0.08);
                        say.setVolume(1);
                        say.say("KICK");
                        names.reserve(22);
                        times.arraySetSize(22);
                        keys.arraySetSize(22);
                        names.push_back("HeadPitch");
                        times[0].arraySetSize(5);
                        keys[0].arraySetSize(5);

                        times[0][0] = 0.440000;
                        keys[0][0] = AL::ALValue::array(0.0245020, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[0][1] = 1.20000;
                        keys[0][1] = AL::ALValue::array(0.0214340, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[0][2] = 1.96000;
                        keys[0][2] = AL::ALValue::array(0.0229680, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.280000, 0.00000));
                        times[0][3] = 2.80000;
                        keys[0][3] = AL::ALValue::array(0.0214340, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                        times[0][4] = 3.04000;
                        keys[0][4] = AL::ALValue::array(0.0229680, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                        names.push_back("HeadYaw");
                        times[1].arraySetSize(5);
                        keys[1].arraySetSize(5);

                        times[1][0] = 0.440000;
                        keys[1][0] = AL::ALValue::array(-0.00617796, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[1][1] = 1.20000;
                        keys[1][1] = AL::ALValue::array(-0.00924597, AL::ALValue::array(3, -0.253333, 0.000766999), AL::ALValue::array(3, 0.253333, -0.000766999));
                        times[1][2] = 1.96000;
                        keys[1][2] = AL::ALValue::array(-0.0107800, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.280000, 0.00000));
                        times[1][3] = 2.80000;
                        keys[1][3] = AL::ALValue::array(-0.00310997, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                        times[1][4] = 3.04000;
                        keys[1][4] = AL::ALValue::array(-0.0107800, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                        names.push_back("LAnklePitch");
                        times[2].arraySetSize(5);
                        keys[2].arraySetSize(5);

                        times[2][0] = 0.440000;
                        keys[2][0] = AL::ALValue::array(-0.408086, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[2][1] = 1.20000;
                        keys[2][1] = AL::ALValue::array(-0.567621, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[2][2] = 1.96000;
                        keys[2][2] = AL::ALValue::array(-0.567621, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.280000, 0.00000));
                        times[2][3] = 2.80000;
                        keys[2][3] = AL::ALValue::array(-0.567621, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                        times[2][4] = 3.04000;
                        keys[2][4] = AL::ALValue::array(-0.576826, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                        names.push_back("LAnkleRoll");
                        times[3].arraySetSize(5);
                        keys[3].arraySetSize(5);

                        times[3][0] = 0.440000;
                        keys[3][0] = AL::ALValue::array(-0.00302603, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[3][1] = 1.20000;
                        keys[3][1] = AL::ALValue::array(0.0521979, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[3][2] = 1.96000;
                        keys[3][2] = AL::ALValue::array(0.0521979, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.280000, 0.00000));
                        times[3][3] = 2.80000;
                        keys[3][3] = AL::ALValue::array(0.0521979, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                        times[3][4] = 3.04000;
                        keys[3][4] = AL::ALValue::array(0.0521980, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                        names.push_back("LElbowRoll");
                        times[4].arraySetSize(5);
                        keys[4].arraySetSize(5);

                        times[4][0] = 0.440000;
                        keys[4][0] = AL::ALValue::array(-0.00872665, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[4][1] = 1.20000;
                        keys[4][1] = AL::ALValue::array(-0.0306380, AL::ALValue::array(3, -0.253333, 0.00441891), AL::ALValue::array(3, 0.253333, -0.00441891));
                        times[4][2] = 1.96000;
                        keys[4][2] = AL::ALValue::array(-0.0352401, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.280000, 0.00000));
                        times[4][3] = 2.80000;
                        keys[4][3] = AL::ALValue::array(-0.0306380, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                        times[4][4] = 3.04000;
                        keys[4][4] = AL::ALValue::array(-0.0352400, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                        names.push_back("LElbowYaw");
                        times[5].arraySetSize(5);
                        keys[5].arraySetSize(5);

                        times[5][0] = 0.440000;
                        keys[5][0] = AL::ALValue::array(-1.37451, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[5][1] = 1.20000;
                        keys[5][1] = AL::ALValue::array(-1.37757, AL::ALValue::array(3, -0.253333, 0.00102276), AL::ALValue::array(3, 0.253333, -0.00102276));
                        times[5][2] = 1.96000;
                        keys[5][2] = AL::ALValue::array(-1.38064, AL::ALValue::array(3, -0.253333, 0.000728725), AL::ALValue::array(3, 0.280000, -0.000805433));
                        times[5][3] = 2.80000;
                        keys[5][3] = AL::ALValue::array(-1.38218, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                        times[5][4] = 3.04000;
                        keys[5][4] = AL::ALValue::array(-1.38064, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                        names.push_back("LHipPitch");
                        times[6].arraySetSize(5);
                        keys[6].arraySetSize(5);

                        times[6][0] = 0.440000;
                        keys[6][0] = AL::ALValue::array(-0.464760, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[6][1] = 1.20000;
                        keys[6][1] = AL::ALValue::array(-0.618161, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[6][2] = 1.96000;
                        keys[6][2] = AL::ALValue::array(-0.616627, AL::ALValue::array(3, -0.253333, -0.000485813), AL::ALValue::array(3, 0.280000, 0.000536952));
                        times[6][3] = 2.80000;
                        keys[6][3] = AL::ALValue::array(-0.615092, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                        times[6][4] = 3.04000;
                        keys[6][4] = AL::ALValue::array(-0.622762, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                        names.push_back("LHipRoll");
                        times[7].arraySetSize(5);
                        keys[7].arraySetSize(5);

                        times[7][0] = 0.440000;
                        keys[7][0] = AL::ALValue::array(4.19617e-05, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[7][1] = 1.20000;
                        keys[7][1] = AL::ALValue::array(0.493989, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[7][2] = 1.96000;
                        keys[7][2] = AL::ALValue::array(0.493989, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.280000, 0.00000));
                        times[7][3] = 2.80000;
                        keys[7][3] = AL::ALValue::array(0.492455, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                        times[7][4] = 3.04000;
                        keys[7][4] = AL::ALValue::array(0.493990, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                        names.push_back("LHipYawPitch");
                        times[8].arraySetSize(5);
                        keys[8].arraySetSize(5);

                        times[8][0] = 0.440000;
                        keys[8][0] = AL::ALValue::array(-0.00456004, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[8][1] = 1.20000;
                        keys[8][1] = AL::ALValue::array(0.0614019, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[8][2] = 1.96000;
                        keys[8][2] = AL::ALValue::array(0.0614019, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.280000, 0.00000));
                        times[8][3] = 2.80000;
                        keys[8][3] = AL::ALValue::array(0.0614019, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                        times[8][4] = 3.04000;
                        keys[8][4] = AL::ALValue::array(0.0614020, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                        names.push_back("LKneePitch");
                        times[9].arraySetSize(5);
                        keys[9].arraySetSize(5);

                        times[9][0] = 0.440000;
                        keys[9][0] = AL::ALValue::array(0.794570, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[9][1] = 1.20000;
                        keys[9][1] = AL::ALValue::array(1.04001, AL::ALValue::array(3, -0.253333, -0.00460241), AL::ALValue::array(3, 0.253333, 0.00460241));
                        times[9][2] = 1.96000;
                        keys[9][2] = AL::ALValue::array(1.04461, AL::ALValue::array(3, -0.253333, -0.00145717), AL::ALValue::array(3, 0.280000, 0.00161055));
                        times[9][3] = 2.80000;
                        keys[9][3] = AL::ALValue::array(1.04921, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                        times[9][4] = 3.04000;
                        keys[9][4] = AL::ALValue::array(1.04768, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                        names.push_back("LShoulderPitch");
                        times[10].arraySetSize(5);
                        keys[10].arraySetSize(5);

                        times[10][0] = 0.440000;
                        keys[10][0] = AL::ALValue::array(1.63213, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[10][1] = 1.20000;
                        keys[10][1] = AL::ALValue::array(1.62600, AL::ALValue::array(3, -0.253333, 0.00357938), AL::ALValue::array(3, 0.253333, -0.00357938));
                        times[10][2] = 1.96000;
                        keys[10][2] = AL::ALValue::array(1.61066, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.280000, 0.00000));
                        times[10][3] = 2.80000;
                        keys[10][3] = AL::ALValue::array(1.61526, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                        times[10][4] = 3.04000;
                        keys[10][4] = AL::ALValue::array(1.60299, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                        names.push_back("LShoulderRoll");
                        times[11].arraySetSize(5);
                        keys[11].arraySetSize(5);

                        times[11][0] = 0.440000;
                        keys[11][0] = AL::ALValue::array(0.601287, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[11][1] = 1.20000;
                        keys[11][1] = AL::ALValue::array(0.592082, AL::ALValue::array(3, -0.253333, 0.00383506), AL::ALValue::array(3, 0.253333, -0.00383506));
                        times[11][2] = 1.96000;
                        keys[11][2] = AL::ALValue::array(0.578276, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.280000, 0.00000));
                        times[11][3] = 2.80000;
                        keys[11][3] = AL::ALValue::array(0.578276, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                        times[11][4] = 3.04000;
                        keys[11][4] = AL::ALValue::array(0.576742, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                        names.push_back("RAnklePitch");
                        times[12].arraySetSize(5);
                        keys[12].arraySetSize(5);

                        times[12][0] = 0.440000;
                        keys[12][0] = AL::ALValue::array(-0.408002, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[12][1] = 1.20000;
                        keys[12][1] = AL::ALValue::array(-0.0321720, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[12][2] = 1.96000;
                        keys[12][2] = AL::ALValue::array(-0.880473, AL::ALValue::array(3, -0.253333, 0.0902144), AL::ALValue::array(3, 0.280000, -0.0997107));
                        times[12][3] = 2.80000;
                        keys[12][3] = AL::ALValue::array(-0.980184, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                        times[12][4] = 3.04000;
                        keys[12][4] = AL::ALValue::array(0.360532, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                        names.push_back("RAnkleRoll");
                        times[13].arraySetSize(5);
                        keys[13].arraySetSize(5);

                        times[13][0] = 0.440000;
                        keys[13][0] = AL::ALValue::array(0.00464395, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[13][1] = 1.20000;
                        keys[13][1] = AL::ALValue::array(0.380475, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[13][2] = 1.96000;
                        keys[13][2] = AL::ALValue::array(0.165714, AL::ALValue::array(3, -0.253333, 0.0333097), AL::ALValue::array(3, 0.280000, -0.0368160));
                        times[13][3] = 2.80000;
                        keys[13][3] = AL::ALValue::array(0.128898, AL::ALValue::array(3, -0.280000, 0.0119311), AL::ALValue::array(3, 0.0800000, -0.00340889));
                        times[13][4] = 3.04000;
                        keys[13][4] = AL::ALValue::array(0.119694, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                        names.push_back("RElbowRoll");
                        times[14].arraySetSize(5);
                        keys[14].arraySetSize(5);

                        times[14][0] = 0.440000;
                        keys[14][0] = AL::ALValue::array(0.00872665, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[14][1] = 1.20000;
                        keys[14][1] = AL::ALValue::array(0.0245859, AL::ALValue::array(3, -0.253333, -0.00306812), AL::ALValue::array(3, 0.253333, 0.00306812));
                        times[14][2] = 1.96000;
                        keys[14][2] = AL::ALValue::array(0.0276540, AL::ALValue::array(3, -0.253333, -0.000971542), AL::ALValue::array(3, 0.280000, 0.00107381));
                        times[14][3] = 2.80000;
                        keys[14][3] = AL::ALValue::array(0.0307220, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                        times[14][4] = 3.04000;
                        keys[14][4] = AL::ALValue::array(0.0291880, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                        names.push_back("RElbowYaw");
                        times[15].arraySetSize(5);
                        keys[15].arraySetSize(5);

                        times[15][0] = 0.440000;
                        keys[15][0] = AL::ALValue::array(1.40817, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[15][1] = 1.20000;
                        keys[15][1] = AL::ALValue::array(1.40203, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[15][2] = 1.96000;
                        keys[15][2] = AL::ALValue::array(1.40510, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.280000, 0.00000));
                        times[15][3] = 2.80000;
                        keys[15][3] = AL::ALValue::array(1.40357, AL::ALValue::array(3, -0.280000, 0.00119303), AL::ALValue::array(3, 0.0800000, -0.000340865));
                        times[15][4] = 3.04000;
                        keys[15][4] = AL::ALValue::array(1.40050, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                        names.push_back("RHipPitch");
                        times[16].arraySetSize(5);
                        keys[16].arraySetSize(5);

                        times[16][0] = 0.440000;
                        keys[16][0] = AL::ALValue::array(-0.466378, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[16][1] = 1.20000;
                        keys[16][1] = AL::ALValue::array(-0.156510, AL::ALValue::array(3, -0.253333, -0.0871824), AL::ALValue::array(3, 0.253333, 0.0871824));
                        times[16][2] = 1.96000;
                        keys[16][2] = AL::ALValue::array(0.0567160, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.280000, 0.00000));
                        times[16][3] = 2.80000;
                        keys[16][3] = AL::ALValue::array(0.00302603, AL::ALValue::array(3, -0.280000, 0.0536900), AL::ALValue::array(3, 0.0800000, -0.0153400));
                        times[16][4] = 3.04000;
                        keys[16][4] = AL::ALValue::array(-0.699546, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                        names.push_back("RHipRoll");
                        times[17].arraySetSize(5);
                        keys[17].arraySetSize(5);

                        times[17][0] = 0.440000;
                        keys[17][0] = AL::ALValue::array(-0.00456004, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[17][1] = 1.20000;
                        keys[17][1] = AL::ALValue::array(0.282298, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[17][2] = 1.96000;
                        keys[17][2] = AL::ALValue::array(0.257754, AL::ALValue::array(3, -0.253333, 0.0119013), AL::ALValue::array(3, 0.280000, -0.0131541));
                        times[17][3] = 2.80000;
                        keys[17][3] = AL::ALValue::array(0.207132, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                        times[17][4] = 3.04000;
                        keys[17][4] = AL::ALValue::array(0.280764, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                        names.push_back("RHipYawPitch");
                        times[18].arraySetSize(5);
                        keys[18].arraySetSize(5);

                        times[18][0] = 0.440000;
                        keys[18][0] = AL::ALValue::array(-0.00456004, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[18][1] = 1.20000;
                        keys[18][1] = AL::ALValue::array(0.0614019, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[18][2] = 1.96000;
                        keys[18][2] = AL::ALValue::array(0.0614019, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.280000, 0.00000));
                        times[18][3] = 2.80000;
                        keys[18][3] = AL::ALValue::array(0.0614019, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                        times[18][4] = 3.04000;
                        keys[18][4] = AL::ALValue::array(0.0614020, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                        names.push_back("RKneePitch");
                        times[19].arraySetSize(5);
                        keys[19].arraySetSize(5);

                        times[19][0] = 0.440000;
                        keys[19][0] = AL::ALValue::array(0.797722, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[19][1] = 1.20000;
                        keys[19][1] = AL::ALValue::array(-0.00302603, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[19][2] = 1.96000;
                        keys[19][2] = AL::ALValue::array(0.859083, AL::ALValue::array(3, -0.253333, -0.279802), AL::ALValue::array(3, 0.280000, 0.309255));
                        times[19][3] = 2.80000;
                        keys[19][3] = AL::ALValue::array(1.76414, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                        times[19][4] = 3.04000;
                        keys[19][4] = AL::ALValue::array(-0.0923279, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                        names.push_back("RShoulderPitch");
                        times[20].arraySetSize(5);
                        keys[20].arraySetSize(5);

                        times[20][0] = 0.440000;
                        keys[20][0] = AL::ALValue::array(1.52944, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[20][1] = 1.20000;
                        keys[20][1] = AL::ALValue::array(1.50950, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[20][2] = 1.96000;
                        keys[20][2] = AL::ALValue::array(1.51103, AL::ALValue::array(3, -0.253333, -0.000728451), AL::ALValue::array(3, 0.280000, 0.000805130));
                        times[20][3] = 2.80000;
                        keys[20][3] = AL::ALValue::array(1.51410, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                        times[20][4] = 3.04000;
                        keys[20][4] = AL::ALValue::array(1.50643, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                        names.push_back("RShoulderRoll");
                        times[21].arraySetSize(5);
                        keys[21].arraySetSize(5);

                        times[21][0] = 0.440000;
                        keys[21][0] = AL::ALValue::array(-0.779314, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                        times[21][1] = 1.20000;
                        keys[21][1] = AL::ALValue::array(-0.739430, AL::ALValue::array(3, -0.253333, -0.00613479), AL::ALValue::array(3, 0.253333, 0.00613479));
                        times[21][2] = 1.96000;
                        keys[21][2] = AL::ALValue::array(-0.733295, AL::ALValue::array(3, -0.253333, -0.00170007), AL::ALValue::array(3, 0.280000, 0.00187902));
                        times[21][3] = 2.80000;
                        keys[21][3] = AL::ALValue::array(-0.728692, AL::ALValue::array(3, -0.280000, -0.00238645), AL::ALValue::array(3, 0.0800000, 0.000681843));
                        times[21][4] = 3.04000;
                        keys[21][4] = AL::ALValue::array(-0.724090, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                        try
                        {
                          // motion.moveTo(0.165,0,0);
                          motion.angleInterpolationBezier(names, times, keys);
                          posture.goToPosture("StandInit", 0.5);
                        }
                        catch(const std::exception&)
                        {
                            cout << "Error in kick " << endl;
                        }
                      std::cout << "Well done!" << std::endl;
                      flag = false;
                        //////////////////////////////////////////////7

                    }
               }


                    /////////////////////////---------kick---------/////7////////////////////////7
                    /// ///////////////////////////////////////////////////////////////////7

               if ((postura == "LyingBelly")||(postura == "LyingBack")||(postura == "Back"))
               {
                  posture.goToPosture("Stand",1);
               }
                //imshow("Thresholded Image", imgThresholded); //show the thresholded image
                //imshow("Src", frame); //show the original image
                //key = waitKey(50);
            }
            contFrames++;
        }
       camProxy.unsubscribe(clientName);
}

void rutinaApagado(){
    AL::ALMotionProxy motion;
    AL::ALRobotPostureProxy posture;
    motion.stopMove();
    posture.goToPosture("Crouch",0.8);
    motion.setStiffnesses("Body",0);
}
