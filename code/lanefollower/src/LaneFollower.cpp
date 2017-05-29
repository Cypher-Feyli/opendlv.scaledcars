/**
 * lanefollower - Sample application for following lane markings.
 * Copyright (C) 2012 - 2015 Christian Berger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <iostream>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "LaneFollower.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "opendavinci/odcore/base/KeyValueConfiguration.h"
#include "opendavinci/odcore/base/Lock.h"
#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/wrapper/SharedMemoryFactory.h"

#include "automotivedata/GeneratedHeaders_AutomotiveData.h"
#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include <cmath>




namespace automotive {
    namespace miniature {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace odcore::data::image;
        using namespace automotive;
        using namespace automotive::miniature;
        using namespace cv;
        const int32_t ULTRASONIC_FRONT_CENTER = 3;
        const int32_t ULTRASONIC_FRONT_RIGHT = 4;
        const int32_t INFRARED_FRONT_RIGHT = 0;
        const int32_t INFRARED_REAR_RIGHT = 2;

        const double OVERTAKING_DISTANCE = 5.5;
        const double HEADING_PARALLEL = 0.04;

        // Overall state machines for moving and measuring.
        enum StateMachineMoving { FORWARD, TO_LEFT_LANE_LEFT_TURN, TO_LEFT_LANE_RIGHT_TURN, CONTINUE_ON_LEFT_LANE, TO_RIGHT_LANE_RIGHT_TURN, TO_RIGHT_LANE_LEFT_TURN };
        enum StateMachineMeasuring { DISABLE, FIND_OBJECT_INIT, FIND_OBJECT, FIND_OBJECT_PLAUSIBLE, HAVE_BOTH_IR, HAVE_BOTH_IR_SAME_DISTANCE, END_OF_OBJECT };

        StateMachineMoving stageMoving = FORWARD;
        StateMachineMeasuring stageMeasuring = FIND_OBJECT_INIT;

        // State counter for dynamically moving back to right lane.
        int32_t stageToRightLaneRightTurn = 0;
        int32_t stageToRightLaneLeftTurn = 0;

        // Distance variables to ensure we are overtaking only stationary or slowly driving obstacles.
        double distanceToObstacle = 0;
        double distanceToObstacleOld = 0;
        static bool useRightLaneMarking = true;


        LaneFollower::LaneFollower(const int32_t &argc, char **argv) : TimeTriggeredConferenceClientModule(argc, argv, "lanefollower"),
            m_hasAttachedToSharedImageMemory(false),
            m_sharedImageMemory(),
            m_image(NULL),
            m_debug(false),
            m_font(),
            m_previousTime(),
            m_eSum(0),
            m_eOld(0),
            m_vehicleControl() {}

        LaneFollower::~LaneFollower() {}

        void LaneFollower::setUp() {
            // This method will be call automatically _before_ running body().
            if (m_debug) {
                // Create an OpenCV-window.
                cvNamedWindow("WindowShowImage", CV_WINDOW_AUTOSIZE);
                cvMoveWindow("WindowShowImage", 300, 100);
            }
        }

       

        void LaneFollower::tearDown() {
            // This method will be call automatically _after_ return from body().
            if (m_image != NULL) {
                cvReleaseImage(&m_image);
            }

            if (m_debug) {
                cvDestroyWindow("WindowShowImage");
            }
        }

        bool LaneFollower::readSharedImage(Container &c) {
            bool retVal = false;

            if (c.getDataType() == odcore::data::image::SharedImage::ID()) {
                SharedImage si = c.getData<SharedImage> ();

                // Check if we have already attached to the shared memory.
                if (!m_hasAttachedToSharedImageMemory) {
                    m_sharedImageMemory
                            = odcore::wrapper::SharedMemoryFactory::attachToSharedMemory(
                                    si.getName());
                }

                // Check if we could successfully attach to the shared memory.
                if (m_sharedImageMemory->isValid()) {
                    // Lock the memory region to gain exclusive access using a scoped lock.
                    Lock l(m_sharedImageMemory);
                    const uint32_t numberOfChannels = 3;
                    // For example, simply show the image.
                    if (m_image == NULL) {
                        m_image = cvCreateImage(cvSize(si.getWidth(), si.getHeight()), IPL_DEPTH_8U, numberOfChannels);
                    }

                    // Copying the image data is very expensive...
                    if (m_image != NULL) {
                        memcpy(m_image->imageData,
                               m_sharedImageMemory->getSharedMemory(),
                               si.getWidth() * si.getHeight() * numberOfChannels);
                    }

                    // Mirror the image.
                    cvFlip(m_image, 0, -1);

                    retVal = true;
                }
            }
            return retVal;
        }
    //inspired partially by http://opencvexamples.blogspot.com/2013/10/line-detection-by-hough-line-transform.html
     //This method returns true if a horizontal line exist
    bool LaneFollower::intersectionDetector(Mat mat){
        bool detected = false;
          vector<Vec2f> lines;
           // detect lines
            HoughLines(mat, lines, 1, CV_PI/180, 150, 0, 0 );
         
            // draw lines
            for( size_t i = 0; i < lines.size(); i++ )
            {
                double  theta = lines[i][1];
                if( theta>CV_PI/180*80 && theta<CV_PI/180*100){
               
                cerr << "intersection detected" << endl;
                detected = true;

             }
            
            }return detected;
       }
       
        void LaneFollower::processImage() {
             //canny edge detection(http://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html?highlight=canny)
            /* Here we create three mat variable the mat unprocessed will take the m_image from the sharedImage
            the mat gray for the grayscaled image and canny_image for the canny*/
            Mat unprocessed(m_image),gray, canny_image;
            Rect cropArea(0, 288, 640, 192); 
            Mat croppedImage = unprocessed(cropArea);
            cvtColor( croppedImage, gray, CV_BGR2GRAY );
            Canny( gray, canny_image, 90, 150, 3);
            
           
            double e = 0;
          
            const int32_t CONTROL_SCANLINE = 174; 
            const int32_t distance = 250;
            int xLeft = 0;
            int xRight = 0;


            TimeStamp beforeImageProcessing;
           // this loop first goes through the bottom of the image - 8 and ends at imageheight 40.
            // starting the loop with the integer 184 and continue the loop as long as
            // y is bigger than 0, y decreases with 10 every loop
            for(int32_t y = 184; y > 40; y -= 10) {
                
                if(y==174){
                    xLeft = 0;
                    xRight = m_image->width;
                }else{
                    xLeft = (m_image->width/2)-4;
                    xRight = (m_image->width/2)+4;
                }

                // Search from middle to the left:
                uchar pixelLeft;
                Point left;
                left.y = y;
                left.x = -1;
                  for(int x = (m_image->width/2); x > xLeft; x--) {

                    //http://answers.opencv.org/question/1870/find-pixel-color-out-of-cvmat-on-specific-position/
                  // here we get the pixel value at location y,x in the mat canny_image 0 is black and 255 is white
                    pixelLeft = canny_image.at<uchar>(Point(x, y));
                   // when finding a non black pixel, break the loop and store the x value which is the vertical pixel location 
                     if (pixelLeft > 177)  {
                        left.x = x;
                        break;
                    }
                }

                // Search from middle to the right:
                uchar pixelRight;
                Point right;
                right.y = y;
                right.x = -1;
                for(int x = (m_image->width/2); x < xRight; x++) {
                    pixelRight = canny_image.at<uchar>(Point(x, y));
                    if (pixelRight > 177)  {
                        right.x = x;
                        break;
                    }
                }
             
               //If a white pixel is found in the center of the image and the car is not in a stopline
              //we check if there is a intersection.
                if(((right.x<3+(m_image->width/2)&&right.x>-1) || (left.x>(m_image->width/2)-3&&left.x>-1))&& (!stopline) ){
                    if(intersectionDetector(canny_image) == true){
                        stopline = true;
                        cerr << "stooooooooooooooooooooooopppppppp"  << endl;
                        stopped = 100;
                    }
                }

                if (m_debug) {
                    if (left.x > 0) {
                        //put lines on the image from the middle of the image to where the white pixels are found 
                        Scalar green = CV_RGB(0, 255, 0);

                        line(croppedImage, Point(m_image->width/2, y), left, green);

                    }
                    if (right.x > 0) {
                        Scalar red = CV_RGB(255, 0, 0);

                        line(croppedImage, Point(m_image->width/2, y), right, red);
                    }
                }


                if (y == CONTROL_SCANLINE) {
                    // Calculate the deviation error.
                    if (right.x > 0) {
                        if (!useRightLaneMarking) {
                            m_eSum = 0;
                            m_eOld = 0;
                        } 
                     
                       // cross track error calculation
                        e = ((right.x - m_image->width/2.0) - distance)/distance;

                        useRightLaneMarking = true;
                    }
                    else if (left.x > 0) {
                        if (useRightLaneMarking) {
                            m_eSum = 0;
                            m_eOld = 0;
                        }
                        
                        e = (distance - (m_image->width/2.0 - left.x))/distance;

                        useRightLaneMarking = false;
                    }
                    else {
                        // If no measurements are available, reset PID controller.
                        m_eSum = 0;
                        m_eOld = 0;
                    }
                }
            }

            TimeStamp afterImageProcessing;
            cerr << "Processing time: " << (afterImageProcessing.toMicroseconds() - beforeImageProcessing.toMicroseconds())/1000.0 << "ms." << endl;

            // Show resulting features.
            if (m_debug) {
                if (m_image != NULL) {
                    imshow("unprocessed",  croppedImage);
                    imshow("cannyV1",  canny_image);
                    cvWaitKey(10);
                }
            }

            TimeStamp currentTime;
            double timeStep = (currentTime.toMicroseconds() - m_previousTime.toMicroseconds()) / (1000.0 * 1000.0);
            m_previousTime = currentTime;

            if (fabs(e) < 1e-2) {
                m_eSum = 0;
            }
            else {
                m_eSum += e;
            }
//            const double Kp = 2.5;
//            const double Ki = 8.5;
//            const double Kd = 0;

            // The following values have been determined by experience from students of GU university.
            const double Kp = 1.30;
            const double Ki = 0.01;
            const double Kd = 0.1;

            const double p = Kp * e;
            const double i = Ki * timeStep * m_eSum;
            const double d = Kd * (e - m_eOld)/timeStep;
            m_eOld = e;

            const double y = p + i + d;
          //  cerr << "y" << y << endl;
            double desiredSteering = 0;
            if (fabs(e) > 1e-2) {
                desiredSteering = y;
             
             //0.52radians is approximately 30degrees which is the maximum angle of the real car
                if (desiredSteering > 0.52) {
                    desiredSteering = 0.52;
                }
                if (desiredSteering < -0.52) {
                    desiredSteering = -0.52;
                }
            }
           cerr << "PID: " << "e = " << e << ", eSum = " << m_eSum << ", desiredSteering = " << desiredSteering << ", y = " << y << endl;


            // the states of the car depending on if a stopline is detected
         
          //stopped
           if(stopline == true && stopped >0){
            m_vehicleControl.setSpeed(0);
            stopped--;
            cerr << stopped  << endl;
            forwa = 50;
           }
         
          //moving forward to pass the intersection
           else if(stopline == true && forwa >0){
             m_vehicleControl.setSpeed(1);
            // adjusting the angle for the real car
             m_vehicleControl.setSteeringWheelAngle(-0.18);
             cerr << forwa << endl;
             forwa--; 
           }
            //lanefollowing state
          else{
            stopline=false;
            m_vehicleControl.setSpeed(2);
            m_vehicleControl.setSteeringWheelAngle(desiredSteering);
          }
        }

          void LaneFollower::overtaker(){
                 // Parameters for overtaking.
            

                    // 1. Get most recent vehicle data:
                    Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                    VehicleData vd = containerVehicleData.getData<VehicleData> ();

                    // 2. Get most recent sensor board data:
                    Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                    SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();
                    //bool passed = false;

                    // Moving state machine.
                   if (stageMoving == FORWARD) {
                        // Use m_vehicleControl data from image processing.

                        stageToRightLaneLeftTurn = 0;
                        stageToRightLaneRightTurn = 0;
                    }
                    else if (stageMoving == TO_LEFT_LANE_LEFT_TURN) {
                        // Move to the left lane: Turn left part until both IRs see something.
                        m_vehicleControl.setSpeed(1);
                        m_vehicleControl.setSteeringWheelAngle(-0.52);
                         cerr << "TO_LEFT_LANE_LEFT_TURN    "<< stageToRightLaneRightTurn<<" ->right    left <- "<< stageToRightLaneLeftTurn<< endl;

                        // State machine measuring: Both IRs need to see something before leaving this moving state.
                        stageMeasuring = HAVE_BOTH_IR;

                        stageToRightLaneRightTurn++;
                    }
                    else if (stageMoving == TO_LEFT_LANE_RIGHT_TURN) {
                        // Move to the left lane: Turn right part until both IRs have the same distance to obstacle.
                        m_vehicleControl.setSpeed(.6);
                        m_vehicleControl.setSteeringWheelAngle(0.52);
                         cerr << "TO_LEFT_LANE_RIGHT_TURN       "<< stageToRightLaneRightTurn<<" ->right    left <- "<< stageToRightLaneLeftTurn << endl;

                        // State machine measuring: Both IRs need to have the same distance before leaving this moving state.
                        stageMeasuring = HAVE_BOTH_IR_SAME_DISTANCE;

                        stageToRightLaneLeftTurn++;
                    }
                    else if (stageMoving == CONTINUE_ON_LEFT_LANE) {
                        // Move to the left lane: Passing stage.

                        // Use m_vehicleControl data from image processing.
                        cerr << "CONTINUE_ON_LEFT_LANE      " << stageToRightLaneRightTurn<<" ->right    left <- "<< stageToRightLaneLeftTurn<< endl;


                        // Find end of object.
                        stageMeasuring = END_OF_OBJECT;
                    }
                    else if (stageMoving == TO_RIGHT_LANE_RIGHT_TURN) {
                        // Move to the right lane: Turn right part.
                        m_vehicleControl.setSpeed(1.3);
                       m_vehicleControl.setSteeringWheelAngle(0.52);
                       cerr << "TO_RIGHT_LANE_RIGHT_TURN       "<< stageToRightLaneRightTurn<<" ->right    left <- "<< stageToRightLaneLeftTurn << endl;
                        

                        stageToRightLaneRightTurn--;
                        if (stageToRightLaneRightTurn == -13) {
                            stageMoving = TO_RIGHT_LANE_LEFT_TURN;
                        }
                    }
                    else if (stageMoving == TO_RIGHT_LANE_LEFT_TURN) {
                         cerr << "TO_RIGHT_LANE_LEFT_TURN          " << stageToRightLaneRightTurn<<" ->right    left <- "<< stageToRightLaneLeftTurn<< endl;
                        // Move to the left lane: Turn left part.
                        
                        
                        m_vehicleControl.setSpeed(2);
                        m_vehicleControl.setSteeringWheelAngle(-0.52);
                     
                        stageToRightLaneLeftTurn = 0;
                        if (stageToRightLaneLeftTurn == 0) {
                            // Start over.
                            stageMoving = FORWARD;
                            stageMeasuring = FIND_OBJECT_INIT;

                            distanceToObstacle = 0;
                            distanceToObstacleOld = 0;

                            // Reset PID controller.
                            m_eSum = 0;
                            m_eOld = 0;
                        }
                    }

                    // Measuring state machine.
                    if (stageMeasuring == FIND_OBJECT_INIT) {
                        distanceToObstacleOld = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER);
                        stageMeasuring = FIND_OBJECT;
                    }
                    else if (stageMeasuring == FIND_OBJECT) {
                        distanceToObstacle = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER);

                        // Approaching an obstacle (stationary or driving slower than us).
                        if (  (distanceToObstacle > 0) && (((distanceToObstacleOld - distanceToObstacle) > 0) || (fabs(distanceToObstacleOld - distanceToObstacle) < 1e-2)) ) {
                            // Check if overtaking shall be started.                        
                            stageMeasuring = FIND_OBJECT_PLAUSIBLE;
                        }

                        distanceToObstacleOld = distanceToObstacle;
                    }
                    else if (stageMeasuring == FIND_OBJECT_PLAUSIBLE) {
                        if (sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER) < OVERTAKING_DISTANCE) {
                            stageMoving = TO_LEFT_LANE_LEFT_TURN;

                            // Disable measuring until requested from moving state machine again.
                            stageMeasuring = DISABLE;
                        }
                        else {
                            stageMeasuring = FIND_OBJECT;
                        }
                    }
                    else if (stageMeasuring == HAVE_BOTH_IR) {
                        // Remain in this stage until both IRs see something.
                        if ( (sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) > 0) && (sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT) > 0) ) {
                            // Turn to right.
                            stageMoving = TO_LEFT_LANE_RIGHT_TURN;
                        }
                    }
                    else if (stageMeasuring == HAVE_BOTH_IR_SAME_DISTANCE) {
                        // Remain in this stage until both IRs have the similar distance to obstacle (i.e. turn car)
                        // and the driven parts of the turn are plausible.
                        const double IR_FR = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                        const double IR_RR = sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT);

                        if ((fabs(IR_FR - IR_RR) < HEADING_PARALLEL) && ((stageToRightLaneLeftTurn - stageToRightLaneRightTurn) > 0)) {
                            // Straight forward again.
                            stageMoving = CONTINUE_ON_LEFT_LANE;

                            // Reset PID controller.
                            m_eSum = 0;
                            m_eOld = 0;
                        }
                    }
                    else if (stageMeasuring == END_OF_OBJECT) {
                        // Find end of object.
                        distanceToObstacle = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT);

                        if (distanceToObstacle < 0) {
                            // Move to right lane again.
                            stageMoving = TO_RIGHT_LANE_RIGHT_TURN;

                            // Disable measuring until requested from moving state machine again.
                            stageMeasuring = DISABLE;
                        }
                    }
}
        // This method will do the main data processing job.
        // Therefore, it tries to open the real camera first. If that fails, the virtual camera images from camgen are used.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode LaneFollower::body() {
           //Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
           //        SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();
            // Get configuration data.
            KeyValueConfiguration kv = getKeyValueConfiguration();
            m_debug = kv.getValue<int32_t> ("lanefollower.debug") == 1;

            // Initialize fonts.
                const double hscale = 0.4;
            const double vscale = 0.3;
            const double shear = 0.2;
            const int thickness = 1;
            const int lineType = 6;

            cvInitFont(&m_font, CV_FONT_HERSHEY_DUPLEX, hscale, vscale, shear, thickness, lineType);




            // Overall state machine handler.
            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                bool has_next_frame = false;
                
              //  overtaker();
                // Get the most recent available container for a SharedImage.
                Container c = getKeyValueDataStore().get(odcore::data::image::SharedImage::ID());

                if (c.getDataType() == odcore::data::image::SharedImage::ID()) {
                    // Example for processing the received container.
                    has_next_frame = readSharedImage(c);
                }

                // Process the read image and calculate regular lane following set values for control algorithm.
                if (true == has_next_frame) {
                    processImage();
                }//if (sbd.getValueForKey_MapOfDistances(3)>2){
               //  overtaker();
          //  }
                   overtaker();
                // Create container for finally sending the set values for the control algorithm.
                Container c2(m_vehicleControl);
                // Send container.
                getConference().send(c2);
            }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

    }
} // automotive::miniature
