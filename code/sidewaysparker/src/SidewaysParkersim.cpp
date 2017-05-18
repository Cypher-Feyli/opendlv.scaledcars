/**
 * sidewaysparker - Sample application for realizing a sideways parking car.
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

#include <cstdio>
#include <cmath>
#include <iostream>

#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/data/Container.h"

#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"

#include "SidewaysParker.h"

namespace automotive {
    namespace miniature {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace automotive;
        using namespace automotive::miniature;

        SidewaysParker::SidewaysParker(const int32_t &argc, char **argv) :
            TimeTriggeredConferenceClientModule(argc, argv, "SidewaysParker") {
        }

        SidewaysParker::~SidewaysParker() {}

        void SidewaysParker::setUp() {
            // This method will be call automatically _before_ running body().
        }

        void SidewaysParker::tearDown() {
            // This method will be call automatically _after_ return from body().
        }

        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode SidewaysParker::body() {
            const double ULTRASONIC_FRONT_RIGHT = 0;
           // const double INFRARED_REAR_CENTER = 1;
            
            double distanceOld = 0;
            double absPathStart = 0;
            double absPathEnd = 0;
            
           // int back = 0;
            int stageMoving = 0;
            int stageMeasuring = 0;

            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                // 1. Get most recent vehicle data:
                Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                VehicleData vd = containerVehicleData.getData<VehicleData> ();

                // 2. Get most recent sensor board data:
                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();
        
                // back = sbd.getValueForKey_MapOfDistances(INFRARED_REAR_CENTER);
                 //cerr << "Back is"<< back << endl;
                 




                // Create vehicle control data.
                VehicleControl vc;

                // Moving state machine.In 10 frequency
                if (stageMoving == 0) {
                    // Go forward.default speed 1
                    vc.setSpeed(1);
                    vc.setSteeringWheelAngle(0);
                }
                if ((stageMoving > 0) && (stageMoving < 10)) {
                    // Move slightly forward.10 tics
                    vc.setSpeed(1);
                    vc.setSteeringWheelAngle(0);
                    stageMoving++;
                }
                if ((stageMoving >= 10) && (stageMoving < 15)) {
                    // Stop.find the parking gap
                    vc.setSpeed(0);
                    vc.setSteeringWheelAngle(0);
                    stageMoving++;
                }
                if ((stageMoving >= 15) && (stageMoving < 70)) {
                    // Backwards, steering wheel to the right.
                    vc.setSpeed(-1);
                    vc.setSteeringWheelAngle(25);
                    stageMoving++;
                }
                if ((stageMoving >= 70) && (stageMoving < 100)) {
                    // Backwards, steering wheel to the left.
                    vc.setSpeed(-1);
                    vc.setSteeringWheelAngle(-25);
                    stageMoving++;
                }
                if ((stageMoving >= 100) && (stageMoving < 105)) {
                    // turn left to straight up
                    vc.setSpeed(0);
                    vc.setSteeringWheelAngle(-25);
                    stageMoving++;
                }
                  if ((stageMoving >= 105) && (stageMoving < 130)) {
                    //  turn right to straight up.
                    vc.setSpeed(1);
                    vc.setSteeringWheelAngle(10);
                    stageMoving++;



                }
                 if ((stageMoving >= 130) && (stageMoving < 150))  {
                    //  Move slightly Backwards.
                    vc.setSpeed(-1);
                    vc.setSteeringWheelAngle(0);
                    stageMoving++;



                }





                                  
                 if (stageMoving >= 150) {
                    // Stop.
                    vc.setSpeed(0);
                    vc.setSteeringWheelAngle(0);
                    
    
                   
                }



                // Measuring state machine.
                switch (stageMeasuring) {
                    case 0:
                        {
                            // Initialize measurement.Set distanceold between the car and start obstacle.
                            distanceOld = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT);
                            stageMeasuring++;
                        }
                    break;
                    case 1:
                        {
                         // Checking for sequence +, -.Use preivous distacenold data to make sure already find obstacle.
                            if ((distanceOld > 0) && (sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT) < 0)) {
                           // Found sequence +, -. US sensor does not find obstacle anymore , value of US sensor is -1 in sim
                                //find start of gap
                                stageMeasuring = 2;
                                                  
                                                 //WHEEL_ENCODER 
                                absPathStart = vd.getAbsTraveledPath();
                            }
                            //update distanceold value
                            distanceOld = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT);
                            
                        }
                    break;
                    case 2:
                        {
                            // Checking for sequence -, +.
                            if ((distanceOld < 0) && (sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT) > 0)) {
                                // Found sequence -, +.
                                // find end of gap  ,Distanceolad value in sim small than 0.(-1)
                                stageMeasuring = 1;
                                
                                             //WHEEL_ENCODER 
                                absPathEnd = vd.getAbsTraveledPath();
                              

                                const double GAP_SIZE = (absPathEnd - absPathStart);
                                 
                                cerr << "Gap Size  = " << GAP_SIZE << endl;

                                                          //set the gap size 
                                if ((stageMoving < 1) && (GAP_SIZE > 10)) {
                                     
                                    //find GAP and triggle parking 
                                    stageMoving = 1;
                                }
                            }
                            //update distanceold value
                            distanceOld = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT);
                        }
                    break;
                }

                // Create container for finally sending the data.
                Container c(vc);
                // Send container.
                getConference().send(c);
            }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }
    }
} // automotive::miniature

