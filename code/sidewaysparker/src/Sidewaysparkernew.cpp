/**
 * Copyright (C) 2012 - 2015 Christian Berger
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
#include <math.h>
#include <cmath>
#include <iostream>

#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/data/Container.h"

#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"

#include "SidewaysParker"

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


        // This method called automatically before running the body
        void SidewaysParker::setUp() {

             

        }
        // This method will be call automatically after return from body().
        void SidewaysParker::tearDown() {

        }

        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode ParallelParker::body() {
        //const double WHEEL_ENCODER = 5;
		    const double INFRARED_FRONT_RIGHT = 0;
		    const double INFRARED_REAR_CENTER = 1;
			const double WHEEL_ENCODER = 5;
                const double car = 1.5;
				const double carGap = car * 2;
		        
				
				
				double currentTraveledPath;
                double absPathStart = 0;
                double absPathEnd = 0;
                //double currentStageDistance = 0;
               // double secondStageDistance = 0;
			   const double backRight = 3.5;
	          // const double backRight = 4.0;
	           const double reverse = 2.5;
	           const double backLeft = 3.2;
			
               

		        int stage = 0;
                int stageMoving = 0;
                int stageMeasuring = 0;
                //int angle = 0;
             


            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                // 1. Get most recent vehicle data:
                Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                VehicleData vd = containerVehicleData.getData<VehicleData> ();


                // 2. Get most recent sensor board data:
                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();

                // Create vehicle control data. Obs! -> only for the simulator
                VehicleControl vc;

                 cerr << "Stage is" << stage << endl;

                     if (stageMoving ==  0) {
                         vc.setSpeed(1); // In proxy, all the speed which is bigger than 0 sets vc speed to 1615.
  
                         vc.setSteeringWheelAngle(0);  
                          
					 }

                     if (stageMoving == 1) {

					    stage++;
					    stageMoving++;
					   

					  }

                      if (stage == 1) {
                   // This stage triggers when we found a gap. We keep on moving on the same speed.

                         vc.setSpeed(1);
                         vc.setSteeringWheelAngle(0);
		                 
						 currentdistance = vd.getAbsTraveledPath();

                   stage++;

                      }
                        //Go back right 
                      if (stage == 2) {  
					        //Move until the vehicle has moved the desired distance                                               
					     if (vd.getAbsTraveledPath() - currentTraveledPath <= backRight) {
                    		vc.setSpeed(-1);
                    		vc.setSteeringWheelAngle(45)
						 stage++;

					  }	else (stage == 3) {
					                                                                                  
					     vc.setSpeed(0);
                         currentdistance = vd.getAbsTraveledPath();

					  }
                      //Go straight back
					  if (stage == 3) {
                                  //Move until the vehicle has moved the desired distance
					     if (vd.getAbsTraveledPath() - currentTraveledPath <= reverse) {
				         vc.setSpeed(-1);
				         vc.setSteeringWheelAngle(0);
						  stage++;
						 }else(stage == 4){
							 vc.setSpeed(0);
				         currentdistance = vd.getAbsTraveledPath();
						 
						 }
                        

						//Go back left
 
					  }  if(stage == 4 && vd.getAbsTraveledPath() - currentTraveledPath <= backLeft) {
                             //Move until the vehicle is close to the back obstacle
							 stageMeasuring = 3
                         vc.setSpeed(-1);
                         vc.setSteeringWheelAngle(-25);
						  stage++;
                       

                      } else  (stage == 5) {
						  currentdistance = vd.getAbsTraveledPath();
						  stageMeasuring = 3

					      

					  }

					  if (stage == 5) {

                        //Move until the vehicle has moved the desired distance
			     if (vd.getAbsTraveledPath() - currentTraveledPath <= 1){
			        	vc.setSteeringWheelAngle(0);
				          vc.setSpeed(0.5);

		              } else  (stage == 7){
                            currentdistance = vd.getAbsTraveledPath();
			

                  	
					 }

					 if (stage == 7) {

					  vc.setSpeed(0);
                      vc.setSteeringWheelAngle(0);

					 }

					 switch (stageMeasuring) {

					 case 0 :    // No object is found, gap starts

					  {

					      if ((sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) < 6 ) {
                                                 
                                                  absPathStart = vd.getAbsTraveledPath();

					  } 

                                   {

                                             stageMeasuring++;

                                                }
                                            } break;

					 case 1 : // Object is found, gap ends

					 {
					   if ((sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) > 0 {

					             stageMeasuring = 0
					   absPathEnd = vd.getAbsTraveledPath();
					 } 
                         {                   

                                          
					   const double GAP_SIZE = (absPathEnd - absPathStart);

					    cerr << "The gap for parking is = " << GAP_SIZE << endl;

                            if ((stageMoving < 1) && (GAP_SIZE > = carGap)) {
                                stageMoving = 1;
                            }



                        }


					 } break;
					
					case 2 {
			if (sbd.getValueForKey_MapOfDistances(INFRARED_REAR_CENTER) <= 7 && sbd.getValueForKey_MapOfDistances(INFRARED_REAR_CENTER) > 0){
				stageMoving = 5;
				stageMeasuring = 3;
				}
			
		}break;
		            //Stop measuring
		case 3 {
			//Measuring disabled
		}
		
              
			  
			  
			  
					 }

					 Container c(vc);
					 getConference().send(c);

					 }

				  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
			}
		}
	}
