/**
 * proxy - Sample application to encapsulate HW/SW interfacing with embedded systems.
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

#include <ctype.h>
#include <cstring>
#include <cmath>
#include <iostream>
#include "DataParser.h"
#include "opendavinci/odcore/base/KeyValueConfiguration.h"
#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/odcore/data/TimeStamp.h"
#include <stdint.h>
#include <string>
#include <memory>
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>
#include <ctype.h>
#include <cstring>
#include <cmath>
#include <iostream>
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"
#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"

using namespace std;
#include <opendavinci/odcore/base/Thread.h>
// We add some of OpenDaVINCI's namespaces for the sake of readability.
using namespace odcore;
using namespace odcore::wrapper;
#include "OpenCVCamera.h"

#ifdef HAVE_UEYE
#include "uEyeCamera.h"
#endif

#include "Proxy.h"

namespace automotive {
    namespace miniature {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace odtools::recorder;
        //Used when establishing serial connection
        string SERIAL_PORT;
        uint32_t BAUD_RATE;
        Proxy::Proxy(const int32_t &argc, char **argv) :
            TimeTriggeredConferenceClientModule(argc, argv, "proxy"),
            m_recorder(),
            m_camera()
        {}

        Proxy::~Proxy() {
        }

        void Proxy::setUp() {
            // This method will be call automatically _before_ running body().
            if (getFrequency() < 20) {
                cerr << endl << endl << "Proxy: WARNING! Running proxy with a LOW frequency (consequence: data updates are too seldom and will influence your algorithms in a negative manner!) --> suggestions: --freq=20 or higher! Current frequency: " << getFrequency() << " Hz." << endl << endl << endl;
            }

            // Get configuration data.
            KeyValueConfiguration kv = getKeyValueConfiguration();

            // Create built-in recorder.
            const bool useRecorder = kv.getValue<uint32_t>("proxy.useRecorder") == 1;
            if (useRecorder) {
                // URL for storing containers.
                stringstream recordingURL;
                recordingURL << "file://" << "proxy_" << TimeStamp().getYYYYMMDD_HHMMSS_noBlankNoColons() << ".rec";
                // Size of memory segments.
                const uint32_t MEMORY_SEGMENT_SIZE = getKeyValueConfiguration().getValue<uint32_t>("global.buffer.memorySegmentSize");
                // Number of memory segments.
                const uint32_t NUMBER_OF_SEGMENTS = getKeyValueConfiguration().getValue<uint32_t>("global.buffer.numberOfMemorySegments");
                // Run recorder in asynchronous mode to allow real-time recording in background.
                const bool THREADING = true;
                // Dump shared images and shared data?
                const bool DUMP_SHARED_DATA = getKeyValueConfiguration().getValue<uint32_t>("proxy.recorder.dumpshareddata") == 1;

                m_recorder = unique_ptr<Recorder>(new Recorder(recordingURL.str(), MEMORY_SEGMENT_SIZE, NUMBER_OF_SEGMENTS, THREADING, DUMP_SHARED_DATA));
            }

            // Create the camera grabber.
            const string NAME = getKeyValueConfiguration().getValue<string>("proxy.camera.name");
            string TYPE = getKeyValueConfiguration().getValue<string>("proxy.camera.type");
            std::transform(TYPE.begin(), TYPE.end(), TYPE.begin(), ::tolower);
            const uint32_t ID = getKeyValueConfiguration().getValue<uint32_t>("proxy.camera.id");
            const uint32_t WIDTH = getKeyValueConfiguration().getValue<uint32_t>("proxy.camera.width");
            const uint32_t HEIGHT = getKeyValueConfiguration().getValue<uint32_t>("proxy.camera.height");
            const uint32_t BPP = getKeyValueConfiguration().getValue<uint32_t>("proxy.camera.bpp");
            //adjustable values in the config for serial port and baud rate.
            SERIAL_PORT = getKeyValueConfiguration().getValue<string>("proxy.actuator.serialport");
            BAUD_RATE = getKeyValueConfiguration().getValue<uint32_t>("proxy.sensor.serialspeed");
            const bool DEBUG = getKeyValueConfiguration().getValue< bool >("proxy.camera.debug") == 1;
            const bool FLIPPED = getKeyValueConfiguration().getValue< uint32_t >("proxy.camera.flipped") == 1;

            if (TYPE.compare("opencv") == 0) {
                m_camera = unique_ptr< Camera >(new OpenCVCamera(NAME, ID, WIDTH, HEIGHT, BPP, DEBUG, FLIPPED));
            }
            if (TYPE.compare("ueye") == 0) {
#ifdef HAVE_UEYE
                m_camera = unique_ptr<Camera>(new uEyeCamera(NAME, ID, WIDTH, HEIGHT, BPP));
#endif
            }


            if (m_camera.get() == NULL) {
                cerr << "No valid camera type defined." << endl;
            }

        }

        void Proxy::tearDown() {
            // This method will be call automatically _after_ return from body().
        }

        void Proxy::distribute(Container c) {
            // Store data to recorder.
            if (m_recorder.get() != NULL) {
                // Time stamp data before storing.
                c.setReceivedTimeStamp(TimeStamp());
                m_recorder->store(c);
            }

            // Share data.
            getConference().send(c);
        }

        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Proxy::body() {
            //Uses open da vincis library http://opendavinci.cse.chalmers.se/api/classodcore_1_1wrapper_1_1SerialPort.html
            std::shared_ptr<SerialPort> serial(SerialPortFactory::createSerialPort(SERIAL_PORT, BAUD_RATE));
            DataParser handler;
            //set stringlistener to the handler
            serial->setStringListener(&handler);

            // Start receiving bytes.
            serial->start();
            const uint32_t TENTH_SECOND = 1000 * 100;

            //const uint32_t HUNDRED_SECOND = 1000 * 10;

            //const uint32_t ONE_SECOND = 1000 * 1000;
            //odcore::base::Thread::usleepFor(10 * ONE_SECOND);
            uint32_t captureCounter = 0;
            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                // Capture frame.

                if (m_camera.get() != NULL) {
                    odcore::data::image::SharedImage si = m_camera->capture();

                    Container c(si);
                    distribute(c);
                    captureCounter++;
                }
                //Waiting till handshake is done between the odroid and the arduino
                while(!handler.Handshake()){
                    cerr << "initializing handshake" << endl;
                    //wait inbetween a bit
                    odcore::base::Thread::usleepFor(10 * TENTH_SECOND);
                };
                uint32_t sensors=4; //we have only 4 sensors
                //Just making sure each sensor had time to setup on the arduino
                if(captureCounter > 100){
                    if(handler.DataDoneSBD()){  
                        //test printing all values from sensorboard
                        map<uint32_t, double> m = handler.GetValuesSBD();
                        for (const auto &p : m) {
                            std::cout << "m[" << p.first << "] = " << p.second << '\n';
                        }
                        //Make a sensorboarddata with 4 sensors and get the Sensorboardata map 
                        SensorBoardData sbd(sensors, handler.GetValuesSBD());
                        //make a container with sbd
                        Container csbd(sbd);
                        getConference().send(csbd); // send the container
                        handler.ResetSBD(); //reset and wait for new data
                    }
                    //Check if all the vehicledata is done
                    if(handler.DataDoneVD()){
                        cerr << handler.GetValuesVD() << endl; // print proxy
                        VehicleData vehicleData; // vehicledata
                        vehicleData.setAbsTraveledPath(handler.GetValuesVD()); // get the odometer distance
                        Container cvd(vehicleData); // make a container add the odometer data 
                        getConference().send(cvd); // send it
                        handler.ResetVD(); // reset it and wait for new vehicledata
                    }
                    //odcore::base::Thread::usleepFor(100 * HUNDRED_SECOND);

                    Container vehicleControlContainer = getKeyValueDataStore().get(automotive::VehicleControl::ID());
                    VehicleControl vehicleControlData = vehicleControlContainer.getData<VehicleControl> ();

                    double speed = vehicleControlData.getSpeed(); 
                    double Angle = vehicleControlData.getSteeringWheelAngle();
                    int speed1 = (int) speed; // Cast Speed to int
                    //since the car axis is not straight at angle 90 we adjusted this to 81.
                    int Angle1 = (int) 81  + (Angle * (180 / 3.1415926535)); // Cast angle to int
                    cerr << "Angle is: "<<Angle1 << endl; // Print angle we are sending
                    cerr << "direction is: " << speed1 << endl; // Print speed which is direction aswell
                    std::string angleString = std::to_string(Angle1);  //angle to string
                    //These if statements divide speed to 4 diffrent parameters which indciate direction and speed
                    if(speed1 >= 1){ // Speed bigger then 1 we send forward
                        serial->send("[F" + angleString + "]");
                    }
                    if(speed < 1 && speed > 0){ // speed elss then one but bigger then 0 send slow speed
                        serial->send("[K" + angleString + "]");
                    }
                    else if(speed1 == 0){//speed is 0 which is stop
                        serial->send("[S" + angleString + "]");
                    }
                    else if(speed1 < 0){//Speed is less than 0 sends backawrds
                        serial->send("[B" + angleString + "]");
                    }
                }
            }
            cout << "Proxy: Captured " << captureCounter << " frames." << endl;
            serial->stop();
            serial->setStringListener(NULL);

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

    }
} // automotive::miniature


