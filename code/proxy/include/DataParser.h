#ifndef DataParser_H_
#define DataParser_H_

#include <opendavinci/odcore/io/StringListener.h>
#include <map>
#include <string>
using namespace std;
          /**
           * This class parses the data we get from the serial connection.
           * Uses odcores StringListener.
           * http://opendavinci.cse.chalmers.se/api/classodcore_1_1io_1_1StringListener.html
           */
        class DataParser : public odcore::io::StringListener {
            
            public:  
            /**
            * Function to package data into a map.
            * @param s the string it takes and puts it into the map
            */
            virtual void PackageData(const std::string s);
            /**
             * Returns a bool that indicates if we have gotten all the vehicle data 
             * from the arduino board.
             */
            virtual bool DataDoneVD();
            /**
             * Return a bool that indicates if we have gotten all the sensor data from
             * the arduino board.
             */
            virtual bool DataDoneSBD();
            /**
             * Returns a map<uint32_t, double> which is a map of all sensors.
             */
            virtual map<uint32_t, double> GetValuesSBD();
            /**
             * Returns the vehicle data that has been gotten from the car.
             * in this case it is the wheel encoder data.
             */
            virtual double GetValuesVD();
            /**
             * Function that is apart of odcore::io:Stringlistener which takes the next
             * String received
             * http://opendavinci.cse.chalmers.se/api/classodcore_1_1io_1_1StringListener.html
             * @param s the string received
             */
            virtual void nextString(const std::string &s);
            /**
             * Functions that reset the SensorBoardData when called.
             */
            virtual void ResetSBD();
            /**
             * Function that reset the VehicleData when called
             */
            virtual void ResetVD();
            /**
             * Return a bool to check if there was a serial handshake with
             * the arduino
             */
            virtual bool Handshake();

           private:
             
            bool add = false;
            char delim1 = '[';
            char delim2 = ']';
            string dataValues = "";
            //map<string, int> mapValues;
 
         };
#endif /* MYHEADER_H */
