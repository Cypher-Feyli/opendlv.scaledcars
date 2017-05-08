#ifndef DataParser_H_
#define DataParser_H_

#include <opendavinci/odcore/io/StringListener.h>
#include <map>
#include <string>
using namespace std;

        class DataParser : public odcore::io::StringListener {
            
            public:  
          
            virtual void PackageData(const std::string s);
               
            virtual bool DataDoneVD();
           
            virtual bool DataDoneSBD();

            virtual map<uint32_t, double> GetValuesSBD();

            virtual double GetValuesVD();
            
            virtual void nextString(const std::string &s);
            
            virtual void ResetSBD();
            
            virtual void ResetVD();
            
            virtual bool Handshake();

           private:
             
            bool add = false;
            char delim1 = '[';
            char delim2 = ']';
            string dataValues = "";
            //map<string, int> mapValues;
 
         };
#endif /* MYHEADER_H */
