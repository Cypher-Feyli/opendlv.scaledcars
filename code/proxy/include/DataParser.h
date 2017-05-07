#ifndef DataParser_H_
#define DataParser_H_

#include <opendavinci/odcore/io/StringListener.h>
#include <map>
#include <string>
using namespace std;

        class DataParser : public odcore::io::StringListener {
            
            public:  
          
            virtual void PackageData(const std::string s);
               
            virtual bool DataDone();
           
            virtual map<uint32_t, double> GetValues();
            
            virtual void nextString(const std::string &s);
            
            virtual void Reset();
            
            virtual bool Handshake();

           private:
             
            bool add = false;
            char delim1 = '[';
            char delim2 = ']';
            string dataValues = "";
            //map<string, int> mapValues;
 
         };
#endif /* MYHEADER_H */
