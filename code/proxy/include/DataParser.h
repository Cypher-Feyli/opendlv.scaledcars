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
           
            virtual map<string, int> GetValues();
            
            virtual void nextString(const std::string &s);

           private:
             
            bool add = false;
            char delim1 = '[';
            char delim2 = ']';
            string dataValues = "";
            //map<string, int> mapValues;
 
         };
#endif /* MYHEADER_H */
