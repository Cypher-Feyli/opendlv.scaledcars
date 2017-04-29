#include <opendavinci/odcore/io/StringListener.h>
#include <stdint.h>
#include <string>
#include <ctype.h>
#include <cstring>
#include <map>
#include "DataParser.h"
#include<iostream>

int Data = 0;
using namespace std;
bool add = false;
bool done = false;
char delim1 = '[';
char delim2 = ']';
string dataValues = "";
map<string, int> mapValues;
// Your class needs to implement the method void nextString(const std::string &s).
void DataParser::PackageData(const string vals){
    if(vals.at(0) == 'U'){
        int FirstDelim = vals.find(".");
        //mapValues.insert(pair<string,int>("USFR",stoi(vals.substr(FirstDelim+1,secondDelim))));
        mapValues.insert ( pair<string,int>("USFC",stoi(vals.substr(FirstDelim+1))));
        Data++;
    }
    else if(vals.at(0) == 'I'){
        int FirstDelim = vals.find(".");
        int secondDelim = vals.find(",");
        int ThirdDelim = vals.find(";");

        mapValues.insert(pair<string,int>("IRFR",stoi(vals.substr(FirstDelim+1,secondDelim))));
        mapValues.insert( pair<string,int>("IRBR",stoi(vals.substr(secondDelim+1,ThirdDelim))));
        mapValues.insert( pair<string,int>("IRB",stoi(vals.substr(ThirdDelim+1))));
        Data++;
    }
    if(Data == 2){
        done = true;
    }
}
//tests atm new update coming now
void DataParser::nextString(const string &s) {
    // cerr << "Received " << s.length() << " bytes containing '" << s << "'" << endl;
    char* chr = strdup(s.c_str());
    for(unsigned int p = 0; p < s.length(); p++){
        if(chr[p] == delim2 && add){
            PackageData(dataValues);
            dataValues = "";
            add = false;
        }
        if(chr[p] == delim1){
            add = true;
        }
        if(add && chr[p] != delim1){
            dataValues += chr[p];
        }
    }
}

bool DataParser::DataDone(){
    mapValues.clear();
    Data = 0;
    if(done){
        done = false;
        return true;
    }
    else{
        return done;
    }
}
map<string, int> DataParser::GetValues(){
    return mapValues;
}
