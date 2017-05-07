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
bool handshake = false;
char delim1 = '[';
char delim2 = ']';
string dataValues = "";
uint32_t i=0;
map<uint32_t, double> mapValues;
// Your class needs to implement the method void nextString(const std::string &s).
void DataParser::PackageData(const string vals){
    if(vals.at(0) == 'U'){
        int FirstDelim = vals.find(".");
        //mapValues.insert(pair<string,int>("USFR",stoi(vals.substr(FirstDelim+1,secondDelim))));
        mapValues.insert ( pair<uint32_t,double>((i+3),stod(vals.substr(FirstDelim+1))));
        Data++;
    }
    else if(vals.at(0) == 'I'){
        int FirstDelim = vals.find(".");
        int secondDelim = vals.find(",");
        int ThirdDelim = vals.find(";");

        mapValues.insert(pair<uint32_t,double>((i),stod(vals.substr(FirstDelim+1,secondDelim))));
        mapValues.insert( pair<uint32_t,double>((i+1),stod(vals.substr(secondDelim+1,ThirdDelim))));
        mapValues.insert( pair<uint32_t,double>((i+2),stod(vals.substr(ThirdDelim+1))));
        Data++;
    }
    else if(vals.at(0) == 'H'){
        cerr << "Hand shook" << endl;
        handshake = true;        
    }
    if(Data == 2){
        done = true;
    }
}
//tests atm new update coming now
void DataParser::nextString(const string &s) {
    cerr << "Received " << s.length() << " bytes containing '" << s << "'" << endl;
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
    return done;
}
map<uint32_t, double> DataParser::GetValues(){
    return mapValues;
}
bool DataParser::Handshake(){
    return handshake;
}
void DataParser::Reset(){
    mapValues.clear();
    Data = 0;
    done = false;
}
