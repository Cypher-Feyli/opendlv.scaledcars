#include <opendavinci/odcore/io/StringListener.h>
#include <stdint.h>
#include <string>
#include <ctype.h>
#include <cstring>
#include <map>
#include "DataParser.h"
#include<iostream>

bool usvals = false;
bool irvals = false;
using namespace std;
bool add = false;
bool done = false;
bool vehicledata = false;
bool handshake = false;
char delim1 = '[';
char delim2 = ']';
string dataValues = "";
uint32_t i=0;
double dist = 0;
map<uint32_t, double> mapValues;
using namespace std;

/*
 * Function which takes parameter vals and checks what type of package it is. If it is a US or IR packet
 * we insert it to the map. If it's a handshake we set handshake to true, and if it is a wheel encoder packet we
 * set the variable dist to what we received from the lower level board. If we received both US and IR values
 * we set done to true indicating that the sensorBoard map is done.
 */
void DataParser::PackageData(const string vals){
    if(vals.at(0) == 'U'){
        int FirstDelim = vals.find(".");
        mapValues.insert ( pair<uint32_t,double>((i+3),stod(vals.substr(FirstDelim+1))));
        usvals = true;
    }
    else if(vals.at(0) == 'I'){
        int FirstDelim = vals.find(".");
        int secondDelim = vals.find(",");
        int ThirdDelim = vals.find(";");

        mapValues.insert(pair<uint32_t,double>((i),stod(vals.substr(FirstDelim+1,secondDelim))));
        mapValues.insert( pair<uint32_t,double>((i+1),stod(vals.substr(secondDelim+1,ThirdDelim))));
        mapValues.insert( pair<uint32_t,double>((i+2),stod(vals.substr(ThirdDelim+1))));
        irvals = true;
    }
    else if(vals.at(0) == 'V'){
        int FirstDelim = vals.find(".");
        dist = stoi(vals.substr(FirstDelim+1));
        vehicledata = true;
    }
    else if(vals.at(0) == 'H'){
        cerr << "Hand shook" << endl;
        handshake = true;        
    }
    if(usvals  && irvals){
        done = true;
    }
}
/* 
 * This method will handle any bytes received from the low level SerialPort.
 */
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
/**
 * Return a bool which Indicates if SensorBoardData is done.
 */
bool DataParser::DataDoneSBD(){
    return done;
}
/**
 * The map with all the sensorboarddata
 * Returns the map.
 */
map<uint32_t, double> DataParser::GetValuesSBD(){
    return mapValues;
}
/**
 * Return a bool to indicate if vehicledata is done
 */
bool DataParser::DataDoneVD(){
    return vehicledata;
}
/**
 * Returns a double which is data travelled
 */
double DataParser::GetValuesVD(){
    return dist;
}
/**
 * Returns a bool that indicates if a handshake was made
 */
bool DataParser::Handshake(){
    return handshake;
}
/**
 * Resets sensorboarddata
 * and reset all booleans that indicate done values.
 */
void DataParser::ResetSBD(){
    mapValues.clear();
    usvals = false;
    irvals = false;
    done = false;
}
/**
 * Resets vehicledata
 */
void DataParser::ResetVD(){
    dist = 0;
    vehicledata = false;
}

