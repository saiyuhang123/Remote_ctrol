#ifndef LASERLasrProtocol_H
#define LASERLasrProtocol_H
#include <iostream>
#include <vector>
#include <string>
#include <string.h>
#include "protoc.h"
#include <cmath>
using namespace std;



class LasrProtocol
{
public:
    LasrProtocol();

    static unsigned int CRC_Verify_len12( vector<unsigned char> &command);

    static void GenerateParamCommand(XingSongLaserParam param,vector<unsigned char> &data,int type);

    static void SetAreaCommand(HSGetAreaDataPackage command,vector<unsigned char> &data);
    
    static short int CRCVerify(vector<unsigned char> &data, int len);

    static double normalizeAngle(double angle);
private:


};

#endif // LasrProtocol_H
