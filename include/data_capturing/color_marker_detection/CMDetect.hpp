//
//  CMDetect.hpp
//  Color Marker
//
//  Created by Roberto Manduchi on 8/17/12.
//  Copyright (c) 2012 Roberto Manduchi. 
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#include "string"


struct CMPoint {
    int iX;
    int iY;
};

struct CMColInd {
    int ind1;
    int ind2;
    int ind3;
};

struct  CMOutput {
    CMPoint center;
    CMPoint top;
    CMPoint bottom;
    CMPoint left;
    CMPoint right;
    int     perm;
};

inline CMColInd _ColInd(int _ind1, int _ind2, int _ind3){
    
    CMColInd    out;
    out.ind1 = _ind1;
    out.ind2 = _ind2;
    out.ind3 = _ind3;
    return out;
}

#define MAX_PIXELS  1000    // max number of winner pixels over threshold
#define MAX_CASCADE 18      // max number of filters in the cascade

class CMDetect {
   
    int P_SHIFT;
    
    int SCAN_STEP;
    
    int N_DIAG_RATIO;
    
    int N_MIN_HBOARD;
    
    int TOL_SHIFT_RATIO,PT_CLUSTER_THRESHOLD,PT_DISTANCE_THRESHOLD2, MAX_PIXELS1;
    int ps_c1[24],ps_a1[24],ps_d1[24],ps_b1[24],ps_c2[24],ps_a2[24],ps_d2[24],ps_b2[24];
    
    float   classPar_m[18], classPar_b1[18], classPar_b2[18];
    int     classPar_t_min_1[18], classPar_t_min_2[18], classPar_t_max_1[18], classPar_t_max_2[18];
    int     classID[18];
    
    int     cascadeLength;

    typedef enum{Top, Bottom, Left, Right} CardPoint;

    typedef enum{TopBottomHorn, LeftRightHorn, Center} KeyPointType;

    CMColInd    colClass[18];
    
    unsigned char *LTF;
    
    int permIndices[24], nPerms;
    
    int CONSISTENCYCHECK1, CONSISTENCYCHECK2, CONSISTENCYCHECK3;
    
    inline unsigned char* pixPtr(int, int, unsigned char*);
    
    int PermShift();
    
    int LoadTable();    
    
    int LoadPars();
    
    int LoadTableColClass();
    
    int FindOppositeTopBottom(CardPoint);
    
    int FindOppositeLeftRight(CardPoint);
    
    int FindKeyPoint(KeyPointType, CardPoint, CardPoint);
    
    int FindHornTopBottom(CardPoint,CardPoint);
    
    int FindHornLeftRight(CardPoint,CardPoint);

    int FindCenter(CardPoint,CardPoint);
    
    int ComputeKeypoints(int,int);
    
    int FloodSegment(int,int, short*, short*, short*, short*);
    
    
    int N_CHANNELS;
    
   
public:
    
    CMDetect (std::string _userParsFileName, std::string _classParsFileName);

    // DEPRECATED
    // CMDetect (std::string _userParsFileName);
    
    ~CMDetect();
    
    int AccessImage(unsigned char*,int,int,int);
    int FindTarget();
    int ParseClassifiersParsXML();
    int ParseUserParsXML();
    
    int SetMarkerID(int);  // overrules initalization by ParseUserParsXML

    std::string  userParsFileName;
    
    int     perm;
    unsigned char*  ptr;
    int     IMAGE_W;
    int     IMAGE_H;
    int     WIDTH_STEP;
    std::string  classParsFileName;
    
    CMOutput outValues;
    int rad1, rad2, rad3, rad4;
   
};


