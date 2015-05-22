//
//  CMDetect.cpp
//  Color Marker
//
//  Copyright (c) 2012 Roberto Manduchi, Homayoun Bagherinia, James Coughlan 
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

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include "CMDetect.hpp"
#include <libxml/tree.h>
#include <libxml/parser.h>
#include "assert.h"

using namespace std;

#define COLOR_CHANNELS  3   // 4 for iPhone, 3 for RGB
#define DIAG_RATIO  6
#define MIN_HBORD 15

// Constructor with both userParsFileName and classParsFileName in

CMDetect::CMDetect(string _userParsFileName, string _classParsFileName){
    
    userParsFileName = _userParsFileName;
    classParsFileName = _classParsFileName;
    assert(ParseUserParsXML());
    assert(ParseClassifiersParsXML());
    ///
    LTF = new unsigned char [MAX_CASCADE*256*256];
    LoadPars();
    PermShift();
    LoadTable();
    ptr = 0;
    P_SHIFT = 8;
    SCAN_STEP = 2;
    N_DIAG_RATIO = DIAG_RATIO;
    N_MIN_HBOARD = MIN_HBORD;
    N_CHANNELS = COLOR_CHANNELS;
}

// Constructor with only userParsFileName (classParsFileName is contained in userParsFileName)
// DEPRECATED
//
//CMDetect::CMDetect(string _userParsFileName){
//    
//    userParsFileName = _userParsFileName;
//    assert(ParseUserParsXML());
//    assert(ParseClassifiersParsXML());
//    ///
//    LTF = new unsigned char [MAX_CASCADE*256*256];
//    LoadPars();
//    PermShift();
//    LoadTable();
//    ptr = 0;
//    P_SHIFT = 8;
//    SCAN_STEP = 2;
//    N_DIAG_RATIO = DIAG_RATIO;
//    N_MIN_HBOARD = MIN_HBORD;
//    N_CHANNELS = COLOR_CHANNELS;
//}
//

CMDetect::~CMDetect(){
    
    delete [] LTF;
}

// This is the entry port where we access the image data (basically an accessor method)
int CMDetect::AccessImage(unsigned char* thePtr, int theWidth,int theHeight, int theWidthStep){
    ptr = thePtr;
    IMAGE_W = theWidth;
    IMAGE_H = theHeight;
    WIDTH_STEP = theWidthStep;
    return 1;
}

inline unsigned char* CMDetect::pixPtr(int x, int y, unsigned char* origin)
{
	return origin + y*WIDTH_STEP + N_CHANNELS*x;
}


// This loads all of the internal parameters. 
int CMDetect::LoadPars(){
    TOL_SHIFT_RATIO = 3;         
    PT_CLUSTER_THRESHOLD = 1; // RM test 11/11; was 15;  // RM test 9/18
    PT_DISTANCE_THRESHOLD2 = 8; // RM test 11//1; was 4;
    MAX_PIXELS1 = (int)(MAX_PIXELS-5);      // why???
    
    CONSISTENCYCHECK1 = 0;
    CONSISTENCYCHECK2 = 1;
    CONSISTENCYCHECK3 = 0;
    
    
    // order: W B BL R
    
    colClass[0]  =  _ColInd(0, 1, 2);
    colClass[1]  =  _ColInd(2, 1, 2);
    colClass[2]  =  _ColInd(3, 1, 2);
    colClass[3]  =  _ColInd(0, 1, 1);
    colClass[4]  =  _ColInd(2, 1, 1);
    colClass[5]  =  _ColInd(3, 1, 1);
    colClass[6]  =  _ColInd(0, 1, 0);
    colClass[7]  =  _ColInd(2, 1, 0);
    colClass[8]  =  _ColInd(3, 1, 0);
    colClass[9]  =  _ColInd(2, 0, 2);
    colClass[10] =  _ColInd(3, 0, 2);
    colClass[11] =  _ColInd(2, 0, 1);
    colClass[12] =  _ColInd(3, 0, 1);
    colClass[13] =  _ColInd(2, 0, 0);
    colClass[14] =  _ColInd(3, 0, 0);
    colClass[15] =  _ColInd(3, 2, 2);
    colClass[16] =  _ColInd(3, 2, 1);
    colClass[17] =  _ColInd(3, 2, 0);
    
    return 1;
}


// Reads user parameters
int CMDetect::ParseUserParsXML()
{
    xmlDoc *doc = NULL;
    xmlNode *root_element = NULL, *currNode;
    
    int result = 1;
    xmlChar* content;
    
    doc = xmlReadFile(userParsFileName.c_str(), NULL, 0);
    
    if (doc == NULL) {
        cout << "Cannot parse parameter file "<< userParsFileName;
        goto CLEANUP;
    }
    
    // Get the root element node
    root_element = xmlDocGetRootElement(doc);
    
    currNode = root_element->xmlChildrenNode;
    while ((currNode != NULL) && (xmlStrcmp(currNode->name, (const xmlChar *) "Permutations"))) {
        currNode = currNode->next;
    }
    
    if (currNode == NULL) {
        result = 0;
        goto CLEANUP;
    }
    // cout << currNode->name << '\n';
    currNode = currNode->xmlChildrenNode;
    
    while ((currNode != NULL) && (xmlStrcmp(currNode->name, (const xmlChar *) "NumberOfPermutations"))) {
        currNode = currNode->next;
    }
    if (currNode == NULL) {
        result = 0;
        goto CLEANUP;
    }
    
    content = xmlNodeListGetString(doc, currNode->xmlChildrenNode, 1);
    
    // cout << content << '\n';
    
    nPerms = atoi((char *)content);
    xmlFree(content);

//    // reset
    currNode = currNode->parent->xmlChildrenNode;
    
    for (int id=0; id<nPerms; id++) {
        // reset
        // currNode = currNode->parent->xmlChildrenNode;

        while ((currNode != NULL) && (xmlStrcmp(currNode->name, (const xmlChar *) "PermutationIndex"))) {
            currNode = currNode->next;
        }
        if (currNode == NULL) {
            result = 0;
            goto CLEANUP;
        }
        
        content = xmlNodeListGetString(doc, currNode->xmlChildrenNode, 1);
        
        // cout << content << '\n';
        
        permIndices[id] = atoi((char *)content);
        xmlFree(content);

        if (id < nPerms-1) {
            currNode = currNode->next;
        }
        
    }
    
    currNode = currNode->parent->parent->children;
   
    if (classParsFileName.empty()) {
        // it was not initialized when the detector was created
            while ((currNode != NULL) && (xmlStrcmp(currNode->name, (const xmlChar *) "ClassifierParameterFileName"))) {
                currNode = currNode->next;
            }
            if (currNode == NULL) {
                result = 0;
               goto CLEANUP;
            }
        
            content = xmlNodeListGetString(doc, currNode->xmlChildrenNode, 1);
        
            classParsFileName = (char *) content;
            classParsFileName.erase(0, classParsFileName.find_first_not_of(" \t\r\n\v\f"));
            classParsFileName.erase(classParsFileName.find_last_not_of(" \t\r\n\v\f")+1);
            xmlFree(content);

    }

    // reset
    currNode = currNode->parent->xmlChildrenNode;
    while ((currNode != NULL) && (xmlStrcmp(currNode->name, (const xmlChar *) "ProbeHalfSpacing"))) {
        currNode = currNode->next;
    }
    if (currNode != NULL) {
        // if we don't find it, we'll use the default (8)
        content = xmlNodeListGetString(doc, currNode->xmlChildrenNode, 1);
        
        P_SHIFT = atoi((char *)content);
        xmlFree(content);
    }
    
    // reset
    currNode = currNode->parent->xmlChildrenNode;
    while ((currNode != NULL) && (xmlStrcmp(currNode->name, (const xmlChar *) "CascadeLength"))) {
        currNode = currNode->next;
    }
    if (currNode == NULL) {
        result = 0;
        goto CLEANUP;
    }
    
    content = xmlNodeListGetString(doc, currNode->xmlChildrenNode, 1);
    
    //    cout << content << '\n';
    
    cascadeLength = atoi((char *)content);
    xmlFree(content);
    
    if (cascadeLength > 18) {
        cout << "cascadeLength was set to a value higher than 18 - reduced to 18\n";
        cascadeLength = 18;
    }
    
CLEANUP:
    xmlFreeDoc(doc);
    xmlCleanupParser();
    
    return result;
}



// Reads classifier parameters generated by Matlab code
int CMDetect::ParseClassifiersParsXML()
{
    xmlDoc *doc = NULL;
    xmlNode *root_element = NULL, *currNode;
    
    int result = 1;
 
    doc = xmlReadFile(classParsFileName.c_str(), NULL, 0);
    
    if (doc == NULL) {
        cout << "Cannot parse parameter file "<< classParsFileName;
        result = 0;
        goto CLEANUP;
    }
    
    // Get the root element node
    root_element = xmlDocGetRootElement(doc);
    
    // Find "ClassifierParameters"
    
    currNode = root_element->xmlChildrenNode;
    while ((currNode != NULL) && (xmlStrcmp(currNode->name, (const xmlChar *) "ClassifierParameters"))) {
            currNode = currNode->next;
    }
    
    if (currNode == NULL) {
        result = 0;
        goto CLEANUP;
    }
    // cout << currNode->name << '\n';
    currNode = currNode->xmlChildrenNode;
    
    for (int i = 0; i<18;i++){
        while ((currNode != NULL) && (xmlStrcmp(currNode->name, (const xmlChar *) "Classifier"))) {
            currNode = currNode->next;
        }
        if (currNode == NULL) {
            result = 0;
            goto CLEANUP;
        }
        // get the classifier ID
        xmlAttr* property = currNode->properties;
        xmlChar* content;
        content = xmlNodeListGetString(doc, property->xmlChildrenNode, 1);
        
        classID[i]= atoi((char *)content);
        // cout << "Classifier ID "<<content << '\n';
        
        xmlFree(content);
        
        // now look for the parameters
        currNode = currNode->xmlChildrenNode;
 
        while ((currNode != NULL) && (xmlStrcmp(currNode->name, (const xmlChar *) "m"))) {
            currNode = currNode->next;
        }
        if (currNode == NULL) {
            result = 0;
            goto CLEANUP;
        }
        content = xmlNodeListGetString(doc, currNode->xmlChildrenNode, 1);
        classPar_m[classID[i]]= atof((char *)content);
       
       // cout << "m " << content << '\n';
        
        xmlFree(content);
        while ((currNode != NULL) && (xmlStrcmp(currNode->name, (const xmlChar *) "b1"))) {
            currNode = currNode->next;
        }
        if (currNode == NULL) {
            result = 0;
            goto CLEANUP;
        }
        content = xmlNodeListGetString(doc, currNode->xmlChildrenNode, 1);
        classPar_b1[classID[i]]= atof((char *)content);

        //cout << "b1 " << content << '\n';
        
        xmlFree(content);
        while ((currNode != NULL) && (xmlStrcmp(currNode->name, (const xmlChar *) "b2"))) {
            currNode = currNode->next;
        }
        if (currNode == NULL) {
            result = 0;
            goto CLEANUP;
        }
        content = xmlNodeListGetString(doc, currNode->xmlChildrenNode, 1);
        classPar_b2[classID[i]]= atof((char *)content);
        
        //cout << "b2 " << content << '\n';
        
        xmlFree(content);
        while ((currNode != NULL) && (xmlStrcmp(currNode->name, (const xmlChar *) "t_min_1"))) {
            currNode = currNode->next;
        }
        if (currNode == NULL) {
            result = 0;
            goto CLEANUP;
        }
        content = xmlNodeListGetString(doc, currNode->xmlChildrenNode, 1);
        classPar_t_min_1[classID[i]]= atoi((char *)content);
        
        cout << "t_min_1 " << content << '\n';
        
        xmlFree(content);
        while ((currNode != NULL) && (xmlStrcmp(currNode->name, (const xmlChar *) "t_min_2"))) {
            currNode = currNode->next;
        }
        if (currNode == NULL) {
            result = 0;
            goto CLEANUP;
        }
        content = xmlNodeListGetString(doc, currNode->xmlChildrenNode, 1);
        classPar_t_min_2[classID[i]]= atoi((char *)content);
        
        cout << "t_min_2 " << content << '\n';

        xmlFree(content);
        while ((currNode != NULL) && (xmlStrcmp(currNode->name, (const xmlChar *) "t_max_1"))) {
            currNode = currNode->next;
        }
        if (currNode == NULL) {
            result = 0;
            goto CLEANUP;
        }
        content = xmlNodeListGetString(doc, currNode->xmlChildrenNode, 1);
        classPar_t_max_1[classID[i]]= atoi((char *)content);
        
        cout << "t_max_1 " << content << '\n';
        
        
        xmlFree(content);
        while ((currNode != NULL) && (xmlStrcmp(currNode->name, (const xmlChar *) "t_max_2"))) {
            currNode = currNode->next;
        }
        if (currNode == NULL) {
            result = 0;
            goto CLEANUP;
        }
        content = xmlNodeListGetString(doc, currNode->xmlChildrenNode, 1);
        classPar_t_max_2[classID[i]]= atoi((char *)content);
        
        cout << "t_max_2 " << content << '\n';
        
        xmlFree(content);
       
        //
        currNode = currNode->parent;
        currNode = currNode->next;
    }
    
    CLEANUP:
    xmlFreeDoc(doc);
    xmlCleanupParser();
    
    return result;
}

/////////////
// overrule
///////////
int CMDetect::SetMarkerID(int markerID)
{
    permIndices[0] = markerID;
    nPerms = 1;
}

// This is the engine of the system. It detects candidate points, computes "clusters" of neairby points, selects the winner, computes a segmentation of the marker and applies consistency checks.

int CMDetect::FindTarget()

{
    // indices of bgr begining of a pixels x, y
    //	a b
    //  c d
    
    int x, y;

    // ps_a1[] etc must be first initalized by PermShift();

    // detected pixels for considerd permutations
    CMPoint XYArray[24][MAX_PIXELS];
    
    // number of detected pixels for considerd permutations
    int cntr[24];
    
    // their scores (number of neigboring winners)
    int scores[24][MAX_PIXELS];
    
    memset(cntr, 0, 24*sizeof(int)); ///
    
    unsigned char * ptrArr[4];
    
//    unsigned char cArr[4][3];
    
    int NC_SS = N_CHANNELS * SCAN_STEP;
    
    // 1- Find candidate marker points
    for (int iP=0; iP<nPerms; iP++) {
        int currPerm = permIndices[iP];
        for (y=P_SHIFT; y<IMAGE_H-P_SHIFT; y+=SCAN_STEP) {
            ptrArr[0] = ptr + (y+ps_a2[currPerm])*WIDTH_STEP + N_CHANNELS * (P_SHIFT+ps_a1[currPerm]) ;
            ptrArr[1] = ptr + (y+ps_b2[currPerm])*WIDTH_STEP + N_CHANNELS * (P_SHIFT+ps_b1[currPerm]) ;
            ptrArr[2] = ptr + (y+ps_c2[currPerm])*WIDTH_STEP + N_CHANNELS * (P_SHIFT+ps_c1[currPerm]) ;
            ptrArr[3] = ptr + (y+ps_d2[currPerm])*WIDTH_STEP + N_CHANNELS * (P_SHIFT+ps_d1[currPerm]) ;
            
            for (x=P_SHIFT; x<IMAGE_W-P_SHIFT; x+=SCAN_STEP) {

//                // debug RM 8/24 - to see saturated pixels in blue
//                unsigned char * a = pixPtr(x, y, ptr);
//                if ((a[0]>252)||(a[1]>252)||(a[2]>252)) {
//                    a[0] = 255; a[1] = 0; a[2] = 0;
//                }
                
//                // test RM 11/11 - tried copying the color values 
//                cArr[0][0] = *(ptrArr[0]); cArr[0][1] = *(ptrArr[0]+1); cArr[0][2] = *(ptrArr[0]+2);
//                cArr[1][0] = *(ptrArr[1]); cArr[1][1] = *(ptrArr[1]+1); cArr[1][2] = *(ptrArr[1]+2);
//                cArr[2][0] = *(ptrArr[2]); cArr[2][1] = *(ptrArr[2]+1); cArr[2][2] = *(ptrArr[2]+2);
//                cArr[3][0] = *(ptrArr[3]); cArr[3][1] = *(ptrArr[3]+1); cArr[3][2] = *(ptrArr[3]+2);

                int found = 1;
                for (int id=0; id<cascadeLength; id++) {
                    unsigned int cID = classID[id];
                    CMColInd colInd = colClass[cID];
                    
                    if (LTF[cID * 256 * 256 + *(ptrArr[colInd.ind1]+colInd.ind3) * 256 + *(ptrArr[colInd.ind2]+colInd.ind3)]){
                    
                    // RM 11/11 - using shift operators instead of multiplications 
//                    if (LTF[(int)(cID << 16) + (int)(*(ptrArr[colInd.ind1]+colInd.ind3) << 8) + *(ptrArr[colInd.ind2]+colInd.ind3)]){

                        found = 0;
                        break;
                    }
                }
                if (found){
                     if ( cntr[currPerm]<MAX_PIXELS1 ){
                        XYArray[currPerm][cntr[currPerm]].iX = x;
                        XYArray[currPerm][cntr[currPerm]].iY = y;
                        cntr[currPerm]++;
                    }
                    // debug RM 8/24
//                   unsigned char * a = pixPtr(x, y, ptr);
//                   a[0] = 0; a[1] = 0; a[2] = 255;
                    
                    
                }
                ptrArr[0] += NC_SS;
                ptrArr[1] += NC_SS;
                ptrArr[2] += NC_SS;
                ptrArr[3] += NC_SS;
            }
        }

    }
    // 2- Compute clusters of points around detected candidates and selects winner permutation

    int maxScore = 0;		
    int indMaxScore = 0;
    for (int iP=0; iP<nPerms; iP++) {
        int currPerm = permIndices[iP];
        for(int i = 0; i < cntr[currPerm]; i++) {
            scores[currPerm][i]=0;
            for(int j = 0; j < cntr[currPerm]; j++) {
                if(abs(XYArray[currPerm][i].iY - XYArray[currPerm][j].iY) < PT_DISTANCE_THRESHOLD2) {
                    if(abs(XYArray[currPerm][i].iX - XYArray[currPerm][j].iX) < PT_DISTANCE_THRESHOLD2){
                        scores[currPerm][i]++;
                        if (scores[currPerm][i] > maxScore){
                            maxScore = scores[currPerm][i];
                            indMaxScore = i;
                            perm = currPerm;
                        }
                    }
                }
            }
        }
    }
    // now perm has the winning permutation
    // XYArray[perm][indMaxScore] has the point with the max number of neighbors (equal to maxScore)
    // cntr[perm] has the number of detected pixels for the winning permutation
    
    if (maxScore < PT_CLUSTER_THRESHOLD)
        return 0;  // no points found
    
    unsigned char *pix;

    // 3- Compute segmentation
    // This part originally written by James Coughlan

    // compute center=average of detected points:
    int cx=0, cy=0, count=0, toler = TOL_SHIFT_RATIO * P_SHIFT;

    // look at all points within a neighborhood (determined by tolerance) of (ix_best,iy_best).
    // consider only the points with large score 

    for (int i = 0; i < cntr[perm]; i++)
    {
        if ((scores[perm][i]>PT_CLUSTER_THRESHOLD) &&
            (abs(XYArray[perm][i].iX-XYArray[perm][indMaxScore].iX)<toler) &&
            (abs(XYArray[perm][i].iY-XYArray[perm][indMaxScore].iY)<toler))
        {
            cx += XYArray[perm][i].iX;
            cy += XYArray[perm][i].iY;
            count++;
        }
    }

    if (count==0) {
        // not sure this is possible
        return 0;
    }

    cx=cx/count; 
    cy=cy/count;

    // color sectors A,B,C,D in color target

    // find representative RGB values in each sector:

    int A[3], B[3], C[3], D[3];
    short A_[3], B_[3], C_[3], D_[3];
    int i,j,ix,iy;
    for (i=0; i<3;i++)
        A[i] = B[i] = C[i] = D[i] = 0;

    // get color averages: 
    for (i = 0, count = 0; i < cntr[perm]; i++)
    {
        // look at all points within a neighborhood (determined by tolerance) of (ix_best,iy_best).
        // consider only the points with large score 
       if ((scores[perm][i]>PT_CLUSTER_THRESHOLD) &&
           (abs(XYArray[perm][i].iX-XYArray[perm][indMaxScore].iX)<toler) &&
           (abs(XYArray[perm][i].iY-XYArray[perm][indMaxScore].iY)<toler))
        {
            count++;
            
            //sector A:
            ix=XYArray[perm][i].iX+ps_a1[perm];
            iy=XYArray[perm][i].iY+ps_a2[perm]; //goat
            pix=pixPtr(ix, iy, ptr);
            A[0] += pix[0]; A[1] += pix[1]; A[2] += pix[2];
            
            //sector B:
            ix=XYArray[perm][i].iX+ps_b1[perm]; //goat
            iy=XYArray[perm][i].iY+ps_b2[perm];
            pix=pixPtr(ix, iy, ptr);
            B[0] += pix[0]; B[1] += pix[1]; B[2] += pix[2];
            
            //sector C:
            ix=XYArray[perm][i].iX+ps_c1[perm]; //goat
            iy=XYArray[perm][i].iY+ps_c2[perm];
            pix=pixPtr(ix, iy, ptr);
            C[0] += pix[0]; C[1] += pix[1]; C[2] += pix[2];
            
            //sector D:
            ix=XYArray[perm][i].iX+ps_d1[perm];
            iy=XYArray[perm][i].iY+ps_d2[perm]; //goat
            pix=pixPtr(ix, iy, ptr);
            D[0] += pix[0]; D[1] += pix[1]; D[2] += pix[2];
        }
    }

    for (i=0; i<3;i++)
    { // get average - we already know that count > 0
        A_[i] = A[i]/count;
        B_[i] = B[i]/count;
        C_[i] = C[i]/count;
        D_[i] = D[i]/count;
    }
    
    FloodSegment(cx,cy,A_,B_,C_,D_);

    
    // 4 - Compute keypoints
    
    ComputeKeypoints(cx, cy);
    
    // Color keypoints
    
    for (int iy = max(0,outValues.top.iY-3); iy<= min(IMAGE_H-1,outValues.top.iY+3); iy++) {
        for (int ix = max(0,outValues.top.iX-3); ix<= min(IMAGE_W-1,outValues.top.iX+3); ix++) {
            unsigned char * pix = pixPtr(ix, iy, ptr);
            pix[0]=pix[1]=0; pix[2]=255;
        }
    }
    for (int iy = max(0,outValues.left.iY-3); iy<= min(IMAGE_H-1,outValues.left.iY+3); iy++) {
        for (int ix = max(0,outValues.left.iX-3); ix<= min(IMAGE_W-1,outValues.left.iX+3); ix++) {
            unsigned char * pix = pixPtr(ix, iy, ptr);
            pix[0]=pix[1]=0; pix[2]=255;
        }
    }
    for (int iy = max(0,outValues.right.iY-3); iy<= min(IMAGE_H-1,outValues.right.iY+3); iy++) {
        for (int ix = max(0,outValues.right.iX-3); ix<= min(IMAGE_W-1,outValues.right.iX+3); ix++) {
            unsigned char * pix = pixPtr(ix, iy, ptr);
            pix[0]=pix[1]=0; pix[2]=255;
        }
    }
    for (int iy = max(0,outValues.bottom.iY-3); iy<= min(IMAGE_H-1,outValues.bottom.iY+3); iy++) {
        for (int ix = max(0,outValues.bottom.iX-3); ix<= min(IMAGE_W-1,outValues.bottom.iX+3); ix++) {
            unsigned char * pix = pixPtr(ix, iy, ptr);
            pix[0]=pix[1]=0; pix[2]=255;
        }
    }
    for (int iy = max(0,outValues.center.iY-3); iy<= min(IMAGE_H-1,outValues.center.iY+3); iy++) {
        for (int ix = max(0,outValues.center.iX-3); ix<= min(IMAGE_W-1,outValues.center.iX+3); ix++) {
            unsigned char * pix = pixPtr(ix, iy, ptr);
            pix[0]=pix[1]=0; pix[2]=255;
        }
    }
    
    // 5- Consistency checks

    int SemiMajorAxisY1 = 0, SemiMajorAxisX1 = 0, SemiMajorAxisY2 = 0, SemiMajorAxisX2 = 0;
    
    CMPoint SemiAxis;
    int Area=0;


    // Now estimate the vertical semiaxis 1
    for (i=cy-rad1; i<cy+rad3; i++)
        for (j=cx-rad2;j<=cx+rad4;j++)
            if ((pixPtr(j, i, ptr))[0] == 1)
            {
                Area++;
            }
    SemiMajorAxisX1 = rad2;
    SemiMajorAxisX2 = rad4;
    SemiMajorAxisY1 = rad1;
    SemiMajorAxisY2 = rad3;
    SemiAxis.iX = (rad2 > rad4 ? rad2 :rad4);
    SemiAxis.iY = (rad1 > rad3 ? rad1 :rad3);

    // Consistency check 1: axis similarity
    if (CONSISTENCYCHECK1) {
        if (! ((2*SemiMajorAxisX1 < 3*SemiMajorAxisX2) && (2*SemiMajorAxisX2 < 3*SemiMajorAxisX1) &&
        (2*SemiMajorAxisY1 < 3*SemiMajorAxisY2) && (2*SemiMajorAxisY2 < 3*SemiMajorAxisY1)) )
            return 0;
    }
    
    // Consistency check 2: area
    if (CONSISTENCYCHECK2) {
        float 	theoreticalArea;
        theoreticalArea = (3./4.)*3.14*(float)SemiAxis.iX*(float)SemiAxis.iY;
        if (! (((float)Area > (3./4.)*theoreticalArea) && (theoreticalArea > (3./4.)*(float)Area)) )
            return 0;
    }
                
    // Consistency check 3: perimeter
    if (CONSISTENCYCHECK3) {
        // First compute theoreticalPerimeter using the naive formula
        float theoreticalPerimeter = 0.;
        theoreticalPerimeter  = (1. + (3./4.)*3.14) * ((float)SemiAxis.iX + (float)SemiAxis.iY);
        
        // Compute the perimeter: any point that borders a "white" point is part of the contour
        int Perimeter=0;
        for (i=cy-rad1; i<cy+rad3; i++) {
            for (j=cx-rad2;j<=cx+rad4;j++) {
                if ((pixPtr(j, i, ptr))[0] == 1) {
                    if ((pixPtr(j-1, i, ptr)[0] != 1) ||
                        (pixPtr(j+1, i, ptr)[0] != 1) ||
                        (pixPtr(j, i-1, ptr)[0] != 1) ||
                        (pixPtr(j, i+1, ptr)[0] != 1))
                    {
                        Perimeter++;
                    }
                }
            }
        }
        
        if (! (((float)Perimeter > (6./7.)*theoreticalPerimeter) && (theoreticalPerimeter > (6./7.)*(float)Perimeter)))
            return 0;
    }
        
    return 1;
}

////////
int     CMDetect::FindOppositeTopBottom(CardPoint topOrBottom)
{
    int xCand, yCand;
    float   Axy;
    
    int otherY, otherX;
    
    if (topOrBottom==Top) {
        otherY = outValues.bottom.iY;   // must have been computed already!
        otherX = outValues.bottom.iX;   // must have been computed already!
    }
    else {
        otherY = outValues.top.iY;
        otherX = outValues.top.iX;
    }
    
    if (outValues.center.iY != otherY) {
        Axy = (float)(otherX - outValues.center.iX) / (float)(otherY - outValues.center.iY);
    }
    
    if (topOrBottom==Top)
        yCand = outValues.center.iY - rad1;
    else
        yCand = outValues.center.iY + rad3;
    
    if (outValues.center.iY != otherY)
        xCand =  outValues.center.iX - (int) (Axy * (float)(outValues.center.iY - yCand));
    else
        xCand =  outValues.center.iX;
    
    int ixt = max(0,min(IMAGE_W-1,xCand));
    int iyt = max(0,min(IMAGE_H-1,yCand));
    
    bool theSign = (pixPtr(ixt, iyt, ptr)[0] == 1);
    int theStep;
    
    if (topOrBottom==Top)
        theSign ? theStep = -1: theStep = 1;
    else
        theSign ? theStep = 1: theStep = -1;
    
    while ((pixPtr(ixt, iyt, ptr)[0] == 1) == theSign) {
        iyt += theStep;
        if (outValues.center.iX != otherX)
            ixt =  outValues.center.iX - (int) (Axy * (float)(outValues.center.iY - iyt));
        if ((ixt >= 0) && (ixt < IMAGE_W) && (iyt >= 0) && (iyt < IMAGE_H) ) {
            if ((pixPtr(ixt, iyt, ptr)[0] == 1) != theSign) {
                if (topOrBottom==Top) {
                    outValues.top.iX = ixt;
                    outValues.top.iY = iyt;
                }
                else {
                    outValues.bottom.iX = ixt;
                    outValues.bottom.iY = iyt;
                }
                break;
            }
        }
        else {
            if (topOrBottom==Top) {
                outValues.top.iX = 0;
                outValues.top.iY = 0;
            }
            else {
                outValues.bottom.iX = 0;
                outValues.bottom.iY = 0;
            }
            break;
        }
        
    }
    return 1;
}
    ////////
int  CMDetect::FindOppositeLeftRight(CardPoint leftOrRight)
    {
        int xCand, yCand;
        float Ayx;
        
        int otherY, otherX;
        
        if (leftOrRight==Left) {
            otherY = outValues.right.iY;   // must have been computed already!
            otherX = outValues.right.iX;   // must have been computed already!
        }
        else {
            otherY = outValues.left.iY;
            otherX = outValues.left.iX;
        }
       if (outValues.center.iX != otherX) {
            Ayx = (float)(otherY - outValues.center.iY) / (float)(otherX - outValues.center.iX);
        }
        
        if (leftOrRight==Left)
            xCand = outValues.center.iX - rad2;
        else
            xCand = outValues.center.iX + rad4;
        
        if (outValues.center.iX != otherX)
            yCand =  outValues.center.iY - (int) (Ayx * (float)(outValues.center.iX - xCand));
        else
            yCand =  outValues.center.iY;
        
        int ixt = max(0,min(IMAGE_W-1,xCand));
        int iyt = max(0,min(IMAGE_H-1,yCand));
        

        bool theSign = (pixPtr(ixt, iyt, ptr)[0] == 1);
        int theStep;
        
        if (leftOrRight==Left)
            theSign ? theStep = -1: theStep = 1;
        else
            theSign ? theStep = 1: theStep = -1;
        
        while ((pixPtr(ixt, iyt, ptr)[0] == 1) == theSign) {
            ixt += theStep;
            if (outValues.center.iY != otherY)
                iyt =  outValues.center.iY - (int) (Ayx * (float)(outValues.center.iX - ixt));
            if ((ixt >= 0) && (ixt < IMAGE_W) && (iyt >= 0) && (iyt < IMAGE_H) ) {
                if ((pixPtr(ixt, iyt, ptr)[0] == 1) != theSign) {
                    if (leftOrRight==Left) {
                        outValues.left.iX = ixt;
                        outValues.left.iY = iyt;
                    }
                    else {
                        outValues.right.iX = ixt;
                        outValues.right.iY = iyt;
                    }
                    break;
                }
            }
            else {
                if (leftOrRight==Left) {
                    outValues.left.iX = 0;
                    outValues.left.iY = 0;
                }
                else {
                    outValues.right.iX = 0;
                    outValues.right.iY = 0;
                }
                break;
            }
            
        }
        return 1;
    }

///
int CMDetect::FindKeyPoint(KeyPointType whichKeyPoint, CardPoint leftOrRight, CardPoint topOrBottom)
{
    int maxDiag = 0, countDiag;
    int xStart, xEnd, yStart, yEnd;
    int s1,s2,s3,s4;
    int nDiag;
    
    if (whichKeyPoint == LeftRightHorn)
    {
        if (leftOrRight==Left) {
            int hBord = min(rad2,max(N_MIN_HBOARD,rad2/4));
            xStart = max(0,outValues.center.iX - rad2 - hBord);
            xEnd = min(IMAGE_W-1,outValues.center.iX - rad2 + hBord);
        }
        else {
            int hBord = min(rad4,max(N_MIN_HBOARD,rad4/4));
            xStart = max(0,outValues.center.iX+ rad4 - hBord);
            xEnd = min(IMAGE_W-1,outValues.center.iX+ rad4 + hBord);
        }
        
        yStart = max(0,outValues.center.iY-rad1);
        yEnd =  min(IMAGE_H-1,outValues.center.iY+rad3);
        
        nDiag = (xEnd - xStart)/N_DIAG_RATIO;

        s1 = s2 = s3 = s4 = -1;
        
        if (leftOrRight==Left) 
            if (topOrBottom==Top) 
                s2 = 1;
            else 
                s4 = 1;
        else 
            if (topOrBottom==Top) 
                s1 = 1;
            else 
                s3 = 1;
    }
    else if (whichKeyPoint == TopBottomHorn)
    {
        if (topOrBottom==Top) {
            int hBord = min(rad1,max(N_MIN_HBOARD,rad1/4));
            yStart = max(0,outValues.center.iY - rad1 - hBord);
            yEnd = min(IMAGE_H-1,outValues.center.iY - rad1 + hBord);
        }
        else {
            int hBord = min(rad3,max(N_MIN_HBOARD,rad3/4));
            yStart = max(0,outValues.center.iY + rad3 - hBord);
            yEnd = min(IMAGE_H-1,outValues.center.iY + rad3 + hBord);
        }
        
        xStart = max(0,outValues.center.iX-rad2);
        xEnd =  min(IMAGE_W-1,outValues.center.iX+rad4);

        nDiag = (yEnd - yStart)/N_DIAG_RATIO;

        s1 = s2 = s3 = s4 = -1;
        
        if (topOrBottom==Top)
            if (leftOrRight==Left)
                s3 = 1;
            else
                s4 = 1;
        else
            if (leftOrRight==Left)
                s1 = 1;
            else
                s2 = 1;
    }
    else    // Center
    {
        int hBord = min((rad1+rad2+rad3+rad4)/4,max(N_MIN_HBOARD,rad2/4));
        
        xStart = max(0,outValues.center.iX - hBord);
        xEnd = min(IMAGE_W-1, outValues.center.iX + hBord);
        
        yStart = max(0, outValues.center.iY - hBord);
        yEnd = min(IMAGE_H-1, outValues.center.iY + hBord);
        
        nDiag = (yEnd - yStart)/N_DIAG_RATIO;

        s1 = s2 = s3 = s4 = 1;
        
        if (topOrBottom==Top)
            if (leftOrRight==Left)
                s1 = -1;
            else
                s2 = -1;
        else
            if (leftOrRight==Left)
                s3 = -1;
            else
                s4 = -1;
    }
    
    
    // These should placed in a parameter file
    if (nDiag < 5)
        nDiag = 5;
    
    if (nDiag > 200) {
        nDiag = 200;
    }
    
    yEnd -= nDiag;
    if (yEnd < yStart)
        return 0;
    
    xEnd -= nDiag;
    if (xEnd < xStart)
        return 0;
    
    // Prepare image
    
    
    // upper triangular
    for (int ix = xStart; ix <= xEnd; ix++){
        // first fill
        pixPtr(ix,yStart,ptr)[1] = (unsigned char) (pixPtr(ix,yStart,ptr)[0] == 1);
        
        // we already know that the following is safe
        for (int j=1; j<=nDiag; j++) {
            pixPtr(ix,yStart,ptr)[1] += (unsigned char) (pixPtr(ix+j,yStart+j,ptr)[0] == 1);
        }
        
        //now go!
        int jEnd = min(xEnd-ix, yEnd-yStart);
        for (int j = 1; j <= jEnd; j++) {
            pixPtr(ix+j,yStart+j,ptr)[1] = pixPtr(ix+j-1,yStart+j-1,ptr)[1] + (unsigned char) (pixPtr(ix+j+nDiag,yStart+j+nDiag,ptr)[0] == 1);
            pixPtr(ix+j,yStart+j,ptr)[1] -= (unsigned char) (pixPtr(ix+j-1,yStart+j-1,ptr)[0] == 1);
        }
        
    }
    // lower triangular
    for (int iy = yStart+1; iy <= yEnd; iy++) {
        // first fill
        pixPtr(xStart,iy,ptr)[1] = (unsigned char) (pixPtr(xStart,iy,ptr)[0] == 1);
        for (int j=1; j<=nDiag; j++) {
            pixPtr(xStart,iy,ptr)[1] += (unsigned char) (pixPtr(xStart+j,iy+j,ptr)[0] == 1);
        }
        //now go!
        int jEnd = min(xEnd-xStart, yEnd-iy);
        for (int j = 1; j <= jEnd; j++) {
            pixPtr(xStart+j,iy+j,ptr)[1] = pixPtr(xStart+j-1,iy+j-1,ptr)[1] + (unsigned char) (pixPtr(xStart+j+nDiag,iy+j+nDiag,ptr)[0] == 1);
            pixPtr(xStart+j,iy+j,ptr)[1] -= (unsigned char) (pixPtr(xStart+j-1,iy+j-1,ptr)[0] == 1);
        }
        
    }
    
    // upper triangular
    for (int ix = xStart; ix <= xEnd; ix++){
        // first fill
        pixPtr(ix,yStart,ptr)[2] = (unsigned char) (pixPtr(ix,yStart,ptr)[0] == 1);
        
        // we already know that the following is safe
        for (int j=1; j<=nDiag; j++) {
            pixPtr(ix,yStart,ptr)[2] += (unsigned char) (pixPtr(ix-j,yStart+j,ptr)[0] == 1);
        }
        
        //now go!
        int jEnd = min(ix-xStart, yEnd-yStart);
        for (int j = 1; j <= jEnd; j++) {
            pixPtr(ix-j,yStart+j,ptr)[2] = pixPtr(ix-j+1,yStart+j-1,ptr)[2] + (unsigned char) (pixPtr(ix-j-nDiag,yStart+j+nDiag,ptr)[0] == 1);
            pixPtr(ix-j,yStart+j,ptr)[2] -= (unsigned char) (pixPtr(ix-j+1,yStart+j-1,ptr)[0] == 1);
        }
        
    }
    // lower triangular
    for (int iy = yStart+1; iy <= yEnd; iy++) {
        // first fill
        pixPtr(xEnd,iy,ptr)[2] = (unsigned char) (pixPtr(xEnd,iy,ptr)[0] == 1);
        for (int j=1; j<=nDiag; j++) {
            pixPtr(xEnd,iy,ptr)[2] += (unsigned char) (pixPtr(xEnd-j,iy+j,ptr)[0] == 1);
        }
        //now go!
        int jEnd = min(xEnd-xStart, yEnd-iy);
        for (int j = 1; j <= jEnd; j++) {
            pixPtr(xEnd-j,iy+j,ptr)[2] = pixPtr(xEnd-j+1,iy+j-1,ptr)[2] + (unsigned char) (pixPtr(xEnd-j-nDiag,iy+j+nDiag,ptr)[0] == 1);
            pixPtr(xEnd-j,iy+j,ptr)[2] -= (unsigned char) (pixPtr(xEnd-j+1,iy+j-1,ptr)[0] == 1);
        }
        
    }
    
    int currMaxX = 0, currMaxY = 0;
    
    for (int iy = yStart + nDiag; iy <= yEnd; iy++) {
        for (int ix = xStart + nDiag; ix < xEnd - nDiag; ix++) {
            countDiag =  s1 * (int) pixPtr(ix-nDiag, iy-nDiag, ptr)[1] +
            s2 * (int) pixPtr(ix+nDiag, iy-nDiag, ptr)[2] +
            s3 * (int) pixPtr(ix, iy, ptr)[2] +
            s4 * (int) pixPtr(ix, iy, ptr)[1];
            
            if (countDiag > maxDiag) {
                maxDiag = countDiag;
                currMaxX = ix;
                currMaxY = iy;
            }
        }
    }
    if (whichKeyPoint == LeftRightHorn)
    {
        if (leftOrRight==Left) {
            outValues.left.iX = currMaxX;
            outValues.left.iY = currMaxY;
        }
        else{
            outValues.right.iX = currMaxX;
            outValues.right.iY = currMaxY;
        }        
    }
    else if (whichKeyPoint == TopBottomHorn)
    {
        if (topOrBottom==Top) {
            outValues.top.iX = currMaxX;
            outValues.top.iY = currMaxY;
        }
        else{
            outValues.bottom.iX = currMaxX;
            outValues.bottom.iY = currMaxY;
        }        
    }
    else    // Center
    {
        outValues.center.iX = currMaxX;
        outValues.center.iY = currMaxY;
    }

    return 1;
}


/////
int CMDetect::FloodSegment(int cx, int cy, short *A, short *B,short *C,short *D)
{
    // region growing:
    //  RM prepare the image: value '1' in the first color channel means that pixel has been segmented as 'color' so we first set the first channel of all pixels that have it equal to 1 to 0
    
    for (int y=0; y<IMAGE_H; y++){
        for (int x=0; x<IMAGE_W; x++) {
            unsigned char * pix = pixPtr(x, y, ptr);
            if (*pix == 1)
                *pix = 0;
        }
    }
    
    //center segmented as color
    
    unsigned char *pix = pixPtr(cx, cy, ptr);
    
    pix[0]=1;
    
    int i0,j0;
    // propagate shell rad (inner) to rad+1 (outer), where rad=0,1,2,3,...
    int flag=1;
    rad1 =0; rad2 =0; rad3 =0; rad4 =0; // initial inner radius
    int foundSomething1 = 1, foundSomething2 = 1, foundSomething3 = 1, foundSomething4 = 1;
    
    ///
    int alphaB = ((A[0]*A[0]+A[1]*A[1]+A[2]*A[2]) - (B[0]*B[0]+B[1]*B[1]+B[2]*B[2]))/2;
    int alphaC = ((A[0]*A[0]+A[1]*A[1]+A[2]*A[2]) - (C[0]*C[0]+C[1]*C[1]+C[2]*C[2]))/2;
    int alphaD = ((A[0]*A[0]+A[1]*A[1]+A[2]*A[2]) - (D[0]*D[0]+D[1]*D[1]+D[2]*D[2]))/2;
    
    int betaB[3], betaC[3], betaD[3];
    
    for (int i=0;i<3;i++){
        betaB[i] = A[i] - B[i];
        betaC[i] = A[i] - C[i];
        betaD[i] = A[i] - D[i];
    }
    
    
    while ((foundSomething1||foundSomething2||foundSomething3||foundSomething4) &&
           (cx-rad2-2>=0)&&(cx+rad4+2<IMAGE_W)
           && (cy-rad1-2>=0)&&(cy+rad3+2<IMAGE_H))
    {
        flag=0;
        
        // Parts 1,2,3,4 of inner shell: top, left, bottom, right
        
        
        // XXX		#1 (top)
        // ...
        // ...
        i0=cy-rad1; foundSomething1 = 0;
        for (j0=cx-rad2; j0<=cx+rad4; j0++)
        {
            if ( (pixPtr(j0, i0, ptr))[0] == 1)
            { //if (i0,j0) has been included in segmentation:
                //look among neighbors in outer shell for non-whites (3 neighbors)
                int i=i0-1; //above
                
                for (int j=j0-2; j<=j0+2; j++)
                {
                    pix = pixPtr(j, i, ptr);
                    if (pix[0] != 1) {
                        if (pix[0]*betaB[0] + pix[1]*betaB[1] + pix[2]*betaB[2] < alphaB) {
                            pix[0]=1; pix[1]=255; pix[2]=255; foundSomething1=1;
                        }
                        else
                            if (pix[0]*betaC[0] + pix[1]*betaC[1] + pix[2]*betaC[2] < alphaC){
                                pix[0]=1; pix[1]=255; pix[2]=255; foundSomething1=1;
                            }
                            else
                                if (pix[0]*betaD[0] + pix[1]*betaD[1] + pix[2]*betaD[2] < alphaD){
                                    pix[0]=1; pix[1]=255; pix[2]=255; foundSomething1=1;
                                }
                    }
                } //end for j
            } //end if (i0,j0) has been included in segmentation
        } //end for j0 loop
        
        
        // X..		#2 (left)
        // X..
        // X..
        j0=cx-rad2;foundSomething2 = 0;
        for (i0=cy-rad1; i0<=cy+rad3; i0++)
        {
            if ((pixPtr(j0, i0, ptr))[0] == 1)
            { //if (i0,j0) has been included in segmentation:
                //look among neighbors in outer shell for non-whites (3 neighbors)
                int j=j0-1; //left
                
                for (int i=i0-2; i<=i0+2; i++)
                {
                    pix=pixPtr(j, i, ptr);
                    if (pix[0] != 1) {
                        if (pix[0]*betaB[0] + pix[1]*betaB[1] + pix[2]*betaB[2] < alphaB) {
                            pix[0]=1; pix[1]=255; pix[2]=255; foundSomething2=1;
                        }
                        else
                            if (pix[0]*betaC[0] + pix[1]*betaC[1] + pix[2]*betaC[2] < alphaC){
                                pix[0]=1; pix[1]=255; pix[2]=255; foundSomething2=1;
                            }
                            else
                                if (pix[0]*betaD[0] + pix[1]*betaD[1] + pix[2]*betaD[2] < alphaD){
                                    pix[0]=1; pix[1]=255; pix[2]=255; foundSomething2=1;
                                }
                        
                    }
                } //end for i
            } //end if (i0,j0) has been included in segmentation
        } //end for i0 loop
        
        
        // ...		#3 (bottom)
        // ...
        // XXX
        i0=cy+rad3;foundSomething3 = 0;
        for (j0=cx-rad2; j0<=cx+rad4; j0++)
        {
            if ((pixPtr(j0, i0, ptr))[0] == 1)
            { //if (i0,j0) has been included in segmentation:
                //look among neighbors in outer shell in outer shell for non-whites (3 neighbors)
                int i=i0+1; //below

                for (int j=j0-2; j<=j0+2; j++)
                {
                    pix = pixPtr(j, i, ptr);
                    if (pix[0] != 1) {
                        if (pix[0]*betaB[0] + pix[1]*betaB[1] + pix[2]*betaB[2] < alphaB) {
                            pix[0]=1; pix[1]=255; pix[2]=255; foundSomething3=1;
                        }
                        else
                            if (pix[0]*betaC[0] + pix[1]*betaC[1] + pix[2]*betaC[2] < alphaC){
                                pix[0]=1; pix[1]=255; pix[2]=255; foundSomething3=1;
                            }
                            else
                                if (pix[0]*betaD[0] + pix[1]*betaD[1] + pix[2]*betaD[2] < alphaD){
                                    pix[0]=1; pix[1]=255; pix[2]=255; foundSomething3=1;
                                }
                        
                    }
                } //end for j
            } //end if (i0,j0) has been included in segmentation
        } //end for j0 loop
        
        // ..X		#4 (right)
        // ..X
        // ..X
        j0=cx+rad4;foundSomething4 = 0;
        for (i0=cy-rad1; i0<=cy+rad3; i0++)
        {
            if ((pixPtr(j0, i0, ptr))[0] == 1)
            { //if (i0,j0) has been included in segmentation:
                //look among neighbors for non-whites (3 neighbors)
                int j=j0+1; //right

                for (int i=i0-2; i<=i0+2; i++)
                {
                    pix=pixPtr(j, i, ptr);
                    if (pix[0] != 1) {
                        if (pix[0]*betaB[0] + pix[1]*betaB[1] + pix[2]*betaB[2] < alphaB) {
                            pix[0]=1; pix[1]=255; pix[2]=255; foundSomething4=1;
                        }
                        else
                            if (pix[0]*betaC[0] + pix[1]*betaC[1] + pix[2]*betaC[2] < alphaC){
                                pix[0]=1; pix[1]=255; pix[2]=255; foundSomething4=1;
                            }
                            else
                                if (pix[0]*betaD[0] + pix[1]*betaD[1] + pix[2]*betaD[2] < alphaD){
                                    pix[0]=1; pix[1]=255; pix[2]=255; foundSomething4=1;
                                }
                    }
                } //end for i
            } //end if (i0,j0) has been included in segmentation
        } //end for i0 loop
        if (foundSomething1) rad1++;
        if (foundSomething2) rad2++;
        if (foundSomething3) rad3++;
        if (foundSomething4) rad4++;
        //		rad++;
    } // end while loop
    
    return 1;
}

/////
int CMDetect::ComputeKeypoints(int cx, int cy)
{
    // Find 5 points (center, top, bottom, left, right)
    
    outValues.perm = perm;
    outValues.center.iX = cx;
    outValues.center.iY = cy;
    
    outValues.left.iX =  outValues.left.iY = 0;
    outValues.right.iX =  outValues.right.iY = 0;
    outValues.top.iX =  outValues.top.iY = 0;
    outValues.bottom.iX =  outValues.bottom.iY = 0;
    switch (perm) {
            // white sector top left
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
            FindKeyPoint(Center, Left, Top);
            FindKeyPoint(LeftRightHorn, Left, Bottom);
            FindKeyPoint(TopBottomHorn, Right, Top);
            FindOppositeLeftRight(Right);
            FindOppositeTopBottom(Bottom);
            break;
            // white sector top right
        case 6:
        case 7:
        case 12:
        case 13:
        case 18:
        case 19:
            FindKeyPoint(Center, Right, Top);
            FindKeyPoint(LeftRightHorn, Right, Bottom);
            FindKeyPoint(TopBottomHorn, Left, Top);
            FindOppositeLeftRight(Left);
            FindOppositeTopBottom(Bottom);
            break;
            // white sector bottom left
        case 8:
        case 10:
        case 14:
        case 16:
        case 20:
        case 22:
            FindKeyPoint(Center, Left, Bottom);
            FindKeyPoint(LeftRightHorn, Left, Top);
            FindKeyPoint(TopBottomHorn, Right, Bottom);
            FindOppositeLeftRight(Right);
            FindOppositeTopBottom(Top);
            break;
            // white sector bottom right
        case 9:
        case 11:
        case 15:
        case 17:
        case 21:
        case 23:
            FindKeyPoint(Center, Right, Bottom);
            FindKeyPoint(LeftRightHorn, Right, Top);
            FindKeyPoint(TopBottomHorn, Left, Bottom);
            FindOppositeLeftRight(Left);
            FindOppositeTopBottom(Top);
            break;
            
        default:
            break;
    }
   
    return 1;
}

// Computes probes for different permutations
int CMDetect::PermShift()
{
    // a = white
    // b = green
    // c = orange
    // d = black
    //
    // a  b
    // c  d
    
// case 0: // a b c d
    ps_a1[0] = -P_SHIFT; ps_a2[0] = -P_SHIFT;
    ps_b1[0] = +P_SHIFT; ps_b2[0] = -P_SHIFT;
    ps_c1[0] = -P_SHIFT; ps_c2[0] = +P_SHIFT;
    ps_d1[0] = +P_SHIFT; ps_d2[0] = +P_SHIFT;
    
// case 1: // a b d c
    ps_a1[1] = -P_SHIFT; ps_a2[1] = -P_SHIFT;
    ps_b1[1] = +P_SHIFT; ps_b2[1] = -P_SHIFT;
    ps_d1[1] = -P_SHIFT; ps_d2[1] = +P_SHIFT;
    ps_c1[1] = +P_SHIFT; ps_c2[1] = +P_SHIFT;
    
//case 2: // a c b d
    ps_a1[2] = -P_SHIFT; ps_a2[2] = -P_SHIFT;
    ps_c1[2] = +P_SHIFT; ps_c2[2] = -P_SHIFT;
    ps_b1[2] = -P_SHIFT; ps_b2[2] = +P_SHIFT;
    ps_d1[2] = +P_SHIFT; ps_d2[2] = +P_SHIFT;
    
//case 3: // a c d b
    ps_a1[3] = -P_SHIFT; ps_a2[3] = -P_SHIFT;
    ps_c1[3] = +P_SHIFT; ps_c2[3] = -P_SHIFT;
    ps_d1[3] = -P_SHIFT; ps_d2[3] = +P_SHIFT;
    ps_b1[3] = +P_SHIFT; ps_b2[3] = +P_SHIFT;
    
//case 4: // a d b c
    ps_a1[4] = -P_SHIFT; ps_a2[4] = -P_SHIFT;
    ps_d1[4] = +P_SHIFT; ps_d2[4] = -P_SHIFT;
    ps_b1[4] = -P_SHIFT; ps_b2[4] = +P_SHIFT;
    ps_c1[4] = +P_SHIFT; ps_c2[4] = +P_SHIFT;
    
//case 5: // a d c b
    ps_a1[5] = -P_SHIFT; ps_a2[5] = -P_SHIFT;
    ps_d1[5] = +P_SHIFT; ps_d2[5] = -P_SHIFT;
    ps_c1[5] = -P_SHIFT; ps_c2[5] = +P_SHIFT;
    ps_b1[5] = +P_SHIFT; ps_b2[5] = +P_SHIFT;
    
//case 6: // b a c d
    ps_b1[6] = -P_SHIFT; ps_b2[6] = -P_SHIFT;
    ps_a1[6] = +P_SHIFT; ps_a2[6] = -P_SHIFT;
    ps_c1[6] = -P_SHIFT; ps_c2[6] = +P_SHIFT;
    ps_d1[6] = +P_SHIFT; ps_d2[6] = +P_SHIFT;
    
//case 7: // b a d c
    ps_b1[7] = -P_SHIFT; ps_b2[7] = -P_SHIFT;
    ps_a1[7] = +P_SHIFT; ps_a2[7] = -P_SHIFT;
    ps_d1[7] = -P_SHIFT; ps_d2[7] = +P_SHIFT;
    ps_c1[7] = +P_SHIFT; ps_c2[7] = +P_SHIFT;
    
//case 8: // b c a d
    ps_b1[8] = -P_SHIFT; ps_b2[8] = -P_SHIFT;
    ps_c1[8] = +P_SHIFT; ps_c2[8] = -P_SHIFT;
    ps_a1[8] = -P_SHIFT; ps_a2[8] = +P_SHIFT;
    ps_d1[8] = +P_SHIFT; ps_d2[8] = +P_SHIFT;
    
//case 9: // b c d a
    ps_b1[9] = -P_SHIFT; ps_b2[9] = -P_SHIFT;
    ps_c1[9] = +P_SHIFT; ps_c2[9] = -P_SHIFT;
    ps_d1[9] = -P_SHIFT; ps_d2[9] = +P_SHIFT;
    ps_a1[9] = +P_SHIFT; ps_a2[9] = +P_SHIFT;
    
//case 10: // b d a c
    ps_b1[10] = -P_SHIFT; ps_b2[10] = -P_SHIFT;
    ps_d1[10] = +P_SHIFT; ps_d2[10] = -P_SHIFT;
    ps_a1[10] = -P_SHIFT; ps_a2[10] = +P_SHIFT;
    ps_c1[10] = +P_SHIFT; ps_c2[10] = +P_SHIFT;
    
//case 11: // b d c a
    ps_b1[11] = -P_SHIFT; ps_b2[11] = -P_SHIFT;
    ps_d1[11] = +P_SHIFT; ps_d2[11] = -P_SHIFT;
    ps_c1[11] = -P_SHIFT; ps_c2[11] = +P_SHIFT;
    ps_a1[11] = +P_SHIFT; ps_a2[11] = +P_SHIFT;
    
//case 12: // c a b d
    ps_c1[12] = -P_SHIFT; ps_c2[12] = -P_SHIFT;
    ps_a1[12] = +P_SHIFT; ps_a2[12] = -P_SHIFT;
    ps_b1[12] = -P_SHIFT; ps_b2[12] = +P_SHIFT;
    ps_d1[12] = +P_SHIFT; ps_d2[12] = +P_SHIFT;
    
//case 13: // c a d b
    ps_c1[13] = -P_SHIFT; ps_c2[13] = -P_SHIFT;
    ps_a1[13] = +P_SHIFT; ps_a2[13] = -P_SHIFT;
    ps_d1[13] = -P_SHIFT; ps_d2[13] = +P_SHIFT;
    ps_b1[13] = +P_SHIFT; ps_b2[13] = +P_SHIFT;
    
//case 14: // c b a d
    ps_c1[14] = -P_SHIFT; ps_c2[14] = -P_SHIFT;
    ps_b1[14] = +P_SHIFT; ps_b2[14] = -P_SHIFT;
    ps_a1[14] = -P_SHIFT; ps_a2[14] = +P_SHIFT;
    ps_d1[14] = +P_SHIFT; ps_d2[14] = +P_SHIFT;
    
//case 15: // c b d a
    ps_c1[15] = -P_SHIFT; ps_c2[15] = -P_SHIFT;
    ps_b1[15] = +P_SHIFT; ps_b2[15] = -P_SHIFT;
    ps_d1[15] = -P_SHIFT; ps_d2[15] = +P_SHIFT;
    ps_a1[15] = +P_SHIFT; ps_a2[15] = +P_SHIFT;
    
//case 16: // c d a b
    ps_c1[16] = -P_SHIFT; ps_c2[16] = -P_SHIFT;
    ps_d1[16] = +P_SHIFT; ps_d2[16] = -P_SHIFT;
    ps_a1[16] = -P_SHIFT; ps_a2[16] = +P_SHIFT;
    ps_b1[16] = +P_SHIFT; ps_b2[16] = +P_SHIFT;
    
//case 17: // c d b a
    ps_c1[17] = -P_SHIFT; ps_c2[17] = -P_SHIFT;
    ps_d1[17] = +P_SHIFT; ps_d2[17] = -P_SHIFT;
    ps_b1[17] = -P_SHIFT; ps_b2[17] = +P_SHIFT;
    ps_a1[17] = +P_SHIFT; ps_a2[17] = +P_SHIFT;
    
//case 18: // d a b c
    ps_d1[18] = -P_SHIFT; ps_d2[18] = -P_SHIFT;
    ps_a1[18] = +P_SHIFT; ps_a2[18] = -P_SHIFT;
    ps_b1[18] = -P_SHIFT; ps_b2[18] = +P_SHIFT;
    ps_c1[18] = +P_SHIFT; ps_c2[18] = +P_SHIFT;
    
//case 19: // d a c b
    ps_d1[19] = -P_SHIFT; ps_d2[19] = -P_SHIFT;
    ps_a1[19] = +P_SHIFT; ps_a2[19] = -P_SHIFT;
    ps_c1[19] = -P_SHIFT; ps_c2[19] = +P_SHIFT;
    ps_b1[19] = +P_SHIFT; ps_b2[19] = +P_SHIFT;
    
//case 20: // d b a c
    ps_d1[20] = -P_SHIFT; ps_d2[20] = -P_SHIFT;
    ps_b1[20] = +P_SHIFT; ps_b2[20] = -P_SHIFT;
    ps_a1[20] = -P_SHIFT; ps_a2[20] = +P_SHIFT;
    ps_c1[20] = +P_SHIFT; ps_c2[20] = +P_SHIFT;
    
//case 21: // d b c a
    ps_d1[21] = -P_SHIFT; ps_d2[21] = -P_SHIFT;
    ps_b1[21] = +P_SHIFT; ps_b2[21] = -P_SHIFT;
    ps_c1[21] = -P_SHIFT; ps_c2[21] = +P_SHIFT;
    ps_a1[21] = +P_SHIFT; ps_a2[21] = +P_SHIFT;
    
//case 22: // d c a b
    ps_d1[22] = -P_SHIFT; ps_d2[22] = -P_SHIFT;
    ps_c1[22] = +P_SHIFT; ps_c2[22] = -P_SHIFT;
    ps_a1[22] = -P_SHIFT; ps_a2[22] = +P_SHIFT;
    ps_b1[22] = +P_SHIFT; ps_b2[22] = +P_SHIFT;
    
//case 23: // d c b a
    ps_d1[23] = -P_SHIFT; ps_d2[23] = -P_SHIFT;
    ps_c1[23] = +P_SHIFT; ps_c2[23] = -P_SHIFT;
    ps_b1[23] = -P_SHIFT; ps_b2[23] = +P_SHIFT;
    ps_a1[23] = +P_SHIFT; ps_a2[23] = +P_SHIFT;
    
    return 1;
};

// Loads look-up classifier table
int CMDetect::LoadTable(){
    
    // note: '0' means 'detect'    
    
    for (int id = 0; id < 18; id++){
        for (int j=0; j<256; j++) {
            for (int i=0; i<256; i++) {
                float diff = (float)j - (classPar_m[id] * (float)i + 0.5);
                if (diff >  classPar_b1[id] || diff < classPar_b2[id] )
                    LTF[id* 256 * 256 + j* 256 +i] = 1;
                else
                    LTF[id* 256 * 256 + j* 256 +i] =  0;
            }
        }
        
        // saturation zone.
        
        int SAT_VALUE = 240;    // this should be imported from the xml file
        
        int maxX1 = min(255, (int) ((255. - classPar_b1[id])/classPar_m[id]));
        int maxY1 = min(255, (int) (classPar_m[id] * 255. +classPar_b1[id]));
        int maxX2 = min(255, (int) ((255. - classPar_b2[id])/classPar_m[id]));
        int maxY2 = min(255, (int) (classPar_m[id] * 255. +classPar_b2[id]));
        
        int maxXStart = min(maxX1, maxX2);
        
        // why was this multiplied by 2?
//        int maxXEnd = min(255,2 * max(maxX1, maxX2));
        int maxXEnd = min(255,max(maxX1, maxX2));

        int maxYStart = min(maxY1, maxY2);
        
        // why was this multiplied by 2?
//        int maxYEnd = min(255,2 * max(maxY1, maxY2));
        int maxYEnd = min(255,max(maxY1, maxY2));
        
        // anything below the smallest value found (divided by margin ratio) in each channel is set to '1' ('not detect')
        
        
        for (int i=0; i < classPar_t_min_1[id]; i++)
            for (int j = 0; j <= 255; j++)
                LTF[id* 256 * 256 + j* 256 +i] = 1;

        for (int j = 0; j < classPar_t_min_2[id]; j++)
            for (int i=0; i <= 255; i++)
                LTF[id* 256 * 256 + j* 256 +i] = 1;
        
        // anything above the largest value found (multiplied by margin ratio) in each channel is set to '1' ('not detect'). Unless the max is in thesaturation region. Then we assume that it can grow indefinitely, so we set to '0' ('detect') all values that could be obtained with that channel saturating.
        
        // Changed RM 11-11-12. Now if the max of the strip inon any direction (x or y) hits the saturation value, we simply set to '0' the whole saturation strip. Seems to work much better.
        
        if (classPar_t_max_1[id] < SAT_VALUE) {
            for (int i=classPar_t_max_1[id]+1; i<=255; i++)            
                    for (int j = 0; j <= 255; j++)
                        LTF[id* 256 * 256 + j* 256 +i] = 1;
        }
        else {
            for (int j=0; j<=255; j++) {
            //for (int j=maxYStart; j<=maxYEnd; j++) {
                for (int i=SAT_VALUE; i<=255; i++) {
                    LTF[id* 256 * 256 + j* 256 +i] = 0;
                }
            }
        }
    
       
        if (classPar_t_max_2[id] < SAT_VALUE) {
            for (int j = classPar_t_max_2[id]+1; j<=255; j++)
                for (int i=0; i <= 255; i++)
                    LTF[id* 256 * 256 + j* 256 +i] = 1;
            
        }
        else {

            for (int i=0; i<=255; i++) {
            //for (int i=maxXStart; i<=maxXEnd; i++) {
                for (int j=SAT_VALUE; j<=255; j++) {
                    LTF[id* 256 * 256 + j* 256 +i] = 0;
                }
            }
        }
    }
    return 1;
}
    

