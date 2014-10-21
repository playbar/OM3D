//
//  bilinearInterpolation.h
//  pbrt
//
//  Created by Natasha Kholgade on 11/1/12.
//
//

#ifndef pbrt_bilinearInterpolation_h
#define pbrt_bilinearInterpolation_h

#include "helpers.h"
#include "opencv2/opencv.hpp"

bool bilinearInterp(float x, float y, int sz, float* invals, IplImage* binaryImage, Vector* bilerpedcolor);
bool quickNearestInterp(float x, float y, int sz, float* invals, IplImage* binaryImage, Vector* bilerpedcolor);
bool bilinearInterpTexture(float x, float y, int height, int width, float* invals, float* Btex, Vector* bilerpedcolor, int id,
						   int* TF, PbrtPoint* TV, int* adjacencies);
bool bilinearInterpTexture(float x, float y, int height, int width, float* invals, Vector* bilerpedcolor, int id,
						   int* TF, PbrtPoint* TV);
bool bilinearInterpTexture(float x, float y, int height, int width, float* invals, float* Btex, float* bilerpedcolor, int id,
						   int* TF, PbrtPoint* TV, int* adjacencies);
bool bilinearInterpTexture(float x, float y, int height, int width, float* invals, float* bilerpedcolor, int id,
						   int* TF, PbrtPoint* TV);
bool textureCorrectIndices(float x, float y, int height, int width, float* invals, int id, int* TF, PbrtPoint *TV, int* adj, Vector* result) ;

#endif
