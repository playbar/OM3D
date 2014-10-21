/*
 OM3D allows users to manipulate objects in photographs in 3D
 by using stock 3D models.
 Copyright (C) 2014 Natasha Kholgade and Tomas Simon
 
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 as published by the Free Software Foundation; either version 2
 of the License, or (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 
 For further details, please contact the authors at nkholgad@cs.cmu.edu
 */

#ifndef pbrt_bilinearInterpolation_h
#define pbrt_bilinearInterpolation_h

#include "helpers.h"
#include "opencv2/opencv.hpp"

void getTexVal(float* Itex, int* materialnums, int shapeId, Vector* outputcolor);
void getTexValImage(float* Itex, int* materialNumImage, int i, Vector* outputcolor);

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
