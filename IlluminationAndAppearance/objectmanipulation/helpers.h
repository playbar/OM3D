//
//  helpers.h
//  pbrt
//
//  Created by Natasha Kholgade on 11/1/12.
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

#ifndef pbrt_helpers_h
#define pbrt_helpers_h

#include "geometry.h"
#include "transform.h"
#include "texture.h"
#include "trianglemesh.h"
#include "primitive.h"
#include "intersection.h"
#include "definitions.h"

/* Readers and writers */
void writeFile(string direc, void* ptr, int size, const char* name);
void readFile(string direc, void* ptr, int size, const char* name);
FILE* openFile(string direc, const char* name, const char* type);

void readImage( string direc, const char* name, float* ptr);
void readBinaryImage( string direc, const char* name, bool* ptr);
void readBinaryImage( string direc, const char* name, float* ptr);
void readImageInfo( string direc, const char* name, int& width, int& height);
void writeImage( string direc, float* ptr, int width, int height, const char* name);


/* Comparator for the pair */
bool comparator(std::pair<int,int> i, std::pair<int,int> j);

/* Following were attempted for bilinear interpolation; didn't really work I think*/
void computeFaceAdjacency(int* faces, int* Tfaces, int* fadjacencies, int* tadjacencies, int nt);
void determineSeams(int* present_iround, bool* isseam, int iround, int nrounds, int ntexpixel, int height);


/* Compute Bounding Box */
float computeBoundingBox(PbrtPoint* Vertices, int nverts, PbrtPoint* bbox);

/* Estimate a radius for the light-sphere */
float estimateLightSphereRadius(PbrtPoint* Vertices, int nverts, float o2w[4][4]);

void texturescale(PbrtPoint vdtex, int dmapheight, int dmapwidth, float* xout, float* yout );


/* Used for calculating gaussian */
float getWeight(float* x, float* mu, float oneover2sigmasq=.5f);
float getWeight(float d, float oneover2sigmasq=.5f);
float getDistanceSq(float* x, float* mu);


bool checkIntersection(float x, float y, float x1, float y1, float x2, float y2, float x3, float y3, float* bcs);

#endif
