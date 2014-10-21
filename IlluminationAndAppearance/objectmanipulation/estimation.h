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

#ifndef __illumapp__illumapp__
#define __illumapp__illumapp__


#include <iostream>
#include "stdafx.h"
#include "api.h"
#include "probes.h"
#include "parser.h"
#include "parallel.h"
#include "geometry.h"
#include "transform.h"
#include "texture.h"
#include "trianglemesh.h"
#include "sphere.h"
#include "primitive.h"
#include "material.h"
#include "bvh.h"
#include "intersection.h"
#include "opencv2/opencv.hpp"
#include "time.h"
#include "math.h"
#ifdef OPTIMIZE_REFLECTANCE
#include "engine.h"
#endif
//#include "highgui.hpp"
#include "rng.h"
#include "mosek.h"
#include "ANNhelp.h"
#include "bilinearInterpolation.h"
#include "MRFEnergy.h"
#include "quadrature.h"

#include "definitions.h"

void iterativeIlluminationAppearanceEstimate(BVHAccel* bvhAccel, Vector* vertexNormals, int* Faces,
                                            float* I, bool* Igroundmask, bool* Ishadowmask, float* K, bool* B,
                                             float* PixelSpaceAppearance, IplImage* binaryImage, IplImage* erodedImage, IplImage* middilatedImage,
                                             float u0, float v0, float ifu, float ifv, float fu, float fv, float* groundAxis,// camera parameters
                                             Vector** diffuseLight, Vector& ambientLight, // light sphere parameters
                                             float* cTheta, float* sTheta, float* cPhi, float* sPhi, float SphereRadius,
                                             int* materialNumImage, float* adjustedmaterials, int* mtlnums, Vector& albedo, Vector& groundAlbedo, Vector& ambientGroundAlbedo,
                                             param& parameters,
                                             int width, int height,// image parameters
                                             int nkernels, int ndivs, int shapeIdOffset, int dmapwidth, int dmapheight, int nt,
                                             string& path, string& direc, string& outputfilename, int lightType,
                                             float** diffuseLightOutput, float** groundNoObjLightOutput);

void estimateAppearance(BVHAccel* bvhAccel, Vector* vertexNormals, int* Faces,
                        float* I, bool* Igroundmask, bool* Ishadowmask, float* K, bool* B,
                        float* PixelSpaceAppearance, IplImage* binaryImage, IplImage* erodedImage, IplImage* middilatedImage,
                        float u0, float v0, float ifu, float ifv, float fu, float fv, float* groundAxis,// camera parameters
                        Vector** diffuseLight, Vector& ambientLight, // light sphere parameters
                        float* cTheta, float* sTheta, float* cPhi, float* sPhi, float SphereRadius,
                        int* materialNumImage, float* adjustedmaterials, int* mtlnums, Vector& albedo, Vector& groundAlbedo, Vector& ambientGroundAlbedo,
                        param& parameters,
                        int width, int height, int ndivs, int shapeIdOffset, int dmapwidth, int dmapheight, int nt,
                        string& path, string& direc, string& outputfilename, int iternum, float lightLambdaError,
                        float** diffuseLightOutput, float** groundNoObjLightOutput);


void estimateIllumination(BVHAccel* bvhAccel, Vector* vertexNormals, int* Faces,
                          float* I,	bool* Igroundmask, bool* Ishadowmask, // image data structure
                          float* PixelSpaceAppearance, IplImage* binaryImage, IplImage* erodedImage,
                          float u0, float v0, float ifu, float ifv, float* groundAxis,// camera parameters
                          float SphereRadius, float* K, bool* B,// light sphere parameters
                          float* cTheta, float* sTheta, float* cPhi, float* sPhi, 
                          int* mtlnums, Vector& albedo, Vector& groundAlbedo, Vector &ambientGroundAlbedo,
                          param& parameters,
                          int width, int height, // image parameters
                          int nkernels, int ndivs, int shapeIdOffset, int dmapwidth, int dmapheight, // extra parameters
                          string path, string direc, int iternum, float& lightLambdaError,
                          Vector** diffuseLight, Vector& ambientLight, int lightType=TYPE_COLOR);



void completeAppearance(BVHAccel* bvhAccel, BVHAccel* texAccel, Vector* vertexNormals, Vector* vertexNormalsOrig,
                        PbrtPoint* Vertices, PbrtPoint* VerticesOriginal, int* Faces, PbrtPoint* TextureVertices, int* TextureFaces, // geometry
                        float* I, float* Itexartist, float* Itexdiff, float* Btex, float* Igroundtex,
                        bool* Igroundmask, bool* Ishadowmask,
                        float* PixelSpaceAppearance, IplImage* binaryImage, IplImage* erodedImage, IplImage* middilatedImage,
                        float* colordifference, float** diffuseLightOutput, float** groundNoObjLightOutput,
                        float u0, float v0, float ifu, float ifv, float fu, float fv, float* groundAxis,// camera parameters
                        Vector** diffuseLight, Vector& ambientLight, // light sphere parameters
                        float* cTheta, float* sTheta, float* cPhi, float* sPhi, float SphereRadius,
                        int* materialNumImage, float* adjustedmaterials, int* mtlnums, Vector& albedo, Vector& groundAlbedo, Vector& ambientGroundAlbedo,
                        param& parameters,
                        int width, int height,
                        int nkernels, int ndivs, int nsamp, int shapeIdOffset, int texIdOffset, int dmapwidth, int dmapheight, int nv, int nt,
                        string& path, string& direc, string& outputfilename,
                        float o2w[4][4]);

void extractPixelSpaceAppearanceAndMasks( BVHAccel* bvhAccel, PbrtPoint* TextureVertices, int* TextureFaces, int* adjacencies,// geometry
                                         int* mtlnums, float* Itexartist, float* Btex, // image data structure
                                         float u0, float v0, float ifu, float ifv,// camera parameters
                                         int width, int height, int shapeIdOffset, int dmapwidth, int dmapheight, // extra parameters
                                         float* PixelSpaceAppearance, IplImage* binaryImage, IplImage* erodedImage, IplImage* middilatedImage );

#endif /* defined(__illumapp__illumapp__) */
