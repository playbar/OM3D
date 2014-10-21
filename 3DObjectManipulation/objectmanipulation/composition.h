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

#ifndef illumapp_composition_h
#define illumapp_composition_h

#include "bilinearInterpolation.h"
#include "quadrature.h"
void composition(BVHAccel* bvhAccel, Vector* vertexNormals, int* Faces,
                 PbrtPoint* TextureVertices, int* TextureFaces, PbrtPoint* TextureVertices_D, int* TextureFaces_D, int* adjacencies,// geometry
                 float* I, float* Itexartist, float* Itexdiff, float *Btex, float blendfactor, float* Igroundtex, bool* Igroundmask,
                 float* Idiffuse, float* Idifference, BVHAccel* firstAccel, PbrtPoint* VerticesFirstTransformed,
                 float u0, float v0, float ifu, float ifv, float* groundAxis,// camera parameters
                 float* cTheta, float* sTheta, float* cPhi, float* sPhi, float SphereRadius, Vector* diffuseLight, Vector& ambientLight,// light sphere parameters
                 Vector& albedo, Vector& groundAlbedo, 
                 int width, int height, int nt, int nkernels, int ndivs, int nsamp, int shapeIdOffset, int dmapwidth, int dmapheight,
                 int filterhsize, int filtertype, float inv_sigma_sq, // filter parameters
                 string& path, string& direc, string& outputfilename, float* outputimage); // extra parameters

#endif
