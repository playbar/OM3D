//
//  composition.h
//  illumapp
//
//  Created by Natasha Kholgade on 4/9/14.
//
//

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
