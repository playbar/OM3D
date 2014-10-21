//
//  quadrature.h
//  pbrt
//
//  Created by Natasha Kholgade on 11/1/12.
//
//

#ifndef __pbrt__quadrature__
#define __pbrt__quadrature__

#include "geometry.h"
#include "bvh.h"

Vector quadratureWall(PbrtPoint intersectionPt, Vector n, Ray r, BVHAccel* bvhAccel,
					  Vector* diffuseLight,
					  float* cTheta, float* sTheta, float* cPhi, float* sPhi, float SphereRadius,
					  int ndivs, float* groundAxis, Vector origcolor, Vector wallalbedo);
Vector quadrature(PbrtPoint intersectionPt, Vector n, Ray r, BVHAccel* bvhAccel,
				  Vector* diffuseLight, Vector* specularLight, bool hasSpecular,
				  float* cTheta, float* sTheta, float* cPhi, float* sPhi, float SphereRadius,
				  int ndivs, float* groundAxis, Vector &specularColor, float* t=NULL, double* K=NULL, bool* B=NULL);
Vector quadrature(PbrtPoint intersectionPt, Vector n, Ray r, BVHAccel* bvhAccel,
				  Vector* diffuseLight, float* cTheta, float* sTheta, float* cPhi, float* sPhi, float SphereRadius,
				  int ndivs, float* groundAxis, float* t=NULL, double* K=NULL, bool* B=NULL);

void quadrature(PbrtPoint intersectionPt, Vector n, Ray r, BVHAccel* bvhAccel,
				Vector** diffuseLight, Vector** specularLight, bool hasSpecular,
				float* cTheta, float* sTheta, float* cPhi, float* sPhi, float SphereRadius,
				int ndivs, float* groundAxis,
				float* shines, int nshines,
				Vector* diffuseColors, Vector* specularColors,
				PbrtPoint groundPt, Vector groundn, Vector* groundColorNoObj,
				float* t=NULL, double* K=NULL, bool* B=NULL) ;

void quadrature(PbrtPoint intersectionPt, Vector n, Ray r, BVHAccel* bvhAccel,
				Vector** diffuseLight,
				float* cTheta, float* sTheta, float* cPhi, float* sPhi, float SphereRadius,
				int ndivs, float* groundAxis,
				Vector* diffuseColors,
				PbrtPoint groundPt, Vector groundn, Vector* groundColorNoObj,
				float* t=NULL, double* K=NULL, bool* B=NULL) ;

#endif
