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

#ifndef __pbrt__quadrature__
#define __pbrt__quadrature__

#include "geometry.h"
#include "bvh.h"

Vector quadrature(PbrtPoint intersectionPt, Vector n, Ray r, BVHAccel* bvhAccel,
				  Vector* diffuseLight, Vector* specularLight, bool hasSpecular,
				  float* cTheta, float* sTheta, float* cPhi, float* sPhi, float SphereRadius,
				  int ndivs, float* groundAxis, Vector &specularColor, float* t=NULL, double* K=NULL, bool* B=NULL);
Vector quadrature(PbrtPoint intersectionPt, Vector n, Ray r, BVHAccel* bvhAccel,
				  Vector* diffuseLight, float* cTheta, float* sTheta, float* cPhi, float* sPhi, float SphereRadius,
				  int ndivs, float* groundAxis, bool isgnd, float* t=NULL, double* K=NULL, bool* B=NULL);

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
				int ndivs, float* groundAxis, bool isgnd,
				Vector* diffuseColors,
				PbrtPoint groundPt, Vector groundn, Vector* groundColorNoObj,
				float* t=NULL, double* K=NULL, bool* B=NULL) ;

#endif
