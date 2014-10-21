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

#include "bilinearInterpolation.h"


bool bilinearInterp(float x, float y, int sz, float* invals, IplImage* binaryImage, Vector* bilerpedcolor) {
	// search for the four pixels (floor and ceiling) that are closest to this pixel
	float x1,x2,y1,y2;
	int idx11, idx12, idx21, idx22;
	float x1x, y1y, x2x, y2y, x1xy1y, x1xy2y, x2xy1y, x2xy2y, denom;
	x1=floorf(x); x2=x1+1;
	y1=floorf(y); y2=y1+1;
	bool retval;
	
	// only use the values that are within the eroded binary mask
	
	if (binaryImage==NULL || (*cvGet2D(binaryImage,x1,y1).val > 0 &&
							  *cvGet2D(binaryImage,x1,y2).val > 0 &&
							  *cvGet2D(binaryImage,x2,y1).val > 0 &&
							  *cvGet2D(binaryImage,x2,y2).val > 0)) {
		idx11=(int)(x1*sz+y1);
		idx12=(int)(x1*sz+y2);
		idx21=(int)(x2*sz+y1);
		idx22=(int)(x2*sz+y2);
		
		// bilerp
		x1x=x-x1; y1y=y-y1; x2x=x2-x; y2y=y2-y;
		x1xy1y=x1x*y1y; x1xy2y=x1x*y2y; x2xy1y=x2x*y1y; x2xy2y=x2x*y2y;
		denom=1.f/(x2-x1)*(y2-y1);
		
		bilerpedcolor->x=(invals[3*idx11]*x2xy2y+invals[3*idx12]*x2xy1y+invals[3*idx21]*x1xy2y+invals[3*idx22]*x1xy1y)*denom;
		bilerpedcolor->y=(invals[3*idx11+1]*x2xy2y+invals[3*idx12+1]*x2xy1y+invals[3*idx21+1]*x1xy2y+invals[3*idx22+1]*x1xy1y)*denom;
		bilerpedcolor->z=(invals[3*idx11+2]*x2xy2y+invals[3*idx12+2]*x2xy1y+invals[3*idx21+2]*x1xy2y+invals[3*idx22+2]*x1xy1y)*denom;
		retval=true;
	} else {
		bilerpedcolor->x=0.f; bilerpedcolor->y=0.f; bilerpedcolor->z=0.f;
		retval=false;
	}
	return retval;
}


bool quickNearestInterp(float x, float y, int sz, float* invals, IplImage* binaryImage, Vector* bilerpedcolor) {
	// search for the four pixels (floor and ceiling) that are closest to this pixel
	float x1,x2,y1,y2;
	int idx11, idx12, idx21, idx22;
	//float x1x, y1y, x2x, y2y, x1xy1y, x1xy2y, x2xy1y, x2xy2y, denom;
	x1=floorf(x); x2=x1+1;
	y1=floorf(y); y2=y1+1;
	bool retval;
	
	// only use the values that are within the eroded binary mask
	
	if (binaryImage==NULL || (*cvGet2D(binaryImage,x1,y1).val > 0 &&
							  *cvGet2D(binaryImage,x1,y2).val > 0 &&
							  *cvGet2D(binaryImage,x2,y1).val > 0 &&
							  *cvGet2D(binaryImage,x2,y2).val > 0)) {
		idx11=(int)(x1*sz+y1);
		idx12=(int)(x1*sz+y2);
		idx21=(int)(x2*sz+y1);
		idx22=(int)(x2*sz+y2);
		
		float d11=(x-x1)*(x-x1)+(y-y1)*(y-y1);
		float d12=(x-x1)*(x-x1)+(y-y2)*(y-y2);
		float d21=(x-x2)*(x-x2)+(y-y1)*(y-y1);
		float d22=(x-x2)*(x-x2)+(y-y2)*(y-y2);
        
		std::pair<int, int> p(0,0);
		vector<std::pair<int,int> > dists(4,p);
		dists[0].first=d11; dists[0].second=idx11;
		dists[1].first=d12; dists[1].second=idx12;
		dists[2].first=d21; dists[2].second=idx21;
		dists[3].first=d22; dists[3].second=idx22;
		
		sort(dists.begin(), dists.end(), comparator);
		
		float d;
		
		bool canbreak=false;
		
		int j=0;
		for (j=0; j<4 && !canbreak; j++) {
			d=invals[3*dists[j].second]*invals[3*dists[j].second]+invals[3*dists[j].second+1]*invals[3*dists[j].second+1]+
			invals[3*dists[j].second+2]*invals[3*dists[j].second+2];
			canbreak=d>1e-3;
		}
		
		if (canbreak) {
			bilerpedcolor->x=invals[3*dists[j-1].second];
			bilerpedcolor->y=invals[3*dists[j-1].second+1];
			bilerpedcolor->z=invals[3*dists[j-1].second+2];
		}
		
		retval=true;
	} else {
		bilerpedcolor->x=0.f; bilerpedcolor->y=0.f; bilerpedcolor->z=0.f;
		retval=false;
	}
	return retval;
}

bool bilinearInterpTexture(float x, float y, int height, int width, float* invals, float* Btex, float* bilerpedcolor, int id,
						   int* TF, PbrtPoint* TV, int* adjacencies) {
	// search for the four pixels (floor and ceiling) that are closest to this pixel
	float x1,x2,y1,y2;
	int idx11, idx12, idx21, idx22;
	float x1x, y1y, x2x, y2y, x1xy1y, x1xy2y, x2xy1y, x2xy2y, denom;
	x1=floorf(x); x2=x1+1;
	y1=floorf(y); y2=y1+1;
	bool retval;
	
	// only use the values that are within the eroded binary mask
	
	// for texture, find the correctly adjacent faces before interpolating
	
	
	idx11=(int)(x1*height+y1);
	idx12=(int)(x1*height+y2);
	idx21=(int)(x2*height+y1);
	idx22=(int)(x2*height+y2);
	
	// bilerp
	x1x=x-x1; y1y=y-y1; x2x=x2-x; y2y=y2-y;
	x1xy1y=x1x*y1y; x1xy2y=x1x*y2y; x2xy1y=x2x*y1y; x2xy2y=x2x*y2y;
	denom=1.f/(x2-x1)*(y2-y1);
	
	float r11=invals[3*idx11], r12=invals[3*idx12], r21=invals[3*idx21], r22=invals[3*idx22];
	float g11=invals[3*idx11+1], g12=invals[3*idx12+1], g21=invals[3*idx21+1], g22=invals[3*idx22+1];
	float bb11=invals[3*idx11+2], bb12=invals[3*idx12+2], bb21=invals[3*idx21+2], bb22=invals[3*idx22+2];
	
	Vector v11, v12, v21, v22;
	v11.x=r11; v11.y=g11; v11.z=bb11;
	v12.x=r12; v12.y=g12; v12.z=bb12;
	v21.x=r21; v21.y=g21; v21.z=bb21;
	v22.x=r22; v22.y=g22; v22.z=bb22;
	
	float d11=(x-x1)*(x-x1)+(y-y1)*(y-y1);
	float d12=(x-x1)*(x-x1)+(y-y2)*(y-y2);
	float d21=(x-x2)*(x-x2)+(y-y1)*(y-y1);
	float d22=(x-x2)*(x-x2)+(y-y2)*(y-y2);
	
	
	//if (!textureCorrectIndices(x1,y1,height,width,invals,faceIndex,TextureFaces,TextureVertices,adjacencies,&v11))
	//float xb=x*1.f/width, yb=y*1.f/height;
	
	
	bool b11, b12, b21, b22;
	//b11=checkIntersection(x1*1.f/width, y1*1.f/height, TV[TF[3*id]].x, TV[TF[3*id]].y, TV[TF[3*id+1]].x, TV[TF[3*id+1]].y, TV[TF[3*id+2]].x, TV[TF[3*id+2]].y, NULL);
	//b12=checkIntersection(x1*1.f/width, y2*1.f/height, TV[TF[3*id]].x, TV[TF[3*id]].y, TV[TF[3*id+1]].x, TV[TF[3*id+1]].y, TV[TF[3*id+2]].x, TV[TF[3*id+2]].y, NULL);
	//b21=checkIntersection(x2*1.f/width, y1*1.f/height, TV[TF[3*id]].x, TV[TF[3*id]].y, TV[TF[3*id+1]].x, TV[TF[3*id+1]].y, TV[TF[3*id+2]].x, TV[TF[3*id+2]].y, NULL);
	//b22=checkIntersection(x2*1.f/width, y2*1.f/height, TV[TF[3*id]].x, TV[TF[3*id]].y, TV[TF[3*id+1]].x, TV[TF[3*id+1]].y, TV[TF[3*id+2]].x, TV[TF[3*id+2]].y, NULL);
	b11=!Btex || Btex[idx11]<.5f;
	b12=!Btex || Btex[idx12]<.5f;
	b21=!Btex || Btex[idx21]<.5f;
	b22=!Btex || Btex[idx22]<.5f;
	d11=b11 ? d11 : INFINITY;
	d12=b12 ? d12 : INFINITY;
	d21=b21 ? d21 : INFINITY;
	d22=b22 ? d22 : INFINITY;
	
	
	//{
	//	 }
	//if (!textureCorrectIndices(x1,y2,height,width,invals,faceIndex,TextureFaces,TextureVertices,adjacencies,&v12))
	//{
	//	 }
	//if (!textureCorrectIndices(x2,y1,height,width,invals,faceIndex,TextureFaces,TextureVertices,adjacencies,&v21))
	//{
	//	 }
	//if (!textureCorrectIndices(x2,y2,height,width,invals,faceIndex,TextureFaces,TextureVertices,adjacencies,&v22))
	//{
	//	 }
	
	if (b11 & b12 & b21 & b22) {
		bilerpedcolor[0]=(v11.x*x2xy2y+v12.x*x2xy1y+v21.x*x1xy2y+v22.x*x1xy1y)*denom;
		bilerpedcolor[1]=(v11.y*x2xy2y+v12.y*x2xy1y+v21.y*x1xy2y+v22.y*x1xy1y)*denom;
		bilerpedcolor[2]=(v11.z*x2xy2y+v12.z*x2xy1y+v21.z*x1xy2y+v22.z*x1xy1y)*denom;
	} else {
		float mind=INFINITY; int idx=0;
		if (d11<mind) { mind=d11; idx=0; }
		if (d12<mind) { mind=d12; idx=1; }
		if (d21<mind) { mind=d21; idx=2; }
		if (d22<mind) { idx=3; }
		if (idx==0) {bilerpedcolor[0]=v11.x;bilerpedcolor[1]=v11.y;bilerpedcolor[2]=v11.z;}
		if (idx==1) {bilerpedcolor[0]=v12.x;bilerpedcolor[1]=v12.y;bilerpedcolor[2]=v12.z;}
		if (idx==2) {bilerpedcolor[0]=v21.x;bilerpedcolor[1]=v21.y;bilerpedcolor[2]=v21.z;}
		if (idx==3) {bilerpedcolor[0]=v22.x;bilerpedcolor[1]=v22.y;bilerpedcolor[2]=v22.z;}
	}
	
	retval=true;
	return retval;
}

bool bilinearInterpTexture(float x, float y, int height, int width, float* invals, float* bilerpedcolor, int id,
						   int* TF, PbrtPoint* TV) {
	// search for the four pixels (floor and ceiling) that are closest to this pixel
	float x1,x2,y1,y2;
	int idx11, idx12, idx21, idx22;
	float x1x, y1y, x2x, y2y, x1xy1y, x1xy2y, x2xy1y, x2xy2y, denom;
	x1=floorf(x); x2=x1+1;
	y1=floorf(y); y2=y1+1;
	bool retval;
	
	// only use the values that are within the eroded binary mask
	
	// for texture, find the correctly adjacent faces before interpolating
	
	
	idx11=(int)(x1*height+y1);
	idx12=(int)(x1*height+y2);
	idx21=(int)(x2*height+y1);
	idx22=(int)(x2*height+y2);
	
	// bilerp
	x1x=x-x1; y1y=y-y1; x2x=x2-x; y2y=y2-y;
	x1xy1y=x1x*y1y; x1xy2y=x1x*y2y; x2xy1y=x2x*y1y; x2xy2y=x2x*y2y;
	denom=1.f/(x2-x1)*(y2-y1);
	
	float r11=invals[3*idx11], r12=invals[3*idx12], r21=invals[3*idx21], r22=invals[3*idx22];
	float g11=invals[3*idx11+1], g12=invals[3*idx12+1], g21=invals[3*idx21+1], g22=invals[3*idx22+1];
	float bb11=invals[3*idx11+2], bb12=invals[3*idx12+2], bb21=invals[3*idx21+2], bb22=invals[3*idx22+2];
	
	Vector v11, v12, v21, v22;
	v11.x=r11; v11.y=g11; v11.z=bb11;
	v12.x=r12; v12.y=g12; v12.z=bb12;
	v21.x=r21; v21.y=g21; v21.z=bb21;
	v22.x=r22; v22.y=g22; v22.z=bb22;
	
	float d11=(x-x1)*(x-x1)+(y-y1)*(y-y1);
	float d12=(x-x1)*(x-x1)+(y-y2)*(y-y2);
	float d21=(x-x2)*(x-x2)+(y-y1)*(y-y1);
	float d22=(x-x2)*(x-x2)+(y-y2)*(y-y2);
	
	
    float mind=INFINITY; int idx=0;
		if (d11<mind) { mind=d11; idx=0; }
		if (d12<mind) { mind=d12; idx=1; }
		if (d21<mind) { mind=d21; idx=2; }
		if (d22<mind) { idx=3; }
		if (idx==0) {bilerpedcolor[0]=v11.x;bilerpedcolor[1]=v11.y;bilerpedcolor[2]=v11.z;}
		if (idx==1) {bilerpedcolor[0]=v12.x;bilerpedcolor[1]=v12.y;bilerpedcolor[2]=v12.z;}
		if (idx==2) {bilerpedcolor[0]=v21.x;bilerpedcolor[1]=v21.y;bilerpedcolor[2]=v21.z;}
		if (idx==3) {bilerpedcolor[0]=v22.x;bilerpedcolor[1]=v22.y;bilerpedcolor[2]=v22.z;}
	
	retval=true;
	return retval;
}


bool bilinearInterpTexture(float x, float y, int height, int width, float* invals, float* Btex, Vector* bilerpedcolor, int id,
						   int* TF, PbrtPoint* TV, int* adjacencies) {
	// search for the four pixels (floor and ceiling) that are closest to this pixel
	float x1,x2,y1,y2;
	int idx11, idx12, idx21, idx22;
	float x1x, y1y, x2x, y2y, x1xy1y, x1xy2y, x2xy1y, x2xy2y, denom;
	x1=floorf(x); x2=x1+1;
	y1=floorf(y); y2=y1+1;
	bool retval;
	
	// only use the values that are within the eroded binary mask
	
	// for texture, find the correctly adjacent faces before interpolating
	
	
	idx11=(int)(x1*height+y1);
	idx12=(int)(x1*height+y2);
	idx21=(int)(x2*height+y1);
	idx22=(int)(x2*height+y2);
	
	// bilerp
	x1x=x-x1; y1y=y-y1; x2x=x2-x; y2y=y2-y;
	x1xy1y=x1x*y1y; x1xy2y=x1x*y2y; x2xy1y=x2x*y1y; x2xy2y=x2x*y2y;
	denom=1.f/(x2-x1)*(y2-y1);
	
	float r11=invals[3*idx11], r12=invals[3*idx12], r21=invals[3*idx21], r22=invals[3*idx22];
	float g11=invals[3*idx11+1], g12=invals[3*idx12+1], g21=invals[3*idx21+1], g22=invals[3*idx22+1];
	float bb11=invals[3*idx11+2], bb12=invals[3*idx12+2], bb21=invals[3*idx21+2], bb22=invals[3*idx22+2];
	
	Vector v11, v12, v21, v22;
	v11.x=r11; v11.y=g11; v11.z=bb11;
	v12.x=r12; v12.y=g12; v12.z=bb12;
	v21.x=r21; v21.y=g21; v21.z=bb21;
	v22.x=r22; v22.y=g22; v22.z=bb22;
	
	float d11=(x-x1)*(x-x1)+(y-y1)*(y-y1);
	float d12=(x-x1)*(x-x1)+(y-y2)*(y-y2);
	float d21=(x-x2)*(x-x2)+(y-y1)*(y-y1);
	float d22=(x-x2)*(x-x2)+(y-y2)*(y-y2);
	
	
	//if (!textureCorrectIndices(x1,y1,height,width,invals,faceIndex,TextureFaces,TextureVertices,adjacencies,&v11))
	//float xb=x*1.f/width, yb=y*1.f/height;
	
	
	bool b11, b12, b21, b22;
	//b11=checkIntersection(x1*1.f/width, y1*1.f/height, TV[TF[3*id]].x, TV[TF[3*id]].y, TV[TF[3*id+1]].x, TV[TF[3*id+1]].y, TV[TF[3*id+2]].x, TV[TF[3*id+2]].y, NULL);
	//b12=checkIntersection(x1*1.f/width, y2*1.f/height, TV[TF[3*id]].x, TV[TF[3*id]].y, TV[TF[3*id+1]].x, TV[TF[3*id+1]].y, TV[TF[3*id+2]].x, TV[TF[3*id+2]].y, NULL);
	//b21=checkIntersection(x2*1.f/width, y1*1.f/height, TV[TF[3*id]].x, TV[TF[3*id]].y, TV[TF[3*id+1]].x, TV[TF[3*id+1]].y, TV[TF[3*id+2]].x, TV[TF[3*id+2]].y, NULL);
	//b22=checkIntersection(x2*1.f/width, y2*1.f/height, TV[TF[3*id]].x, TV[TF[3*id]].y, TV[TF[3*id+1]].x, TV[TF[3*id+1]].y, TV[TF[3*id+2]].x, TV[TF[3*id+2]].y, NULL);
	b11=!Btex || Btex[idx11]<.5f;
	b12=!Btex || Btex[idx12]<.5f;
	b21=!Btex || Btex[idx21]<.5f;
	b22=!Btex || Btex[idx22]<.5f;
	d11=b11 ? d11 : INFINITY;
	d12=b12 ? d12 : INFINITY;
	d21=b21 ? d21 : INFINITY;
	d22=b22 ? d22 : INFINITY;
	
	
	//{
	//	 }
	//if (!textureCorrectIndices(x1,y2,height,width,invals,faceIndex,TextureFaces,TextureVertices,adjacencies,&v12))
	//{
	//	 }
	//if (!textureCorrectIndices(x2,y1,height,width,invals,faceIndex,TextureFaces,TextureVertices,adjacencies,&v21))
	//{
	//	 }
	//if (!textureCorrectIndices(x2,y2,height,width,invals,faceIndex,TextureFaces,TextureVertices,adjacencies,&v22))
	//{
	//	 }
	
	if (b11 & b12 & b21 & b22) {
		bilerpedcolor->x=(v11.x*x2xy2y+v12.x*x2xy1y+v21.x*x1xy2y+v22.x*x1xy1y)*denom;
		bilerpedcolor->y=(v11.y*x2xy2y+v12.y*x2xy1y+v21.y*x1xy2y+v22.y*x1xy1y)*denom;
		bilerpedcolor->z=(v11.z*x2xy2y+v12.z*x2xy1y+v21.z*x1xy2y+v22.z*x1xy1y)*denom;
	} else {
		float mind=INFINITY; int idx=0;
		if (d11<mind) { mind=d11; idx=0; }
		if (d12<mind) { mind=d12; idx=1; }
		if (d21<mind) { mind=d21; idx=2; }
		if (d22<mind) { idx=3; }
		if (idx==0) *bilerpedcolor=v11;
		if (idx==1) *bilerpedcolor=v12;
		if (idx==2) *bilerpedcolor=v21;
		if (idx==3) *bilerpedcolor=v22;
	}
	
	retval=true;
	return retval;
}
bool bilinearInterpTexture(float x, float y, int height, int width, float* invals, Vector* bilerpedcolor, int id,
						   int* TF, PbrtPoint* TV) {
	// search for the four pixels (floor and ceiling) that are closest to this pixel
	float x1,x2,y1,y2;
	int idx11, idx12, idx21, idx22;
	float x1x, y1y, x2x, y2y, x1xy1y, x1xy2y, x2xy1y, x2xy2y, denom;
	x1=floorf(x); x2=x1+1;
	y1=floorf(y); y2=y1+1;
	bool retval;
	
	// only use the values that are within the eroded binary mask
	
	// for texture, find the correctly adjacent faces before interpolating
	
	
	idx11=(int)(x1*height+y1);
	idx12=(int)(x1*height+y2);
	idx21=(int)(x2*height+y1);
	idx22=(int)(x2*height+y2);
	
	// bilerp
	x1x=x-x1; y1y=y-y1; x2x=x2-x; y2y=y2-y;
	x1xy1y=x1x*y1y; x1xy2y=x1x*y2y; x2xy1y=x2x*y1y; x2xy2y=x2x*y2y;
	denom=1.f/(x2-x1)*(y2-y1);
	
	float r11=invals[3*idx11], r12=invals[3*idx12], r21=invals[3*idx21], r22=invals[3*idx22];
	float g11=invals[3*idx11+1], g12=invals[3*idx12+1], g21=invals[3*idx21+1], g22=invals[3*idx22+1];
	float bb11=invals[3*idx11+2], bb12=invals[3*idx12+2], bb21=invals[3*idx21+2], bb22=invals[3*idx22+2];
	
	Vector v11, v12, v21, v22;
	v11.x=r11; v11.y=g11; v11.z=bb11;
	v12.x=r12; v12.y=g12; v12.z=bb12;
	v21.x=r21; v21.y=g21; v21.z=bb21;
	v22.x=r22; v22.y=g22; v22.z=bb22;
	
	float d11=(x-x1)*(x-x1)+(y-y1)*(y-y1);
	float d12=(x-x1)*(x-x1)+(y-y2)*(y-y2);
	float d21=(x-x2)*(x-x2)+(y-y1)*(y-y1);
	float d22=(x-x2)*(x-x2)+(y-y2)*(y-y2);
	
	
		float mind=INFINITY; int idx=0;
		if (d11<mind) { mind=d11; idx=0; }
		if (d12<mind) { mind=d12; idx=1; }
		if (d21<mind) { mind=d21; idx=2; }
		if (d22<mind) { idx=3; }
		if (idx==0) *bilerpedcolor=v11;
		if (idx==1) *bilerpedcolor=v12;
		if (idx==2) *bilerpedcolor=v21;
		if (idx==3) *bilerpedcolor=v22;
	
	retval=true;
	return retval;
}
bool textureCorrectIndices(float x, float y, int height, int width, float* invals, int id, int* TF, PbrtPoint *TV, int* adj, Vector* result) {
	// check if it intersects yourself, cuz if it does, just return false
	x=x*1.f/width; y=y*1.f/height;
	if (checkIntersection(x, y, TV[TF[3*id]].x, TV[TF[3*id]].y, TV[TF[3*id+1]].x, TV[TF[3*id+1]].y, TV[TF[3*id+2]].x, TV[TF[3*id+2]].y, NULL)) {
		return false;
	}
	float bcs[3];
	
	int j;
	
	for (j=0; j<3; j++) {
		// odd one out is 3*j+j
		int i0, i1, i0t, i1t;
		float x0, y0, x1, y1;
		float x0t, y0t, x1t, y1t;
		if (j==0) { i0=adj[9*id+3*j+1]; i1=adj[9*id+3*j+2]; i0t=TF[3*id+1]; i1t=TF[3*id+2]; }
		else if (j==1) {i0=adj[9*id+3*j+2]; i1=adj[9*id+3*j]; i0t=TF[3*id+2]; i1t=TF[3*id]; }
		else if (j==2) {i0=adj[9*id+3*j]; i1=adj[9*id+3*j+1]; i0t=TF[3*id]; i1t=TF[3*id+1]; }
		x0=TV[i0].x; x1=TV[i1].x; y0=TV[i0].y; y1=TV[i1].y;
		x0t=TV[i0t].x; x1t=TV[i1t].x; y0t=TV[i0t].y; y1t=TV[i1t].y;
		
		float denominv=1./(x0*x0 - 2*x0*x1 + x1*x1 + y0*y0 - 2*y0*y1 + y1*y1);
		float a=denominv*((x0 - x1)*x0t+(y0 - y1)*y0t+(x1 - x0)*x1t+(y1 - y0)*y1t);
		float b=denominv*((y0 - y1)*x0t+(x1 - x0)*y0t+(y1 - y0)*x1t+(x0 - x1)*y1t);
		float c=denominv*((x1*x1 - x0*x1 + y1*y1 - y0*y1)*x0t+(x0*y1 - x1*y0)*y0t+(x0*x0 - x1*x0 + y0*y0 - y1*y0)*x1t+(x1*y0 - x0*y1)*y1t);
		float d=denominv*((x1*y0 - x0*y1)*x0t+(x1*x1 - x0*x1 + y1*y1 - y0*y1)*y0t+(x0*y1 - x1*y0)*x1t+(x0*x0 - x1*x0 + y0*y0 - y1*y0)*y1t);
		
		float x_=TV[adj[9*id+3*j+j]].x, y_=TV[adj[9*id+3*j+j]].y;
		float x_t=a*x_+b*y_+c;
		float y_t=-b*x_+a*y_+d;
		
		if (checkIntersection(x, y, x0t, y0t, x1t, y1t, x_t, y_t, bcs)) {
			float xactual=(x0t*bcs[0]+x1t*bcs[1]+x_t*bcs[2])*width;
			float yactual=(y0t*bcs[0]+y1t*bcs[1]+y_t*bcs[2])*height;
			
			bilinearInterp(xactual, yactual, height, invals, NULL, result);
			return true;
		}
	}
	
	// if you get here, you're either not in your assigned face, or adjacent faces. Most likely you're very close to a vertex, and technically, you'd need to
	// look through faces in a fan around your adjacent vertex, but for now we'll just interpolate as usual
	return false;
}
