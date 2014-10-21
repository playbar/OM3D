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

#include "composition.h"

void composition(BVHAccel* bvhAccel, Vector* vertexNormals, int* Faces,
                 PbrtPoint* TextureVertices, int* TextureFaces, PbrtPoint* TextureVertices_D, int* TextureFaces_D, int* adjacencies,// geometry
                 float* I, float* Itexartist, float* Itexdiff, float *Btex, float blendfactor, float* Igroundtex, bool* Igroundmask,
                 float* Idiffuse, float* Idifference, BVHAccel* firstAccel, PbrtPoint* VerticesFirstTransformed,
                 float u0, float v0, float ifu, float ifv, float* groundAxis,// camera parameters
                 float* cTheta, float* sTheta, float* cPhi, float* sPhi, float SphereRadius, Vector* diffuseLight, Vector& ambientLight,// light sphere parameters
                 int* mtlnums, Vector& albedo, Vector& groundAlbedo, Vector& ambientGroundAlbedo,
                 int width, int height, int nt, int nkernels, int ndivs, int nsamp, int shapeIdOffset, int dmapwidth, int dmapheight,
                 int filterhsize, int filtertype, float inv_sigma_sq, // filter parameters
                 string& path, string& direc, string& outputfilename) { // extra parameters
	
	// intelligent sampling: maybe first sample every 10 pixels or so, and do computations for those pixels
	// then sample remaining pixels, and use them only if they are non-ground or if they are ground and if
	// they are close to shadowed regions of the image.
	
	// Declarations
	int i, ttshapeId, shapeId;
	bool didIntersect, didfirstIntersect;//, didnbdoIntersect, didnbuseIntersect;
	float xpix, ypix, xpixim, ypixim, xdelta, ydelta, lambda;
	float inv_max_rand=1.f/RAND_MAX;
	float barycentrics[3];
	time_t start, end;
    int npixels=width*height;
	//bool useAntiAlias=false;
	//int n_u=(int)sqrtf(ndivs);
	float* Igroundobjmask=new float[npixels];
    float* Iobjmask=new float[npixels];
    Intersection* firstisect=new Intersection();
    float firstbcs[3];
	
	path=direc;
	
	for (int i=0; i<npixels; i++) {
        Igroundobjmask[i]=Igroundmask[i] ? 1.0f : .0f;
    }
//	memcpy(Igroundobjmask, Igroundmask, npixels*sizeof(float));
	
	float x, y;
	
	Intersection* isect=new Intersection();
	Intersection* lisect=new Intersection();
	Intersection** isects=new Intersection*[4];
	for (i=0; i<4; i++) {
		isects[i]=new Intersection();
	}
	float bcs[3];
		
	Vector direction(.0,.0,1.0), lightDirection, n, reflection;
	PbrtPoint origin(.0,.0,.0);
	PbrtPoint intersectionPt, vdtex, vdtex_d, firstIntersectionPt;
	Ray r(origin,direction,0,INFINITY);
    Ray r2(origin,direction,0,INFINITY);
	Ray s(intersectionPt,lightDirection,0,INFINITY);
	   
	float* colorsamples=new float[3*nsamp*npixels];
	float* finaloutputsamples=new float[3*nsamp*npixels];
	float* lightoutputsamples=new float[3*nsamp*npixels];
	time(&start);
	
	//float stepdox, stepdoy, stepnum=3;;
	//bool didneighborintersect, groupsAreDifferent;
	int idxcol;
	//int nbh=0;
	
	for (i=0; i<3*nsamp*npixels; i++) colorsamples[i]=.0f;
	for (i=0; i<3*nsamp*npixels; i++) lightoutputsamples[i]=.0f;
	for (i=0; i<3*nsamp*npixels; i++) finaloutputsamples[i]=.0f;
	
	for (i=0; i<npixels; i++) {
		
		ypixim=(i % height);
		xpixim=(i / height);
		
		// do a 'pure' (non-random) test here. Based on the intersection flag of that test, test pixels in the
		// surrounding areas. Test points 2 pixels away and half-size pixels away. If there are points 2 pixels
		// away with the opposite intersection flag, mark this pixel as to 'doAntiAlias'.
		
		r.d.x=(xpixim-u0)*ifu; r.d.y=(ypixim-v0)*ifv; r.maxt=INFINITY; r.mint=0;
		didIntersect=bvhAccel->IntersectQ(r,isect,bcs);
		//didIntersect=bvhAccel->IntersectP(r);
		
        
        Igroundobjmask[i]=didIntersect ? 1.0f : Igroundmask[i];
        Iobjmask[i]=didIntersect;
		
        // CHANGED FOR PEN
        //Igroundobjmask[i]=1.0f;
		
        
	}
    
	path=direc;
    FILE* fid1=fopen(path.append("Igroundobjmask.dat").c_str(), "w"); fwrite(Igroundobjmask, npixels*sizeof(float), 1, fid1); fclose(fid1);
	
    for (i=0; i<4; i++) delete isects[i]; delete[] isects;
    
	
	path=direc;
	
	time(&end);
	double dif;
	dif=difftime(end,start);
	printf("Time taken for figuring which should be anti-aliased=%f seconds.\n",dif);
	
	// if the run number is 0, do everything at a lower sampling rate.
	// at run 1, check the pixels around you that have been sampled; if they are all the same color and ground,
	// there's a high chance you're ground as well, and just copy their color over; that way you can avoid
	// recomputing a whole bunch of things.
	int npixels_covered=0;
	int i3, idxcol1, idxcol2;
	size_t sf3=3*sizeof(float);
	
	time_t startin, endin;
	double difingoforth=0, difinaboveground=0, difinsphere=0, difincolor=0;
	
    time(&start);
    
    
    
    // Lightoutput is the amount of light that comes out (devoid of reflectance)
    // coloredoutput is with the reflectance multipled
    // finaloutput is with the difference tacked on
    Vector diffuselightoutput, coloredoutput, finaloutput;
    
    Vector diffusereflectance;
    Vector appearancedifference;
    
    //i=3084538;
    for (i=0; i<npixels; i++)
    //i=3995931;
    {
        {
            i3=3*i;
            // shoot a ray from the camera center through the pixel (i.e. [x2d,y2d,1])
            ypixim=(i % height);
            xpixim=(i / height);
            
            
            idxcol=3*i; idxcol1=idxcol+1; idxcol2=idxcol+2;
            
            if (abs(Igroundobjmask[i])<1e-3) { //
                memcpy(&colorsamples[idxcol], &Igroundtex[i3], sf3);
                memcpy(&finaloutputsamples[idxcol], &Igroundtex[i3], sf3);
                memcpy(&lightoutputsamples[idxcol], &Igroundtex[i3], sf3);
            } else {
                xpix=xpixim; ypix=ypixim;
                
                r.d.x=(xpix-u0)*ifu;
                r.d.y=(ypix-v0)*ifv;
                r.maxt=INFINITY;
                r.mint=0;
                
                // following tells you if you're ground or not
                didIntersect=bvhAccel->IntersectQ(r,isect,barycentrics);
                
                    time(&startin);
                
                
                npixels_covered++;
                time(&startin);
                //aboveground=true;
                if (!didIntersect) {
                    // if the ray does not intersect with the object,
                    // it might belong the ground
                    lambda=-groundAxis[3]*1.0f/(groundAxis[0]*r.d.x+groundAxis[1]*r.d.y+groundAxis[2]);
                    intersectionPt.x=lambda*r.d.x; intersectionPt.y=lambda*r.d.y; intersectionPt.z=lambda;
                    n=Vector(groundAxis[0],groundAxis[1],groundAxis[2]);
                    
                } else {
                    intersectionPt=r.o+r.d*r.maxt;
                    shapeId=isect->shapeId-shapeIdOffset;
                    ttshapeId=shapeId % nt;
                    
                    n=Normalize(barycentrics[0]*vertexNormals[Faces[3*shapeId]]+
                                barycentrics[1]*vertexNormals[Faces[3*shapeId+1]]+
                                barycentrics[2]*vertexNormals[Faces[3*shapeId+2]]);
                    
                    
                    vdtex=barycentrics[0]*TextureVertices[TextureFaces[3*ttshapeId]]+
                    barycentrics[1]*TextureVertices[TextureFaces[3*ttshapeId+1]]+
                    barycentrics[2]*TextureVertices[TextureFaces[3*ttshapeId+2]];
                    vdtex_d=barycentrics[0]*TextureVertices_D[TextureFaces_D[3*ttshapeId]]+
                    barycentrics[1]*TextureVertices_D[TextureFaces_D[3*ttshapeId+1]]+
                    barycentrics[2]*TextureVertices_D[TextureFaces_D[3*ttshapeId+2]];
                    
                    
                }
                
                time(&endin);
                difinaboveground+=difftime(endin, startin)*1.0f/npixels;
                time(&startin);
                s.o=intersectionPt;
                
                
                // for each pixel in 3D
                //    for each light source
                
                diffuselightoutput=quadrature(intersectionPt, n, r, bvhAccel, diffuseLight, cTheta, sTheta, cPhi, sPhi, SphereRadius, ndivs, groundAxis, !didIntersect);
                
                time(&endin);
                difinsphere+=difftime(endin, startin)*1.0/npixels;
                
                time(&startin);
                if (didIntersect) {
                    
                    //didfirstIntersect=false;
                    
                    r2.maxt=INFINITY; r2.mint=0;
                    
                    firstIntersectionPt=barycentrics[0]*VerticesFirstTransformed[Faces[3*shapeId]]+
                    barycentrics[1]*VerticesFirstTransformed[Faces[3*shapeId+1]]+
                    barycentrics[2]*VerticesFirstTransformed[Faces[3*shapeId+2]];
                    
                    r2.d=Normalize(Vector(firstIntersectionPt));
                    
                    didfirstIntersect=firstAccel->IntersectQ(r2, firstisect, firstbcs);
                    
                    didfirstIntersect=false;
                    if (didfirstIntersect && shapeId==firstisect->shapeId-2) {
                        x=firstIntersectionPt.x/(firstIntersectionPt.z*ifu)+u0;
                        y=firstIntersectionPt.y/(firstIntersectionPt.z*ifv)+v0;
                        
                        if (mtlnums) {
                            getTexVal(Itexartist, mtlnums, shapeId, &diffusereflectance);
                        } else {
                            bilinearInterp(x, y, height, Idiffuse, NULL, &diffusereflectance);
                        }
                        coloredoutput=Emult(diffuselightoutput,diffusereflectance)+Emult(ambientLight,diffusereflectance);
                        
                        bilinearInterp(x, y, height, Idifference, NULL, &appearancedifference);
                        
                        finaloutput=coloredoutput+appearancedifference;
                    } else {
                        
                        // diffuse albedo
                        if (mtlnums) {
                            getTexVal(Itexartist, mtlnums, shapeId, &diffusereflectance);
                        } else {
                            texturescale(vdtex, dmapheight, dmapwidth, &x, &y);
                            bilinearInterpTexture(x, y, dmapheight, dmapwidth,Itexartist, Btex, &diffusereflectance,ttshapeId,TextureFaces,TextureVertices,adjacencies);
                        }
                        
                        // original image value
                        
                        coloredoutput=Emult(diffuselightoutput,diffusereflectance)+Emult(ambientLight,diffusereflectance);
                        
                        if (Itexdiff !=NULL) {   // may need to change to dmapwidth/dmapheight etc.
                            // texture difference
                            if (mtlnums) {
                                dmapheight=NTEX;
                                dmapwidth=NTEX;
                            }
                            texturescale(vdtex, dmapheight, dmapwidth, &x, &y);
                            bilinearInterpTexture(x, y, dmapheight, dmapwidth,Itexdiff, Btex, &appearancedifference,ttshapeId,TextureFaces,TextureVertices,adjacencies);
                            //printf("x=%f, y=%f, dmapheight=%d, dmapwidth=%d, difference=[%f,%f,%f]\n",x,y,dmapheight,dmapwidth,
                            //       appearancedifference.x,appearancedifference.y,appearancedifference.z);
                            finaloutput=coloredoutput+appearancedifference;
                        } else {
                            finaloutput=coloredoutput;
                        }
                    }
                } else {
                    coloredoutput=Emult(diffuselightoutput,groundAlbedo)+Emult(ambientLight,ambientGroundAlbedo);
                    finaloutput=coloredoutput+Vector(Igroundtex[i3],Igroundtex[i3+1],Igroundtex[i3+2]);                    
                }
                
                lightoutputsamples[idxcol]=diffuselightoutput.x > 1.0f ? 1.0f : (diffuselightoutput.x < 0.0f ? .0f : diffuselightoutput.x); //finaloutputsamples.push_back(color.x);
                lightoutputsamples[idxcol1]=diffuselightoutput.y > 1.0f ? 1.0f : (diffuselightoutput.y < 0.0f ? .0f : diffuselightoutput.y); //finaloutputsamples.push_back(color.y);
                lightoutputsamples[idxcol2]=diffuselightoutput.z > 1.0f ? 1.0f : (diffuselightoutput.z < 0.0f ? .0f : diffuselightoutput.z); //finaloutputsamples.push_back(color.z);
                
                colorsamples[idxcol]=coloredoutput.x > 1.0f ? 1.0f : (coloredoutput.x < 0.0f ? .0f : coloredoutput.x); //colorsamples.push_back(colorwithtex.x);
                colorsamples[idxcol1]=coloredoutput.y > 1.0f ? 1.0f : (coloredoutput.y < 0.0f ? .0f : coloredoutput.y); //colorsamples.push_back(colorwithtex.y);
                colorsamples[idxcol2]=coloredoutput.z > 1.0f ? 1.0f : (coloredoutput.z < 0.0f ? .0f : coloredoutput.z); //colorsamples.push_back(colorwithtex.z);
                
                
                finaloutputsamples[idxcol]=finaloutput.x > 1.0f ? 1.0f : (finaloutput.x < 0.0f ? .0f : finaloutput.x); //finaloutputsamples.push_back(color.x);
                finaloutputsamples[idxcol1]=finaloutput.y > 1.0f ? 1.0f : (finaloutput.y < 0.0f ? .0f : finaloutput.y); //finaloutputsamples.push_back(color.y);
                finaloutputsamples[idxcol2]=finaloutput.z > 1.0f ? 1.0f : (finaloutput.z < 0.0f ? .0f : finaloutput.z); //finaloutputsamples.push_back(color.z);
                
                time(&endin);
                difincolor+=difftime(endin, startin)*1.0/npixels;
                
                if ( i % height==0) {
                    printf("%d of %d columns ( %3.2f %% ) done\n", i / height, width, i*100.0f/npixels);
                }
            }
        }
    }
    
    
    time(&end);
    dif=difftime(end,start);
    printf("Average times for goforth=%f, aboveground=%f, sphere=%f, color=%f\n", difingoforth, difinaboveground, difinsphere, difincolor);
    printf("Time taken for initial sample generation=%f seconds; %d (%f/100) pixels covered.\n",dif,npixels_covered,(100.0f*npixels_covered)/npixels);
    
    writeImage(direc, finaloutputsamples, width, height, string("finaloutput_").append(outputfilename).c_str());
    writeImage(direc, lightoutputsamples, width, height, string("lightoutput_").append(outputfilename).c_str());
    writeImage(direc, colorsamples, width, height, string("coloredoutput_").append(outputfilename).c_str());
    
	//path=direc; FILE* ffinalid=fopen(path.append("finaloutput_").append(outputfilename).c_str(),"w");
	//path=direc; FILE* fcoloredid=fopen(path.append("coloredoutput_").append(outputfilename).c_str(),"w");
	//path=direc; FILE* flightoutputid=fopen(path.append("lightoutput_").append(outputfilename).c_str(),"w");
	//path=direc; FILE* forigid=fopen(path.append("texturemappedpixels_").append(outputfilename).c_str(),"w");
	
    //fwrite(lightoutputsamples,sizeof(float)*nsamp*npixels*3,1,flightoutputid);
    //fwrite(colorsamples,sizeof(float)*nsamp*npixels*3,1,fcoloredid);
    //fwrite(finaloutputsamples,sizeof(float)*nsamp*npixels*3,1,ffinalid);
    
    //fclose( ffinalid );
    //fclose( fcoloredid );
    //fclose( flightoutputid );
    //fclose( forigid );
	
	delete[] colorsamples;
	delete[] finaloutputsamples;
	delete[] lightoutputsamples;
	delete[] Igroundobjmask;
	delete isect;
	delete lisect;
    //   if (khanmap) delete[] khanmap;
	
	
}