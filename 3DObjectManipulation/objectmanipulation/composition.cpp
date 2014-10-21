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
#include <opencv2/opencv.hpp>

void composition(BVHAccel* bvhAccel, Vector* vertexNormals, int* Faces,
                 PbrtPoint* TextureVertices, int* TextureFaces, PbrtPoint* TextureVertices_D, int* TextureFaces_D, int* adjacencies,// geometry
                 float* I, float* Itexartist, float* Itexdiff, float *Btex, float blendfactor, float* Igroundtex, bool* Igroundmask,
                 float* Idiffuse, float* Idifference, BVHAccel* firstAccel, PbrtPoint* VerticesFirstTransformed,
                 float u0, float v0, float ifu, float ifv, float* groundAxis,// camera parameters
                 float* cTheta, float* sTheta, float* cPhi, float* sPhi, float SphereRadius, Vector* diffuseLight, Vector& ambientLight,// light sphere parameters
                 Vector& albedo, Vector& groundAlbedo,
                 int width, int height, int nt, int nkernels, int ndivs, int nsamp, int shapeIdOffset, int dmapwidth, int dmapheight,
                 int filterhsize, int filtertype, float inv_sigma_sq, // filter parameters
                 string& path, string& direc, string& outputfilename, float* outputimage) { // extra parameters
	
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
                
                diffuselightoutput=quadrature(intersectionPt, n, r, bvhAccel, diffuseLight, cTheta, sTheta, cPhi, sPhi, SphereRadius, ndivs, groundAxis);
                
                time(&endin);
                difinsphere+=difftime(endin, startin)*1.0/npixels;
                
                time(&startin);
                if (didIntersect) {
                
                    didfirstIntersect=false;
                    
                    if (didfirstIntersect) {
                        x=firstIntersectionPt.x/(firstIntersectionPt.z*ifu)+u0;
                        y=firstIntersectionPt.y/(firstIntersectionPt.z*ifv)+v0;
                        
                        bilinearInterp(x, y, height, Idiffuse, NULL, &diffusereflectance);
                        coloredoutput=Emult(diffuselightoutput,diffusereflectance)+Emult(ambientLight,diffusereflectance);
                        
                        bilinearInterp(x, y, height, Idifference, NULL, &appearancedifference);
                        
                        finaloutput=coloredoutput+appearancedifference;
                    } else {
                        texturescale(vdtex, dmapheight, dmapwidth, &x, &y);
                        
                        
                        // diffuse albedo
                        bilinearInterpTexture(x, y, dmapheight, dmapwidth,Itexartist, Btex, &diffusereflectance,ttshapeId,TextureFaces,TextureVertices,adjacencies);
                        
                        // original image value
                        
                        coloredoutput=Emult(diffuselightoutput,diffusereflectance)+Emult(ambientLight,diffusereflectance);
                        
                        if (Itexdiff !=NULL) {   // may need to change to dmapwidth/dmapheight etc.
                            // texture difference
                            bilinearInterpTexture(x, y, dmapheight, dmapwidth,Itexdiff, Btex, &appearancedifference,ttshapeId,TextureFaces,TextureVertices,adjacencies);
                            finaloutput=coloredoutput+appearancedifference;
                            
                        } else {
                            finaloutput=coloredoutput;
                        }
                    }
                } else {
                    coloredoutput=Emult(diffuselightoutput,groundAlbedo)+Emult(ambientLight,groundAlbedo);
                    finaloutput=coloredoutput+Vector(Igroundtex[i3],Igroundtex[i3+1],Igroundtex[i3+2]);
                    
                }
                
                lightoutputsamples[idxcol]=diffuselightoutput.x > 1.0f ? 1.0f : diffuselightoutput.x; //finaloutputsamples.push_back(color.x);
                lightoutputsamples[idxcol1]=diffuselightoutput.y > 1.0f ? 1.0f : diffuselightoutput.y; //finaloutputsamples.push_back(color.y);
                lightoutputsamples[idxcol2]=diffuselightoutput.z > 1.0f ? 1.0f : diffuselightoutput.z; //finaloutputsamples.push_back(color.z);
                
                colorsamples[idxcol]=coloredoutput.x > 1.0f ? 1.0f : coloredoutput.x; //colorsamples.push_back(colorwithtex.x);
                colorsamples[idxcol1]=coloredoutput.y > 1.0f ? 1.0f : coloredoutput.y; //colorsamples.push_back(colorwithtex.y);
                colorsamples[idxcol2]=coloredoutput.z > 1.0f ? 1.0f : coloredoutput.z; //colorsamples.push_back(colorwithtex.z);
                
                
                finaloutputsamples[idxcol]=finaloutput.x; //finaloutputsamples.push_back(color.x);
                finaloutputsamples[idxcol1]=finaloutput.y; //finaloutputsamples.push_back(color.y);
                finaloutputsamples[idxcol2]=finaloutput.z; //finaloutputsamples.push_back(color.z);
                
                time(&endin);
                difincolor+=difftime(endin, startin)*1.0/npixels;
                
                
            }
        }
        if ( i % height==0) {
            printf("%d of %d columns ( %3.2f %% ) done\n", i / height, width, i*100.0f/npixels);
        }
    }
    
    
    time(&end);
    dif=difftime(end,start);
    
    printf("Cleaning up...\n");
    int resizefactor=2;
    int reswidth= (int)(width/(float)(resizefactor));
    int resheight=(int)(height/(float)(resizefactor));
    float filtersigma=(float)resizefactor/2.0;
    float inv_sigma_squared=1./(2*filtersigma*filtersigma);
    filterhsize=ceil(3*filtersigma);
    int filterwidth=2*filterhsize+1;
    
    float* blurredoutputimage=new float[3*npixels];
    for (int i=0; i<npixels; i++) {
        int xx=i / height;
        int yy=i % height;
        
        blurredoutputimage[i]=.0f;
        float sumweight=.0f;
        for (int sx=-2*filterhsize; sx<=2*filterhsize; sx++) {
            for (int sy=-2*filterhsize; sy<=2*filterhsize; sy++) {
                int xpiximnb=xx+sx; int ypiximnb=yy+sy;
                if (xpiximnb>=0 && xpiximnb<width && ypiximnb>=0 && ypiximnb<height) {
                    int idxnb=((xpiximnb)*height+(ypiximnb));
                    
                    float weight=exp(-inv_sigma_squared*(sx*sx+sy*sy));
                    blurredoutputimage[3*i]+=weight*finaloutputsamples[3*idxnb];
                    blurredoutputimage[3*i+1]+=weight*finaloutputsamples[3*idxnb+1];
                    blurredoutputimage[3*i+2]+=weight*finaloutputsamples[3*idxnb+2];
                    sumweight+=weight;
                }
            }
        }
        blurredoutputimage[3*i]/=sumweight;
        blurredoutputimage[3*i]=blurredoutputimage[3*i]<0.0f ? 0.0f : (blurredoutputimage[3*i] > 1 ? 1.0f : blurredoutputimage[3*i]);
        blurredoutputimage[3*i+1]/=sumweight;
        blurredoutputimage[3*i+1]=blurredoutputimage[3*i+1]<0.0f ? 0.0f : (blurredoutputimage[3*i+1] > 1 ? 1.0f : blurredoutputimage[3*i+1]);
        blurredoutputimage[3*i+2]/=sumweight;
        blurredoutputimage[3*i+2]=blurredoutputimage[3*i+2]<0.0f ? 0.0f : (blurredoutputimage[3*i+2] > 1 ? 1.0f : blurredoutputimage[3*i+2]);
        
        if (xx % 2==0 && yy % 2==0) {
            memcpy(&outputimage[3*( (xx/2)*(resheight)+(yy/2) )], &blurredoutputimage[3*i], 3*sizeof(float));
        }
     }
    
    /*
    for (int w=0; w<reswidth; w++) {
        for (int h=0; h<resheight; h++) {
            int outputindex=w*resheight+h;
            int inputindex=2*w*height+2*h;
            memcpy(&outputimage[3*outputindex], &blurredoutputimage[3*inputindex], 3*sizeof(float));
        }
    }
    */
     
    /*
    cv::Size resizedSize( reswidth,resheight );
    int resizefactor=2;
    double filtersigma=resizefactor/2.0;
    int filterwidth=ceil(6*filtersigma)+1;
    cv::Size filterSize(filterwidth,filterwidth);
    

    path=direc;
    cv::Mat src( width,height,CV_32FC3);
    cv::Mat dst=src.clone();
    cv::GaussianBlur(src, dst, filterSize, filtersigma);
    
    
    cv::Mat_<cv::Vec3b> cvFinalOutputSamples(width,height,cv::Vec3b(0,0,0));
    cv::Mat_<cv::Vec3b> cvBlurFinalOutputSamples(width,height,cv::Vec3b(0,0,0));
    cv::Mat_<cv::Vec3b> cvBlurResizedFinalOutputSamples(reswidth,resheight,cv::Vec3b(0,0,0));
    
    
        cv::GaussianBlur(cvFinalOutputSamples, cvBlurFinalOutputSamples, filterSize, filtersigma);
    memcpy(&(cvFinalOutputSamples.data), finaloutputsamples, 3*sizeof(float)*npixels);
    
    
    cv::resize(cvBlurFinalOutputSamples, cvBlurResizedFinalOutputSamples, resizedSize);
    memcpy(outputimage, &(cvBlurResizedFinalOutputSamples.data), 3*sizeof(float)*reswidth*resheight);
    */
    //memcpy(outputimage, finaloutputsamples, 3*sizeof(float)*npixels);
    
    writeImage(direc, outputimage, reswidth, resheight, string("finaloutput_").append(outputfilename).c_str());
    writeImage(direc, lightoutputsamples, width, height, string("lightoutput_").append(outputfilename).c_str());
    writeImage(direc, colorsamples, width, height, string("coloredoutput_").append(outputfilename).c_str());
    printf("Rendering done, wrote: %s, %s, %s in directory %s\n", string("finaloutput_").append(outputfilename).c_str(),
           string("lightoutput_").append(outputfilename).c_str(), string("coloredoutput_").append(outputfilename).c_str(),
           direc.c_str());
    
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