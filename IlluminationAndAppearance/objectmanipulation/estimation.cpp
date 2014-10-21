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

#include "estimation.h"
#include <Accelerate/Accelerate.h>

void extractPixelSpaceAppearanceAndMasks( BVHAccel* bvhAccel, PbrtPoint* TextureVertices, int* TextureFaces, int* adjacencies,// geometry
                                         int* mtlnums, float* Itexartist, float* Btex, // image data structure
                                         float u0, float v0, float ifu, float ifv, // camera parameters
                                         int width, int height, int shapeIdOffset, int dmapwidth, int dmapheight, // extra parameters
                                         float* PixelSpaceAppearance, IplImage* binaryImage, IplImage* erodedImage, IplImage* middilatedImage )
{
    
    int xpixim, ypixim;
    float xpix, ypix;
	Vector direction(.0,.0,1.0);
	PbrtPoint origin(.0,.0,.0);
	PbrtPoint intersectionPt, vdtex;
	Ray r(origin,direction,0,INFINITY);
    bool didIntersect;
    Intersection isect;
    float barycentrics[3];
    int shapeId;
    
    float x,y;
    int npixels=width*height;
    
    for (int i=0; i<npixels; i++) {
		// shoot a ray from the camera center through the pixel (i.e. [x2d,y2d,1])
		ypixim=(i % height);
		xpixim=(i / height);
		
		xpix=(float)xpixim;
		ypix=(float)ypixim;
		
		r.d.x=(xpix-u0)*ifu;
		r.d.y=(ypix-v0)*ifv;
		r.maxt=INFINITY;
		r.mint=0;
		didIntersect=bvhAccel->IntersectQ(r,&isect,barycentrics);
        PixelSpaceAppearance[3*i]=.0f;
        PixelSpaceAppearance[3*i+1]=.0f;
        PixelSpaceAppearance[3*i+2]=.0f;
        
        cvSet2D(binaryImage, xpixim, ypixim, cvScalar( (float)didIntersect) );
        
        if (didIntersect) {
            shapeId=isect.shapeId-shapeIdOffset;
            vdtex=barycentrics[0]*TextureVertices[TextureFaces[3*shapeId]]+
            barycentrics[1]*TextureVertices[TextureFaces[3*shapeId+1]]+
            barycentrics[2]*TextureVertices[TextureFaces[3*shapeId+2]];
            if (mtlnums) {
                Vector outputColor;
                getTexVal(Itexartist, mtlnums, shapeId, &outputColor);
                PixelSpaceAppearance[3*i]=outputColor.x;
                PixelSpaceAppearance[3*i+1]=outputColor.y;
                PixelSpaceAppearance[3*i+2]=outputColor.z;
            } else {
                texturescale(vdtex, dmapheight, dmapwidth, &x, &y);
                bilinearInterpTexture(x, y, dmapheight, dmapwidth,Itexartist, Btex, &PixelSpaceAppearance[3*i],shapeId,TextureFaces,TextureVertices,adjacencies);
            }
        }
    }
    
    IplConvKernel* elem;
    
    elem=cvCreateStructuringElementEx(3,3,1,1,CV_SHAPE_RECT);
    cvErode(binaryImage, erodedImage, elem, 1);
    cvReleaseStructuringElement(&elem);
    
    elem=cvCreateStructuringElementEx(5,5,2,2, CV_SHAPE_RECT);
    cvDilate(binaryImage, middilatedImage, elem, 1);
    cvReleaseStructuringElement(&elem);
}

void iterativeIlluminationAppearanceEstimate(BVHAccel* bvhAccel, Vector* vertexNormals, int* Faces,
                                             float* I, bool* Igroundmask, bool* Ishadowmask, float* Ktranspose, bool* B,
                                             float* PixelSpaceAppearance, IplImage* binaryImage, IplImage* erodedImage, IplImage* middilatedImage,
                                             float u0, float v0, float ifu, float ifv, float fu, float fv, float* groundAxis,// camera parameters
                                             Vector** diffuseLight, Vector& ambientLight, // light sphere parameters
                                             float* cTheta, float* sTheta, float* cPhi, float* sPhi, float SphereRadius,
                                             int* materialNumImage, float* adjustedmaterials, int* mtlnums, Vector& albedo, Vector& groundAlbedo, Vector& ambientGroundAlbedo,
                                             param& parameters,
                                             int width, int height, int nkernels, int ndivs, int shapeIdOffset, int dmapwidth, int dmapheight, int nt,
                                             string& path, string& direc, string& outputfilename, int lightType,
                                             float** diffuseLightOutput, float** groundNoObjLightOutput)
{
    float lightLambdaError=.0f;
    for (int iter=0; iter<parameters.numIterations; iter++) {
        // iterate a bit hither and thither
        
        // assume to begin with that diffuse artist texture == ambient artists texture == specular artists texture
        
        printf("Light phase\n");
        /*
        
        path=direc; FILE* fid=fopen(path.append("diffuseLight.dat").c_str(),"r");
        for (int j=0; j<ndivs; j++) {
            fread(&diffuseLight[0][j].x, 1, sizeof(float), fid);
            fread(&diffuseLight[0][j].y, 1, sizeof(float), fid);
            fread(&diffuseLight[0][j].z, 1, sizeof(float), fid);
        }
        fclose(fid);
        path=direc; fid=fopen(path.append("ambientLight.dat").c_str(), "r");
        fread(&ambientLight.x,1,sizeof(float),fid);
        fread(&ambientLight.y,1,sizeof(float),fid);
        fread(&ambientLight.z,1,sizeof(float),fid);
        fclose(fid);
        */

        estimateIllumination(bvhAccel, vertexNormals, Faces, // geometry
                             I,	Igroundmask, Ishadowmask,// image data structure
                             PixelSpaceAppearance, binaryImage, erodedImage,
                             u0,  v0,  ifu,  ifv, groundAxis,// camera parameters
                             SphereRadius, Ktranspose, B, // light sphere parameters
                             cTheta, sTheta, cPhi, sPhi,
                             mtlnums, albedo, groundAlbedo,ambientGroundAlbedo,
                             parameters,
                             width, height,  nkernels,  ndivs,  shapeIdOffset, dmapwidth, dmapheight,
                             path, direc, iter, lightLambdaError,
                             diffuseLight, ambientLight, lightType);
        
        printf("Texture phase\n");
        estimateAppearance(bvhAccel, vertexNormals, Faces,
                           I, Igroundmask, Ishadowmask, Ktranspose, B,
                           PixelSpaceAppearance, binaryImage, erodedImage, middilatedImage,
                           u0,  v0,  ifu,  ifv,  fu,  fv,  groundAxis,// camera parameters
                           diffuseLight, ambientLight,// light sphere parameters
                           cTheta, sTheta, cPhi, sPhi, SphereRadius,
                           materialNumImage, adjustedmaterials, mtlnums, albedo,  groundAlbedo, ambientGroundAlbedo,
                           parameters,
                           width, height, ndivs, shapeIdOffset, dmapwidth, dmapheight,nt,
                           path, direc, outputfilename, iter,lightLambdaError,
                           diffuseLightOutput,groundNoObjLightOutput);
        
    }
}

void estimateAppearance(BVHAccel* bvhAccel, Vector* vertexNormals, int* Faces,
                        float* I, bool* Igroundmask, bool* Ishadowmask, float* Ktranspose, bool* B,
                        float* PixelSpaceAppearance, IplImage* binaryImage, IplImage* erodedImage, IplImage* middilatedImage,
                        float u0, float v0, float ifu, float ifv, float fu, float fv, float* groundAxis,// camera parameters
                        Vector** diffuseLight, Vector& ambientLight, // light sphere parameters
                        float* cTheta, float* sTheta, float* cPhi, float* sPhi, float SphereRadius,
                        int* materialNumImage, float* adjustedmaterials, int* mtlnums, Vector& albedo, Vector& groundAlbedo, Vector& ambientGroundAlbedo,
                        param& parameters,
                        int width, int height, int ndivs, int shapeIdOffset, int dmapwidth, int dmapheight, int nt,
                        string& path, string& direc, string& outputfilename, int iternum, float lightLambdaError,
                        float** diffuseLightOutput, float** groundNoObjLightOutput) {	// extra parameters
	
	// Declarations
	
	int i, j,  shapeId;
	bool didIntersect;
	float xpix, ypix, xpixim, ypixim, lambda;
	float barycentrics[3];
	time_t start, end;
    int ntexpixel=dmapwidth*dmapheight;
	//bool aboveground=true;
	FILE* fid;
	//int nlight=0;
    int npixels=width*height;
	
	
	//float** diffuseLightOutput=new float*[1];
	//float** groundNoObjLightOutput=new float*[1];
	//diffuseLightOutput[0]=new float[3*npixels];
	//groundNoObjLightOutput[0]=new float[3*npixels];
	
	Vector* diffuseColors=new Vector[1];
	Vector* groundColorsNoObj=new Vector[1];
	Intersection* isect=new Intersection();
	
	Vector direction(.0,.0,1.0), lightDirection, color, endcolor, n, reflection, bilerpedalb, bilerpedambalb, bilerpedspecalb;
	PbrtPoint origin(.0,.0,.0);
	PbrtPoint intersectionPt, vdtex;
	Ray r(origin,direction,0,INFINITY);
	Ray s(intersectionPt,lightDirection,0,INFINITY);
	
	
	
    // THIS PART SETS UP THE MATERIAL FOR PERFORMING A SOLVE ON THE REFLECTANCE VALUES IN MATLAB
    // COMPUTES DIFFUSE LIGHT OUTPUT, SPECULAR LIGHT OUTPUT, AND REFLECTANCE VALUES FOR EVERYTHING
    
	FILE* ftransfer, *falbedo;
	path=direc; path.append("MyLightOutput.dat"); ftransfer=fopen(path.c_str(), "w");
	path=direc; path.append("AlbedoPixels.txt"); falbedo=fopen(path.c_str(), "w");
	
	// Light calculation phase (does not involve texture map)
	time(&start);
	//nlight=0;
	//Vector specularColor(.0f,.0f,.0f);
	//Vector specularAlbedo(0.752941,0.752941,0.752941);
	
	char* mychar=new char[4];
	
	
	float* useTex;
	path=direc; fid=fopen(path.append("TexUseMask.dat").c_str(), "r");
	if (fid==0) useTex=NULL; else {
		useTex=new float[ntexpixel];
		fread(useTex, sizeof(float), ntexpixel, fid);
	}
	
	
	int nobj=0;
	//float* diffuseAlbedo=new float[npixels*3];
	//float* ambientAlbedo=new float[npixels*3];
	//float* colordifference=new float[npixels*3];
    
	int* fgroups=NULL;
	path=direc;fid=fopen(path.append("facegroups.dat").c_str(), "r");
	if (fid) {
		fgroups=new int[nt];
		fread(fgroups, nt, sizeof(int), fid);
		fclose(fid);
	}
    
    
	nobj=0;
    int nstep=parameters.nstepappearance;
	
    for (i=0; i<npixels; i++) {
        if (i==5424684) {
            printf("%d\n",i);
        }
		endcolor.x=.0; endcolor.y=.0; endcolor.z=.0;
		// shoot a ray from the camera center through the pixel (i.e. [x2d,y2d,1])
		ypixim=(i % height);
		xpixim=(i / height);
		
		// first check intersection; if intersected,
		color.x=.0; color.y=.0; color.z=.0;
		xpix=xpixim;
		ypix=ypixim;
		
		r.d.x=(xpix-u0)*ifu;
		r.d.y=(ypix-v0)*ifv;
		r.maxt=INFINITY;
		r.mint=0;
		didIntersect=bvhAccel->IntersectQ(r,isect,barycentrics);
        materialNumImage[i]=-1;
        
        if (didIntersect || Igroundmask[i]) {
			//aboveground=true;
			PbrtPoint groundPt(.0f,.0f,.0f);
			Vector groundn(.0f,.0f,.0f);
			if (!didIntersect) {
				// if the ray does not intersect with the object, it belongs to the
				// ground
				lambda=-groundAxis[3]*1.0f/(groundAxis[0]*r.d.x+groundAxis[1]*r.d.y+groundAxis[2]);
				intersectionPt.x=lambda*r.d.x; intersectionPt.y=lambda*r.d.y; intersectionPt.z=lambda;
				n=Vector(groundAxis[0],groundAxis[1],groundAxis[2]);
				groundPt=intersectionPt;
				groundn=n;
			} else {
				intersectionPt=r.o+r.d*r.maxt;
				shapeId=isect->shapeId-shapeIdOffset;
				n=Normalize(barycentrics[0]*vertexNormals[Faces[3*shapeId]]+
							barycentrics[1]*vertexNormals[Faces[3*shapeId+1]]+
							barycentrics[2]*vertexNormals[Faces[3*shapeId+2]]);
				
				lambda=-groundAxis[3]*1.0f/(groundAxis[0]*r.d.x+groundAxis[1]*r.d.y+groundAxis[2]);
				groundPt.x=lambda*r.d.x; groundPt.y=lambda*r.d.y; groundPt.z=lambda;
				groundn=Vector(groundAxis[0],groundAxis[1],groundAxis[2]);
                if (mtlnums) { materialNumImage[i]=mtlnums[shapeId]; }
			}
			
			if ( didIntersect ||
                ( !didIntersect && ( Ishadowmask[i] || ( (int)xpixim % nstep==0 && (int)ypixim % nstep==0) ) ) ) {
				quadrature(intersectionPt, n, r, bvhAccel, diffuseLight, cTheta, sTheta, cPhi, sPhi, SphereRadius, ndivs, groundAxis, !didIntersect, diffuseColors, groundPt, groundn, groundColorsNoObj);
				
				fwrite(&(diffuseColors[0].x), sizeof(float), 1, ftransfer);
				fwrite(&(diffuseColors[0].y), sizeof(float), 1, ftransfer);
				fwrite(&(diffuseColors[0].z), sizeof(float), 1, ftransfer);
			} else {
				quadrature(intersectionPt, n, r, bvhAccel, diffuseLight, cTheta, sTheta, cPhi, sPhi, SphereRadius, ndivs, groundAxis, !didIntersect, diffuseColors, groundPt, groundn, groundColorsNoObj);
			}
			
			diffuseLightOutput[0][3*i]=diffuseColors[0].x; diffuseLightOutput[0][3*i+1]=diffuseColors[0].y; diffuseLightOutput[0][3*i+2]=diffuseColors[0].z;
            groundNoObjLightOutput[0][3*i]=groundColorsNoObj[0].x; groundNoObjLightOutput[0][3*i+1]=groundColorsNoObj[0].y; groundNoObjLightOutput[0][3*i+2]=groundColorsNoObj[0].z;
			
			if (didIntersect) {
                fprintf(falbedo, "%f %f %f %d %d %d %f %f %f %f %f %f %f %f %f %f %d\n",
                        PixelSpaceAppearance[3*i], PixelSpaceAppearance[3*i+1], PixelSpaceAppearance[3*i+2],
                        i+1, 1, 0,
                        I[3*i], I[3*i+1], I[3*i+2],
                        PixelSpaceAppearance[3*i], PixelSpaceAppearance[3*i+1], PixelSpaceAppearance[3*i+2],
                        .0f, .0f, .0f,
                        1.0f, mtlnums ? (mtlnums[shapeId]+1) : (fgroups==NULL ? 1 : fgroups[shapeId]));
                nobj++;
			} else {
				if (Ishadowmask[i] || ( (int)xpixim % nstep==0 && (int)ypixim % nstep==0)) {
					fprintf(falbedo, "%f %f %f %d %d %d %f %f %f %f %f %f %f %f %f %f %d\n",
                            groundAlbedo.x, groundAlbedo.y, groundAlbedo.z,
							i+1, 0, 0,
                            I[3*i], I[3*i+1], I[3*i+2],
							ambientGroundAlbedo.x,ambientGroundAlbedo.y,ambientGroundAlbedo.z,
                            .0f, .0f, .0f,
                            Ishadowmask[i] ? 1.0 : parameters.shadowweight, mtlnums ? 0 : -1
							);
                    
				}
			}
			if ((i % 10000)==0) printf("Iternum=%d, i=%d\n",iternum,i);
		}
		
	}
    
    fclose(ftransfer); fclose(falbedo);
#ifdef OPTIMIZE_REFLECTANCE
    mxArray* name=mxCreateStringFromNChars_730(direc.c_str(), direc.length());
	mxArray* mxHasSpecular=mxCreateDoubleMatrix(1,1,mxREAL);
	mxArray *mxheight=mxCreateDoubleMatrix(1,1,mxREAL);
	mxArray *mxwidth=mxCreateDoubleMatrix(1,1,mxREAL);
	Engine* workengine=NULL;
    
    
	for (j=0; j<npixels*3; j++) {
		PixelSpaceAppearance[j]=.0f;
	}
	
    
    // THIS PART ESTIMATES THE REFLECTANCES IN MATLAB
    
	double* prHasSpecular=mxGetPr(mxHasSpecular);
	*prHasSpecular=.0;
	double* prwidth=mxGetPr(mxwidth), *prheight=mxGetPr(mxheight);
	*prwidth=width; *prheight=height;
	
	mxArray* mxIternum=mxCreateDoubleMatrix(1,1,mxREAL);
	double* priternum=mxGetPr(mxIternum);
	*priternum=iternum;
    
    mxArray* mxLambdaRho=mxCreateDoubleMatrix(1,1,mxREAL);
    double* prLambdaRho=mxGetPr(mxLambdaRho);
    *prLambdaRho=parameters.lambda_rho;
    mxArray* mxLambdaRho2=mxCreateDoubleMatrix(1,1,mxREAL);
    double* prLambdaRho2=mxGetPr(mxLambdaRho2);
    *prLambdaRho2=parameters.lambda_rho2;
    mxArray* mxAmbient=mxCreateDoubleMatrix(1,3,mxREAL);
    double* prAmbient=mxGetPr(mxAmbient);
    prAmbient[0]=ambientLight[0]; prAmbient[1]=ambientLight[1]; prAmbient[2]=ambientLight[2];
    
    printf("Opened MATLAB\n");
	workengine=engOpen("matlab -nosplash");
	engPutVariable(workengine, "name", name);
	engPutVariable(workengine, "height", mxheight);
	engPutVariable(workengine, "width", mxwidth);
	engPutVariable(workengine, "hasSpecular", mxHasSpecular);
	engPutVariable(workengine, "iternum", mxIternum);
	engPutVariable(workengine, "lambda_rho", mxLambdaRho);
	engPutVariable(workengine, "lambda_rho2", mxLambdaRho2);
    engPutVariable(workengine, "ambient", mxAmbient);
    
    if (mtlnums) {
        engEvalString(workengine, "globalOptimMaterials; clearvars -except ishinebest errout errfit;");
    } else {
        engEvalString(workengine, "globalOptimLightAlbedo5chair; clearvars -except errout errfit;");
    }
    
    mxArray* mxErrorTotal=engGetVariable(workengine, "errout");
    mxArray* mxErrorFit=engGetVariable(workengine, "errfit");
    
    double* prET=mxGetPr(mxErrorTotal);
    double* prEF=mxGetPr(mxErrorFit);
    
    printf("Iteration=%d, total error=%f, fit error=%f\n", iternum, *prET+lightLambdaError, *prEF);
    
    mxDestroyArray(mxErrorFit);
    mxDestroyArray(mxErrorTotal);
    
    if (mtlnums) {
        readFile(direc, adjustedmaterials, 3*sizeof(float)*dmapwidth*dmapheight, "materialDiffuseNew.dat");
        FILE* frho;
        path=direc;frho=fopen(path.append("materialDiffuseGroundNew.dat").c_str(),"r");
        fread(&groundAlbedo.x,sizeof(float),1,frho);
        fread(&groundAlbedo.y,sizeof(float),1,frho);
        fread(&groundAlbedo.z,sizeof(float),1,frho);
        fclose(frho);
        ambientGroundAlbedo=Vector(groundAlbedo);
    } else {
        path=direc; path.append("albedoout.dat");
        float* inrho=new float[3*(nobj+1)]; float* positions=new float[nobj+1];
        
        FILE* frho=fopen(path.c_str(), "r");
        fread(inrho, 3*(nobj+1), sizeof(float), frho);
        fread(positions, nobj+1, sizeof(float), frho);
        fclose(frho);
        float inpos;
        for (j=0; j<nobj; j++) {
            inpos=positions[j];
            PixelSpaceAppearance[3*(int)inpos]=inrho[j];
            PixelSpaceAppearance[3*(int)inpos+1]=inrho[j+(nobj+1)];
            PixelSpaceAppearance[3*(int)inpos+2]=inrho[j+2*(nobj+1)];
        }
        groundAlbedo.x=inrho[nobj];
        groundAlbedo.y=inrho[nobj+nobj+1];
        groundAlbedo.z=inrho[nobj+2*(nobj+1)];
        
        ambientGroundAlbedo=Vector(groundAlbedo);
        
        delete[] inrho; delete[] positions;
    }
	
	printf("groundAlbedo = [%f,%f,%f], ambientGroundAlbedo = [%f,%f,%f]\n",groundAlbedo.x,groundAlbedo.y,groundAlbedo.z,ambientGroundAlbedo.x,ambientGroundAlbedo.y,ambientGroundAlbedo.z);
    time(&end);
	double dif;
	dif=difftime(end,start);
	printf("Time taken for sample generation: %f seconds.\n",dif);
	engClose(workengine);
    
    mxDestroyArray(name);
	mxDestroyArray(mxHasSpecular);
	mxDestroyArray(mxheight);
	mxDestroyArray(mxwidth);
    mxDestroyArray(mxAmbient);
#else
    printf("Not optimizing reflectances\n");
#endif
    
	delete[] mychar;
    delete[] diffuseColors;
	delete[] groundColorsNoObj;
	
}

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
                        int width, int height, int nkernels, int ndivs, int nsamp, int shapeIdOffset, int texIdOffset, int dmapwidth, int dmapheight, int nv, int nt,
                        string& path, string& direc, string& outputfilename,
                        float o2w[4][4]) {
    
    // THIS PART USES THE REFLECTANCES COMPUTED IN MATLAB TO COMPUTE
    // THE GROUND DIFFERENCE IN PIXEL SPACE AND
    // THE OBJECT DIFFERENCE IN PIXEL SPACE
    
    Vector endcolor, color;
    int xpixim, ypixim;
    time_t start;
    Intersection* isect=new Intersection();
	Intersection* lisect=new Intersection();
    int npixels=width*height;
    
    int i, j,  shapeId;
	bool didIntersect;
	float barycentrics[3];
	//bool aboveground=true;
	FILE* fid;
    
    
	Vector direction(.0,.0,1.0), lightDirection, n, reflection, bilerpedalb, bilerpedambalb, bilerpedspecalb;
	PbrtPoint origin(.0,.0,.0);
	PbrtPoint intersectionPt, vdtex;
	Ray r(origin,direction,0,INFINITY);
	Ray s(intersectionPt,lightDirection,0,INFINITY);
    
    int* fgroups=NULL;
	path=direc;fid=fopen(path.append("facegroups.dat").c_str(), "r");
	if (fid) {
		fgroups=new int[nt];
		fread(fgroups, nt, sizeof(int), fid);
		fclose(fid);
	}
    
#ifdef OPTIMIZE_REFLECTANCE
    if (mtlnums) {
        memcpy(Itexartist, adjustedmaterials, 3*sizeof(float)*dmapheight*dmapwidth);
    }
#endif
    
	for (i=0; i<npixels; i++) {
		colordifference[3*i]=.0f; colordifference[3*i+1]=.0f; colordifference[3*i+2]=.0f;
		endcolor.x=.0; endcolor.y=.0; endcolor.z=.0;
		// shoot a ray from the camera center through the pixel (i.e. [x2d,y2d,1])
		ypixim=(i % height);
		xpixim=(i / height);
		
		// first check intersection; if intersected,
		color.x=.0; color.y=.0; color.z=.0;
		//xpix=xpixim;
		//ypix=ypixim;
		
		// Modify Igroundtex only if you've reached the end of all your light_albedo_iterations.
		
		if (*(cvGet2D(middilatedImage, xpixim, ypixim)).val<.5f && !Igroundmask[i]) {
			// no foreground, no ground -- pull pixels from regular original image
			// Itarget == Isource
			
		} else {
			if (*(cvGet2D(middilatedImage, xpixim, ypixim)).val>.5f) {
				//foreground
                Vector alb;
                if (mtlnums) {
                    getTexValImage(Itexartist, materialNumImage, i, &alb);
                } else {
                    alb.x=PixelSpaceAppearance[3*i];
                    alb.y=PixelSpaceAppearance[3*i+1];
                    alb.z=PixelSpaceAppearance[3*i+2];
                }
				color.x=diffuseLightOutput[0][3*i]*alb.x+ambientLight.x*alb.x;
				color.y=diffuseLightOutput[0][3*i+1]*alb.y+ambientLight.y*alb.y;
				color.z=diffuseLightOutput[0][3*i+2]*alb.z+ambientLight.z*alb.z;
				
				colordifference[3*i]=I[3*i]-color.x; colordifference[3*i+1]=I[3*i+1]-color.y; colordifference[3*i+2]=I[3*i+2]-color.z;
				if (Igroundmask[i]) {
					// foreground and ground --- pixels will be blended in from the light-subtracted background
					// Itarget==Isource
					//
					// DON"T COMMENT OUT EITHER IF YOU WANT TO ESTIMATE OR IF YOU WANT TO USE PHOTOSHOP FILLED GROUND
					Igroundtex[3*i]=Igroundtex[3*i]-(groundAlbedo.x*groundNoObjLightOutput[0][3*i]+ambientLight.x*ambientGroundAlbedo.x);
                    Igroundtex[3*i+1]=Igroundtex[3*i+1]-(groundAlbedo.y*groundNoObjLightOutput[0][3*i+1]+ambientLight.y*ambientGroundAlbedo.y);
                    Igroundtex[3*i+2]=Igroundtex[3*i+2]-(groundAlbedo.z*groundNoObjLightOutput[0][3*i+2]+ambientLight.z*ambientGroundAlbedo.z);					
				}
			} else {
				// no foreground, and ground --- pull pixels from the light-subtracted original image
				// Itarget does not equal Isource
				color.x=diffuseLightOutput[0][3*i]*groundAlbedo.x+ambientLight.x*ambientGroundAlbedo.x;
				color.y=diffuseLightOutput[0][3*i+1]*groundAlbedo.y+ambientLight.y*ambientGroundAlbedo.y;
				color.z=diffuseLightOutput[0][3*i+2]*groundAlbedo.z+ambientLight.z*ambientGroundAlbedo.z;
                
				Igroundtex[3*i]=Igroundtex[3*i]-(groundAlbedo.x*groundNoObjLightOutput[0][3*i]+ambientLight.x*ambientGroundAlbedo.x);
                Igroundtex[3*i+1]=Igroundtex[3*i+1]-(groundAlbedo.y*groundNoObjLightOutput[0][3*i+1]+ambientLight.y*ambientGroundAlbedo.y);
                Igroundtex[3*i+2]=Igroundtex[3*i+2]-(groundAlbedo.z*groundNoObjLightOutput[0][3*i+2]+ambientLight.z*ambientGroundAlbedo.z);
			}
		}
		
	}
	
	//writeFile(direc, Igroundtex, 3*npixels*sizeof(float), "Igroundtex1.dat");
    
	
    // THIS PART PROPAGATES THE OBJECT DIFFERENCE IN PIXEL SPACE TO TEXEL SPACE
    // AND THE SURFACE REFLECTANCE IN PIXEL SPACE TO TEXEL SPACE
    
	int hwinsize=parameters.texturewinsize;
    if (mtlnums) {
        dmapwidth=NTEX;
        dmapheight=NTEX;
    }
    printf("dmapwidth=%d, dmapheight=%d\n",dmapwidth,dmapheight);
    int ntexpixel=dmapwidth*dmapheight;
    
	Vector bilerpedcolorsm;
    
    IplImage* erodedTexMask2=cvCreateImage(cvSize(dmapheight,dmapwidth),IPL_DEPTH_32F,1);
    IplImage* erodedTexMask=cvCreateImage(cvSize(dmapheight,dmapwidth),IPL_DEPTH_32F,1);
    IplImage* texMask=cvCreateImage(cvSize(dmapheight,dmapwidth),IPL_DEPTH_32F,1);
    
    int texx, texy;
    PbrtPoint vertex, vertext, vertexorig;
    float x, y;
    Vector bilerpedcolor, norig;
    Vector bilerpedcolororig;
    
    float barycentricstest[3];
    
    time(&start);
    
    float* pts3D=new float[3*ntexpixel];
    int* present=new int[ntexpixel];
    int* faceIndex=new int[ntexpixel];
    int* groupIndex=new int[ntexpixel];
    for (i=0; i<ntexpixel; i++) groupIndex[i]=-1;
    
    int xdisp[5]={-1,0,1,0};
    int ydisp[5]={0,-1,0,1};
    
    int nseen=0, nunseen=0, nunseenuse=0;
    
    
    int* Idx=new int[ntexpixel];
    int* Idxseen=new int[ntexpixel];
    Vector* normals=new Vector[ntexpixel];
    
    /*
    path=direc; FILE* funique=fopen(path.append("artisttextureunique.dat").c_str(),"r");
    bool existsunique=funique !=0;
    float* Itunique=NULL;
    if (existsunique) {
        Itunique=new float[ntexpixel*3];
        fread(Itunique,3*ntexpixel,sizeof(float),funique);
        fclose(funique);
        memcpy(Itexartist,Itunique,3*ntexpixel*sizeof(float));
        printf("Unique texture does exist\n");
    }
     */
    
    printf("AmbientLight=[%f,%f,%f]\n",ambientLight.x,ambientLight.y,ambientLight.z);
    
    int idebug=6717;
    
    for (i=0; i<ntexpixel; i++) {
        
        
        Idx[i]=-1;
        //	printf("start: %d\n",i);
        texx=(i / dmapheight); texy=(i % dmapheight);
        r.d.x=((float)texx)/dmapwidth; r.d.y=((float)texy)/dmapheight; r.d.z=1.f; // scale for texaccel now
        r.maxt=INFINITY; r.mint=0.0f;
        
        didIntersect=texAccel->IntersectQ(r,isect,barycentrics);
        cvSet2D(texMask, texx, texy, cvScalar(0.f));
        
        if (i==idebug) {
            printf("At i=%d\n",idebug);
            printf("texx=%d, texy=%d, r.d.x=%f, r.d.y=%f, didIntersect=%d\n", texx, texy, r.d.x, r.d.y, didIntersect ? 1 : 0);
        }
        

        
        if (!didIntersect) {
            bool didnbIntersect=false;
            for (j=0; j<4 && !didnbIntersect; j++) {
                r.d.x=((float)(texx+xdisp[j]))/dmapwidth; r.d.y=((float)(texy+ydisp[j]))/dmapheight; r.d.z=1.f; // scale for texaccel now
                r.maxt=INFINITY; r.mint=0.0f;
                didnbIntersect=texAccel->IntersectQ(r,isect,barycentrics);
            }
            didIntersect=didnbIntersect;
        }
        
        if (didIntersect) {
            // there is a texture point
            if (i==idebug) {
                printf("There is a texture point at i=%d\n",idebug);
            }
            shapeId=isect->shapeId-texIdOffset;
            vertex=barycentrics[0]*Vertices[Faces[3*shapeId]]+
            barycentrics[1]*Vertices[Faces[3*shapeId+1]]+
            barycentrics[2]*Vertices[Faces[3*shapeId+2]];
            vertexorig=barycentrics[0]*VerticesOriginal[Faces[3*shapeId]]+
            barycentrics[1]*VerticesOriginal[Faces[3*shapeId+1]]+
            barycentrics[2]*VerticesOriginal[Faces[3*shapeId+2]];
            
            pts3D[3*i]=vertexorig.x;
            pts3D[3*i+1]=vertexorig.y;
            pts3D[3*i+2]=vertexorig.z;
            
            
            faceIndex[i]=shapeId;
            if (fgroups) groupIndex[i]=fgroups[shapeId];
            
            vertext.x=vertex.x*o2w[0][0]+vertex.y*o2w[0][1]+vertex.z*o2w[0][2]+o2w[0][3];
            vertext.y=vertex.x*o2w[1][0]+vertex.y*o2w[1][1]+vertex.z*o2w[1][2]+o2w[1][3];
            vertext.z=vertex.x*o2w[2][0]+vertex.y*o2w[2][1]+vertex.z*o2w[2][2]+o2w[2][3];
            
            r.d.x=vertext.x; r.d.y=vertext.y; r.d.z=vertext.z; r.maxt=INFINITY; r.mint=.0f;
            didIntersect=bvhAccel->IntersectQ(r,isect,barycentricstest);
            norig=(barycentrics[0]*vertexNormalsOrig[Faces[3*shapeId]]+
                   barycentrics[1]*vertexNormalsOrig[Faces[3*shapeId+1]]+
                   barycentrics[2]*vertexNormalsOrig[Faces[3*shapeId+2]]);
            normals[i]=norig;
            
            
            if (didIntersect && shapeId != (isect->shapeId-shapeIdOffset)) {
                if (i==idebug) {
                    printf("There is a texture point at i=%d\n",idebug);
                }
                if ( (int)(texx % (hwinsize))==0 && (int)(texy % (hwinsize))==0 ) {
                    // for speeding up ANN, use pixels at one-half the resolution
                    present[i]=1;
                    Idx[i]=nunseen;
                    nunseen++;
                    nunseenuse++;
                } else {
                    Idx[i]=nunseen;
                    nunseen++;
                    present[i]=0;
                }
                bilerpedcolororig.x=0.f; bilerpedcolororig.y=0.f; bilerpedcolororig.z=0.f;
            } else {
                
                // there is a texture point, and teh shape point is seen
                
                // this is good
                
                // when you're in here, project the vertex back onto the camera
                // FOR NOW... obj2wld is identity, so just use intrinsic parameters
                // --- well it's not identity any longer, so account for it!
                x=fu*vertext.x/vertext.z+u0;
                y=fv*vertext.y/vertext.z+v0;
                Vector iorig, mylighto, myalb, myambalb;
                n=(barycentrics[0]*vertexNormals[Faces[3*shapeId]]+
                   barycentrics[1]*vertexNormals[Faces[3*shapeId+1]]+
                   barycentrics[2]*vertexNormals[Faces[3*shapeId+2]]);
                
                color=quadrature(vertext, n, r, bvhAccel, diffuseLight[0], cTheta, sTheta, cPhi, sPhi, SphereRadius, ndivs, groundAxis, false);
                
                bilinearInterp(x, y, height, I, NULL, &iorig);
                //if (existsunique) {
                //    myalb.x=Itexartist[3*i]; myalb.y=Itexartist[3*i+1]; myalb.z=Itexartist[3*i+2];
                //} else {
                if (mtlnums) {
                    getTexVal(Itexartist, mtlnums, shapeId, &myalb);
                } else {
                    quickNearestInterp(x, y, height, PixelSpaceAppearance, NULL, &myalb);
                }
                //}
                
                bilerpedcolororig=iorig-Emult(myalb, color)-Emult(myalb,ambientLight);
                
                if (*(cvGet2D(erodedImage, x, y).val)>.0f) {
                    // use erosion to claim that those pixels are not seen
                    if ( ((int)texx %  (hwinsize))==0 && ((int)texy %  (hwinsize))==0 ) {
                        // for speeding up ANN, use pixels at lower resolution
                        present[i]=3; Idxseen[nseen]=i; nseen++;
                    } else {
                        present[i]=2;
                    }
                    nseen++;
                    // Repopulate
                    if (!mtlnums) {
                        Itexartist[3*i]=myalb.x; Itexartist[3*i+1]=myalb.y; Itexartist[3*i+2]=myalb.z;
                    }
                    Itexdiff[3*i]=bilerpedcolororig.x; Itexdiff[3*i+1]=bilerpedcolororig.y; Itexdiff[3*i+2]=bilerpedcolororig.z;
                    cvSet2D(texMask, texx, texy, cvScalar(1.0f));
                } else {
                    if ( (int)(texx %  (hwinsize))==0 && (int)(texy %  (hwinsize))==0 ) {
                        // for speeding up ANN, use pixels at one-half the resolution
                        present[i]=1; Idx[i]=nunseen; nunseen++; nunseenuse++;
                    } else {
                        Idx[i]=nunseen; nunseen++; present[i]=0;
                    }
                    
                }
            }
            
        } else {
            // this is bad
            //
            // there is no texture point
            present[i]=-1;
            pts3D[3*i]=INFINITY;
            pts3D[3*i+1]=INFINITY;
            pts3D[3*i+2]=INFINITY;
            bilerpedcolororig.x=0.f; bilerpedcolororig.y=0.f; bilerpedcolororig.z=0.f;
            
        }
        if ( (i % 10000)==0) printf("Texture Res Part: %d\n",i);
    }
    
    //cvReleaseImage(&imsmooth);
    
    IplConvKernel* elem1=cvCreateStructuringElementEx(1,1,0,0,CV_SHAPE_RECT);
    cvErode(texMask, erodedTexMask, elem1, 1);
    cvReleaseStructuringElement(&elem1);
    IplConvKernel* elem2=cvCreateStructuringElementEx(1,1,0,0,CV_SHAPE_RECT);
    cvErode(texMask, erodedTexMask2, elem2, 1);
    cvReleaseStructuringElement(&elem2);
    
    
    for (i=0; i<ntexpixel; i++) {
        if (*(cvGet2D(erodedTexMask2, i / dmapheight, i % dmapheight).val)<.5)
        {
            if (present[i]==2) present[i]=0;
            if (present[i]==3) present[i]=1;
        }
    }
    
    
    //int* switches=new int[ntexpixel];
    int* choices=new int[ntexpixel];
    int* achoices=new int[ntexpixel];
    int* locs=new int[ntexpixel];
    
    writeFile(direc, present, sizeof(int)*ntexpixel, "present.dat");
    writeFile(direc, Itexartist, 3*sizeof(float)*ntexpixel, "artisttexturepregrown.dat");
    writeFile(direc, Itexdiff, 3*sizeof(float)*ntexpixel, "differencetexturepregrown.dat");
    //readFile(direc, Itexartist, 3*sizeof(float)*ntexpixel, "myartisttex.dat");
    //readFile(direc, Itexdiff, 3*sizeof(float)*ntexpixel, "mydifftex.dat");
    
    
    // THIS PART COMPUTES THE MRF USING SYMMETRIES
    RNG rng((uint)time(NULL));
    
    // change seen to unseen means change present, pts3D, faceIndex, Idx
    // principal symmetry
    // iterate over the unseen, and reflect them all to the seen
    
    int nusable=0;
    for (i=0; i<ntexpixel; i++) {
        if (present[i]>=0) nusable++;
    }
    int* idxusable=new int[nusable];
    int iusable=0;
    
    for (i=0; i<ntexpixel; i++) {
        if (present[i]>=0) {
            idxusable[iusable]=i; iusable++;
        }
    }
    
    float* pts3Dusable=new float[nusable*3];
    for (i=0; i<nusable; i++) {
        memcpy(&pts3Dusable[3*i], &pts3D[3*idxusable[i]], 3*sizeof(float));
    }
    
    int nnb=7;
    int* nearestidx=new int[nusable*(nnb+1)];
    float* distances=new float[nusable*(nnb+1)];
    
    anearestself(pts3Dusable, nusable, 3, nearestidx, distances, nnb+1);
    // repeated symmetry
    
    int nmax=100;
    //int nroundmax=nmax;
    for (i=0; i<ntexpixel; i++) { //switches[i]=0;
        choices[i]=-1;
        //locs[i]=-1;
        achoices[i]=-1; }
    
    int nmaxsyms=10;
    int nsyms=0;
    Vector* initsymn=new Vector[nmaxsyms]; float* initsymd=new float[nmaxsyms];
    bool initsymexists=false;
    
    path=direc; FILE* fidsymplane=fopen(path.append("symplane.dat").c_str(), "r");
    if (fidsymplane) {
        fread(&initsymn[0].x, sizeof(float), 1, fidsymplane);
        fread(&initsymn[0].y, sizeof(float), 1, fidsymplane);
        fread(&initsymn[0].z, sizeof(float), 1, fidsymplane);
        fread(&initsymd[0], sizeof(float), 1, fidsymplane);
        fclose(fidsymplane);
        nsyms=1;
        initsymexists=true;
    } else {
        initsymexists=false;
    }
    
    
    int nrounds=parameters.nrounds, nransac=parameters.nransac;
    int nmaxp1=nmax+1;
    int* locscopy=new int[ntexpixel*nmaxp1];
    int* indexes=new int[ntexpixel*nmaxp1];
    int* seenroundcopy;
    int iround=0, iransac=0;
    bool runrounds=true;
    
    int* queryidxs=new int[ntexpixel];
    int* dataidxs=new int[ntexpixel];
    int ndata=0; int nquery=0;
    Vector symn; Vector symt;
    float symd;
    Vector maxsymn; float maxsymd;
    Vector maxsymt;
    
    float scalefactor;
    PbrtPoint* bbox=new PbrtPoint[2];
    float threshold=parameters.ransacthresholdratio*computeBoundingBox(VerticesOriginal, nv, bbox);
    delete[] bbox;
    float thresholdsq=threshold*threshold;
    float normalthresh=parameters.ransacnormalthreshold;
    float apthresh=parameters.ransacappearancethreshold;
    
    printf("threshold=%f, apthresh=%f\n",threshold, apthresh);
    //int nransac2=2*nransac;
    //int imaxransac=0;
    int nummaxransac=-1;
    float dx, dy, dz;
    
    //int nfidxseen=0;
    //int nfidxunseen=0;
    
    int dim=3;
    int iseen=0, iunseen=0, ibar=0;
    double eps=0;
    Vector dd;
    
    ANNpoint			queryPt;				// query point
    ANNidx				nnIdx;					// near neighbor indices
    ANNdist				dist;					// near neighbor distances
    
    ANNidxArray				nnIdxs;					// near neighbor indices
    ANNdistArray		dists;					// near neighbor distances
    
    
    queryPt = annAllocPt(dim);
    
    nnIdxs=new ANNidx[100];
    dists=new ANNdist[100];
    
    bool issym=true;
    int numtransransac=0;
    int numsymransac=0;
    
    int* occurrencecount=new int[ntexpixel];
    int* occurrenceround=new int[ntexpixel];
    
    float multip;
    
    
    int nreflected=-1;
    int nsubreflected=-1;
    int nunseentotal=0;
    int mx=0;
    
    for (i=0; i<ntexpixel; i++) {
        if (present[i]==4) present[i]=2;
        if (present[i]==5) present[i]=3;
    }
    
    seenroundcopy=new int[ntexpixel];
    
    for (i=0; i<ntexpixel; i++) {
        indexes[i]=-1;
        occurrencecount[i]=-1;
        locscopy[i]=-1;
        //seencopy[i]=-1;
        if (present[i]==3 || present[i]==2) {
            occurrencecount[i]=1;
            locscopy[i]=1;
            //seencopy[i]=i;
            locscopy[i]=locs[i];
            
            // indexes in the first layer is the same as i
            indexes[i]=i;
        } else if (present[i]==0 || present[i]==1) {
            occurrencecount[i]=0;
            locscopy[i]=0;
            
            //seencopy[i]=0;
        }
        
    }
    
    /*
    float* layers=new float[3*ntexpixel*4];
    for (int i=0; i<3*ntexpixel*4; i++) {
        layers[i]=.0f;
    }
    for (int i=0; i<ntexpixel; i++) {
        if ( present[i]>=2 ){
            layers[3*i]=Itexartist[3*i]+Itexdiff[3*i];
            layers[3*i+1]=Itexartist[3*i+1]+Itexdiff[3*i+1];
            layers[3*i+2]=Itexartist[3*i+2]+Itexdiff[3*i+2];
        }
    }*/
    
    
    for (iround=0; iround<nrounds && runrounds; iround++) {
        ANNpointArray		dataPts;				// data points
        ANNkd_tree*			kdTree;					// search structure
        // use ransac to find the best reflection from unseen to seen if iround is not zero, and
        // for iround=0, use info from symplane.dat (if it exists)
        nquery=0; ndata=0;
        
        for (i=0; i<ntexpixel; i++) {
            occurrenceround[i]=0;
        }
        
        nunseentotal=0;
        for (i=0; i<ntexpixel; i++) {
            if (present[i]==1) { queryidxs[nquery]=i; nquery++; }
            if (present[i]==3) { dataidxs[ndata]=i; ndata++; }
            if (present[i]==1 || present[i]==0) { nunseentotal++; }
        }
        multip=1;
        
        if (nquery<=parameters.symstopnumber) {
            nquery=0;
            for (i=0; i<ntexpixel; i++) {
                if (present[i]==1 || present[i]==0) { queryidxs[nquery]=i; nquery++; }
            }
            multip=1;
        }
        
        
        runrounds=nquery>parameters.symstopnumber && nunseentotal>parameters.symstopunseennumber;
        
        if (runrounds) {
            
            // build the tree of seen data points
            dataPts = annAllocPts(ndata, dim);			// allocate data points
            //printf("ndata=%d/%d\n",ndata,nusable);
            for (i=0; i<ndata; i++) {
                for (j=0; j<dim; j++) {
                    if (std::isnan(pts3D[dataidxs[i]*dim+j])) {
                        printf("Oops, nan value in data at %d, %d: %f", i, j, pts3D[dataidxs[i]*dim+j]);
                    }
                    dataPts[i][j]=(ANNcoord)pts3D[dataidxs[i]*dim+j];
                }
            }
            kdTree=new ANNkd_tree(dataPts, ndata, dim);
            
            printf("Built tree\n");
            // RANSAC iterations...
            
            if (iround==0 && initsymexists) {
                if (iround<nsyms) {
                    maxsymn=Vector(initsymn[iround]);
                    maxsymd=initsymd[iround];
                } else {
                    maxsymn=Vector(initsymn[iround-(nrounds-nsyms)]);
                    maxsymd=initsymd[iround-(nrounds-nsyms)];
                }
                issym=true;
            } else {
                
                PbrtPoint vv;
                PbrtPoint ww;
                
                nummaxransac=0;
                
                printf("Testing, ndata=%d, nquery=%d\n", ndata, nquery);
                for (iransac=0; iransac<nransac; iransac++) {
                    iseen=rng.RandomUInt() % ndata;
                    iunseen=rng.RandomUInt() % nquery;
                    
                    numtransransac=0;
                    numsymransac=0;
                    
                    //vv=Vseen[iseen];
                        vv.x=pts3D[3*dataidxs[iseen]];
                        vv.y=pts3D[3*dataidxs[iseen]+1];
                        vv.z=pts3D[3*dataidxs[iseen]+2];
                    //ww=Vunseen[iunseen];
                    ww.x=pts3D[3*queryidxs[iunseen]];
                    ww.y=pts3D[3*queryidxs[iunseen]+1];
                    ww.z=pts3D[3*queryidxs[iunseen]+2];
                    symt=Vector(vv-ww);
                    
                    if (Dot(symt,symt) > 1e-6) {
                        
                        //for (i=0; i<ntexpixel; i++) distances[i]=-1;
                        // translational symmetry
                        /*
                        for (i=0; i<nquery; i++) {
                            ibar=3*queryidxs[i];
                            queryPt[0]=pts3D[ibar  ]+symt.x;
                            queryPt[1]=pts3D[ibar+1]+symt.y;
                            queryPt[2]=pts3D[ibar+2]+symt.z;
                            
                            if (std::isnan(queryPt[0]) || std::isnan(queryPt[1]) || std::isnan(queryPt[2])) {
                                printf("Woops, nan value in query at %d: %f,%f,%f", i, queryPt[0], queryPt[1], queryPt[2]);
                            }
                            
                            kdTree->annkSearch(queryPt, 1, &nnIdx, &dist, eps);
                            if (dist<thresholdsq*multip) {
                                //if (distances[dataidxs[nnIdx]]<-.5 || distances[dataidxs[nnIdx]]>dist) {
                                dx=Itexartist[ibar]-Itexartist[3*dataidxs[nnIdx]];
                                dy=Itexartist[ibar+1]-Itexartist[3*dataidxs[nnIdx]+1];
                                dz=Itexartist[ibar+2]-Itexartist[3*dataidxs[nnIdx]+2];

                         
                                if ( ( parameters.groups_instead_of_color &&
                                      (!fgroups || groupIndex[ibar/3]==groupIndex[dataidxs[nnIdx]]) ) ||
                                    (!parameters.groups_instead_of_color && dx*dx+dy*dy+dz*dz<apthresh) ) {
                                    if (Dot(normals[ibar/3], normals[dataidxs[nnIdx]])>normalthresh) {
                                        numtransransac++;
                                    }
                                }
                                numtransransac++;
                            }
                        }*/
                        
                        symn=Normalize(symt);
                        symd=-Dot(symn,.5*Vector(vv+ww));
                        
                        
                        //for (i=0; i<ntexpixel; i++) distances[i]=-1;
                        // reflective symmetry
                        for (i=0; i<nquery; i++) {
                            ibar=3*queryidxs[i];
                            scalefactor=-2*(pts3D[ibar]*symn.x+pts3D[ibar+1]*symn.y+pts3D[ibar+2]*symn.z+symd);
                            queryPt[0]=pts3D[ibar  ]+scalefactor*symn.x;
                            queryPt[1]=pts3D[ibar+1]+scalefactor*symn.y;
                            queryPt[2]=pts3D[ibar+2]+scalefactor*symn.z;
                            
                            kdTree->annkSearch(queryPt, 1, &nnIdx, &dist, eps);
                            if (dist<thresholdsq*multip) {
                                //if (distances[dataidxs[nnIdx]]<-.5 || distances[dataidxs[nnIdx]]>dist) {
                                dx=Itexartist[ibar]-Itexartist[3*dataidxs[nnIdx]];
                                dy=Itexartist[ibar+1]-Itexartist[3*dataidxs[nnIdx]+1];
                                dz=Itexartist[ibar+2]-Itexartist[3*dataidxs[nnIdx]+2];

                                /*
                                if ( (parameters.groups_instead_of_color &&
                                      (!fgroups || groupIndex[ibar/3]==groupIndex[dataidxs[nnIdx]]) ) ||
                                    (!parameters.groups_instead_of_color && dx*dx+dy*dy+dz*dz<apthresh) ) {
                                    if (Dot(normals[ibar/3], normals[dataidxs[nnIdx]])>normalthresh) {
                                        //distances[dataidxs[nnIdx]]=dist;
                                        numsymransac++;
                                    }
                                }*/
                                numsymransac++;
      
                            }
                            //}
                            
                        }
                        
                        if (numsymransac>=numtransransac && numsymransac>nummaxransac) {
                            nummaxransac=numsymransac;
                            //imaxransac=iransac;
                            issym=true;
                            maxsymd=symd;
                            maxsymn=Vector(symn);
                        } else if (numtransransac>numsymransac && numtransransac>nummaxransac) {
                            nummaxransac=numtransransac;
                            issym=false;
                            //imaxransac=iransac;
                            maxsymt=symt;
                        }
                        if ((iransac % 500) == 0) printf("Round=%d, ransac iteration=%d, iseen=%d, iunseen=%d, numtransransac=%d/%d, numsymransac=%d/%d, nummaxransac=%d/%d, issym=%d\n",
                                                         iround, iransac, iseen, iunseen, numtransransac, nquery, numsymransac, nquery, nummaxransac, nquery, issym);
                    }
                }
            }
            
            delete kdTree;
            ndata=0;
            for (i=0; i<ntexpixel; i++) {
                if (present[i]==2 || present[i]==3) {
                    // seen
                    dataidxs[ndata]=i; ndata++;
                }
            }
            dataPts = annAllocPts(ndata, 3);			// allocate data points
            printf("bigger, ndata=%d/%d, issym=%d\n",ndata,nusable,issym);
            for (i=0; i<ndata; i++) {
                for (j=0; j<3; j++) {
                    dataPts[i][j]=(ANNcoord)pts3D[dataidxs[i]*3+j];
                }
            }
            kdTree=new ANNkd_tree(dataPts, ndata, 3);
            
            mx=0;
            // Reflect on all to seen
            
            nreflected=0;
            nsubreflected=0;
            
            printf("Sym params: [%f,%f,%f], [%f,%f,%f], %f\n",maxsymt.x, maxsymt.y, maxsymt.z, maxsymn.x, maxsymn.y, maxsymn.z, maxsymd);
            //for (i=0; i<ntexpixel; i++) distances[i]=-1;
            // layer growth
            for (i=0; i<ntexpixel; i++) {
                if (present[i]>=0) {
                    ibar=3*i;
                    
                    if (issym) {
                        scalefactor=-2*(pts3D[ibar]*maxsymn.x+pts3D[ibar+1]*maxsymn.y+pts3D[ibar+2]*maxsymn.z+maxsymd);
                        queryPt[0]=pts3D[ibar  ]+scalefactor*maxsymn.x;
                        queryPt[1]=pts3D[ibar+1]+scalefactor*maxsymn.y;
                        queryPt[2]=pts3D[ibar+2]+scalefactor*maxsymn.z;
                    } else {
                        queryPt[0]=pts3D[ibar  ]+maxsymt.x;
                        queryPt[1]=pts3D[ibar+1]+maxsymt.y;
                        queryPt[2]=pts3D[ibar+2]+maxsymt.z;
                    }
                    
                    kdTree->annkSearch(queryPt, 1, &nnIdx, &dist, eps);
                    //printf("loc=%d, dist=%f\n", i, dist);
                    if (dist<thresholdsq) {
                        //if (distances[dataidxs[nnIdx]]<-.5 || distances[dataidxs[nnIdx]]>dist) {
                        dx=Itexartist[ibar]-Itexartist[3*dataidxs[nnIdx]];
                        dy=Itexartist[ibar+1]-Itexartist[3*dataidxs[nnIdx]+1];
                        dz=Itexartist[ibar+2]-Itexartist[3*dataidxs[nnIdx]+2];

                        //if ((iround==0 && initsymexists) || ( parameters.groups_instead_of_color && (!fgroups || (groupIndex[ibar/3]==groupIndex[dataidxs[nnIdx]]))) || ( !parameters.groups_instead_of_color && dx*dx+dy*dy+dz*dz<apthresh ) )  {
                        if (true) {
                            
                            int oc=occurrencecount[dataidxs[nnIdx]]<nmax-occurrencecount[i] ? occurrencecount[dataidxs[nnIdx]] : nmax-occurrencecount[i];
                            occurrenceround[i]=oc;
                            seenroundcopy[i]=dataidxs[nnIdx];
                            /*
                            int nlayers=(int)(pow(2,iround));
                            
                            if (iround<2) {
                            for (int kk=0; kk<nlayers; kk++) {
                                memcpy(&layers[3*(ntexpixel*(nlayers+kk)  +i)],
                                       &layers[3*(ntexpixel*kk+dataidxs[nnIdx])],
                                       3*sizeof(float));
                            }
                            }
                            */
                            if (present[i]==0) { present[i]=2; nreflected++; }// represents covered points
                            if (present[i]==1) { present[i]=3; nreflected++; nsubreflected++; } // represents covered points
                        }
                    }
                }
            }
            
            printf("Number of unseen that got reflected=%d (sub=%d)\n", nreflected, nsubreflected);
            
            
            
            for (i=0; i<ntexpixel; i++) {
                if (occurrenceround[i]>=1) {
                    for (j=0; j<occurrenceround[i]; j++) {
                        locscopy[(i+ntexpixel*(occurrencecount[i]+j))]=pow(2,iround)+locscopy[seenroundcopy[i]+ntexpixel*j];
                        indexes[(i+ntexpixel*(occurrencecount[i]+j))]=indexes[seenroundcopy[i]+ntexpixel*j];
                    }
                }
                occurrencecount[i]=occurrencecount[i]+occurrenceround[i];
                if (mx<occurrencecount[i]) mx=occurrencecount[i];
            }
            
            printf("max occurrence=%d\n",mx);
            delete kdTree;
        }
        
    }
    
    //writeFile(direc, layers, 3*ntexpixel*4*sizeof(float), "layers.dat");
    //delete[] layers;
    
    delete[] seenroundcopy;
    
    float* Itexdiffpp=new float[3*ntexpixel];
    memcpy(Itexdiffpp, Itexdiff, 3*ntexpixel*sizeof(float));
    
    // set up graph
    
    // THIS PART SOLVES THE MRF USING SYMMETRIES
    
    MRFEnergy<TypeGeneral>* mrf;
    MRFEnergy<TypeGeneral>::NodeId* nodes;
    MRFEnergy<TypeGeneral>::Options options;
    TypeGeneral::REAL energy, lowerBound;
    // number of nodes is number of points not infinity in pts3D
    const int nodeNum = nusable; // number of nodes
    int jbar=0, jj=0;
    //float integrity=1e-2;
    int sol=0;
    float dx1, dy1, dx2, dy2, dz1, dz2;
    int oc;
    int n_nodes=0;
    int n_edges=0;
    //float highpot=2.0f;
    float highpot=parameters.mrfbeta;
    bool addNode=true;
    int ni, nj, ibarc, jbarc;
    int ii=0;
    
#ifndef NO_DIFFERENCE_GROWTH
    mrf = new MRFEnergy<TypeGeneral>(TypeGeneral::GlobalSize(0));
    nodes = new MRFEnergy<TypeGeneral>::NodeId[nodeNum];
    
    printf("Creating Textures Graph...\n");
    //
    // iterate over nodes, i
    //     iterate over labels, k
    //         Unary potential[i,k] is zero
    for (i=0; i<nusable; i++) {
        oc=occurrencecount[idxusable[i]];
        oc=oc > 0 ? oc : 1;
        TypeGeneral::REAL Dnode[oc];
        
        for (j=0; j<oc; j++) {
            Dnode[j]=highpot;
        }
        
        ibar=idxusable[i];
        if (*(cvGet2D(erodedTexMask, ibar / dmapheight, ibar % dmapheight).val)>.5)
        {
            Dnode[0]=.0f;
        }
        
        nodes[i]=mrf->AddNode(TypeGeneral::LocalSize(oc), TypeGeneral::NodeData(Dnode));
        n_nodes++;
    }
    
    // iterate over nodes, i
    //    iterate over nearest neighbors of node, j
    //        Create PairwisePotentialMat_ij=new[nlabels*nlabels]; initialize with INFINITY
    //        iterate over labels,
    //              iterate over labels
    //                     PairwisePotentialMat_ij[k,l]=lambda if k==l,
    //                                                  ||texturen[i,k]-texturen[j,l]||^2 otherwise
    //		  addEdge between i and j with PairwisePotentialMat_ij
    
    for (i=0; i<nusable; i++) {
        ibar=idxusable[i];
        for (jj=0; jj<nnb+1; jj++) {
            j=nearestidx[i*(nnb+1)+(jj)];
            // if the node index j is leser than the node index i, there is a possibility that
            // j has seen i before as its nearest neighbor. Add only if this not the case
            addNode=i != j;
            if (j<i) {
                for (ii=0; ii<nnb+1 && addNode; ii++) {
                    addNode=i != nearestidx[j*(nnb+1)+ii];
                }
            }
            if (addNode) {
                jbar=idxusable[j];
                ni=occurrencecount[ibar]; nj=occurrencecount[jbar];
                
                n_edges++;
                
                if (ni==0) {
                    if (nj==0) {
                        TypeGeneral::REAL V[1];
                        V[0]=0;
                        mrf->AddEdge(nodes[i], nodes[j], TypeGeneral::EdgeData(TypeGeneral::GENERAL, V));
                    } else {
                        TypeGeneral::REAL V[nj];
                        for (jbarc=0; jbarc<nj; jbarc++) {
                            V[jbarc]=0;
                        }
                        mrf->AddEdge(nodes[i], nodes[j], TypeGeneral::EdgeData(TypeGeneral::GENERAL, V));
                    }
                } else if (nj==0) {
                    TypeGeneral::REAL V[ni];
                    for (ibarc=0; ibarc<ni; ibarc++) {
                        V[ibarc]=0;
                    }
                    mrf->AddEdge(nodes[i], nodes[j], TypeGeneral::EdgeData(TypeGeneral::GENERAL, V));
                } else {
                    TypeGeneral::REAL V[ni*nj];
                    
                    for (ibarc=0; ibarc<ni; ibarc++) {
                        for (jbarc=0; jbarc<nj; jbarc++) {
                            dx1= Itexdiff[3*indexes[(ibar+ntexpixel*ibarc)]];
                            dx2= Itexdiff[3*indexes[(jbar+ntexpixel*jbarc)]];
                            dy1= Itexdiff[3*indexes[(ibar+ntexpixel*ibarc)]+1];
                            dy2= Itexdiff[3*indexes[(jbar+ntexpixel*jbarc)]+1];
                            dz1= Itexdiff[3*indexes[(ibar+ntexpixel*ibarc)]+2];
                            dz2= Itexdiff[3*indexes[(jbar+ntexpixel*jbarc)]+2];
                            
                            dx=dx1-dx2; dy=dy1-dy2; dz=dz1-dz2;
                            
                             if (locscopy[ibar+ntexpixel*ibarc]==locscopy[jbar+ntexpixel*jbarc])
                                 V[ni*jbarc+ibarc]=parameters.mrfgamma*(dx*dx+dy*dy+dz*dz);
                             else
                                 V[ni*jbarc+ibarc]=(dx*dx+dy*dy+dz*dz);
                        }
                    }
                    
                    mrf->AddEdge(nodes[i], nodes[j], TypeGeneral::EdgeData(TypeGeneral::GENERAL, V));
                    
                    
                }
            }
        }
    }
    
    printf("Solving MRF\n");
    // solve the mrf!
    options.m_iterMax=8000; // maximum number of iterations
    options.m_printIter=50;
    options.m_printMinIter=5;
    options.m_eps=1e-6;
    printf("will stop when converges within %f\n", options.m_eps);
    
    mrf->Minimize_TRW_S(options, lowerBound, energy);
    for (i=0; i<nusable; i++) {
        sol=mrf->GetSolution(nodes[i]);
        choices[idxusable[i]]=sol;
        
            if (occurrencecount[idxusable[i]]>0) {
                memcpy(&Itexdiffpp[3*idxusable[i]], &Itexdiff[3*indexes[(idxusable[i]+ntexpixel*sol)]], 3*sizeof(float));
                //memcpy(&Itexdiffsm[3*idxusable[i]], &Itexdiffsm[3*indexes[(idxusable[i]+ntexpixel*sol)]], 3*sizeof(float));
            }
        
    }
    
    delete[] nodes;
    delete mrf;
#endif
    for (i=0; i<ntexpixel; i++) {
        if (present[i]>=0) {
            locs[i]=locscopy[(i+ntexpixel*choices[i])];
        }
    }
    
    
#ifndef NO_AT_GROWTH
    if (!mtlnums)
    {
        
        mrf = new MRFEnergy<TypeGeneral>(TypeGeneral::GlobalSize(0));
        nodes = new MRFEnergy<TypeGeneral>::NodeId[nodeNum];
        
        printf("Creating Textures Graph...\n");
        //
        // iterate over nodes, i
        //     iterate over labels, k
        //         Unary potential[i,k] is zero
        //int oc;
        for (i=0; i<nusable; i++) {
            oc=occurrencecount[idxusable[i]];
            oc=oc > 0 ? oc : 1;
            TypeGeneral::REAL Dnode[oc];
            for (j=0; j<oc; j++) {
                Dnode[j]=highpot;
            }
            ibar=idxusable[i];
            if (*(cvGet2D(erodedTexMask, ibar / dmapheight, ibar % dmapheight).val)>.5)
            {
                Dnode[0]=.0f;
            }
            
            nodes[i]=mrf->AddNode(TypeGeneral::LocalSize(oc), TypeGeneral::NodeData(Dnode));
        }
        
        // iterate over nodes, i
        //    iterate over nearest neighbors of node, j
        //        Create PairwisePotentialMat_ij=new[nlabels*nlabels]; initialize with INFINITY
        //        iterate over labels,
        //              iterate over labels
        //                     PairwisePotentialMat_ij[k,l]=lambda if k==l,
        //                                                  ||texturen[i,k]-texturen[j,l]||^2 otherwise
        //		  addEdge between i and j with PairwisePotentialMat_ij
        
        //int ni, nj, ibarc, jbarc;
        for (i=0; i<nusable; i++) {
            ibar=idxusable[i];
            for (jj=0; jj<nnb+1; jj++) {
                j=nearestidx[i*(nnb+1)+(jj)];
                // if the node index j is leser than the node index i, there is a possibility that
                // j has seen i before as its nearest neighbor. Add only if this not the case
                addNode=i != j;
                if (j<i) {
                    for (ii=0; ii<nnb+1 && addNode; ii++) {
                        addNode=i != nearestidx[j*(nnb+1)+ii];
                    }
                }
                if (addNode) {
                    jbar=idxusable[j];
                    ni=occurrencecount[ibar]; nj=occurrencecount[jbar];
                    
                    if (ni==0) {
                        if (nj==0) {
                            TypeGeneral::REAL V[1];
                            V[0]=0;
                            mrf->AddEdge(nodes[i], nodes[j], TypeGeneral::EdgeData(TypeGeneral::GENERAL, V));
                        } else {
                            TypeGeneral::REAL V[nj];
                            for (jbarc=0; jbarc<nj; jbarc++) {
                                V[jbarc]=0;
                            }
                            mrf->AddEdge(nodes[i], nodes[j], TypeGeneral::EdgeData(TypeGeneral::GENERAL, V));
                        }
                    } else if (nj==0) {
                        TypeGeneral::REAL V[ni];
                        for (ibarc=0; ibarc<ni; ibarc++) {
                            V[ibarc]=0;
                        }
                        mrf->AddEdge(nodes[i], nodes[j], TypeGeneral::EdgeData(TypeGeneral::GENERAL, V));
                    } else {
                        TypeGeneral::REAL V[ni*nj];
                        
                        for (ibarc=0; ibarc<ni; ibarc++) {
                            for (jbarc=0; jbarc<nj; jbarc++) {
                                dx=Itexartist[3*indexes[(ibar+ntexpixel*ibarc)]]-Itexartist[3*indexes[(jbar+ntexpixel*jbarc)]];
                                dy=Itexartist[3*indexes[(ibar+ntexpixel*ibarc)]+1]-Itexartist[3*indexes[(jbar+ntexpixel*jbarc)]+1];
                                dz=Itexartist[3*indexes[(ibar+ntexpixel*ibarc)]+2]-Itexartist[3*indexes[(jbar+ntexpixel*jbarc)]+2];
                                //V[ni*jbarc+ibarc]= locscopy[ibar+ntexpixel*ibarc]==locscopy[jbar+ntexpixel*jbarc] ? integrity : dx*dx+dy*dy+dz*dz;
                                V[ni*jbarc+ibarc]=dx*dx+dy*dy+dz*dz;
                            }
                        }
                        
                        //if (*(cvGet2D(erodedTexMask, ibar / dmapheight, ibar % dmapheight).val)>.5
                        //	&& *(cvGet2D(erodedTexMask, jbar/dmapheight, jbar % dmapheight).val)>.5) {
                        //	V[0]=.0f;
                        //}
                        mrf->AddEdge(nodes[i], nodes[j], TypeGeneral::EdgeData(TypeGeneral::GENERAL, V));
                    }
                }
            }
        }
        
        printf("Solving MRF...\n");
        // solve the mrf!
        options.m_iterMax =40000; // maximum number of iterations
        options.m_printIter=50;
        options.m_printMinIter=5;
        options.m_eps=0;
        
        mrf->Minimize_TRW_S(options, lowerBound, energy);
        
        for (i=0; i<nusable; i++) {
            sol=mrf->GetSolution(nodes[i]);
            achoices[idxusable[i]]=sol;
            if (occurrencecount[idxusable[i]]>0) {
                memcpy(&Itexartist[3*idxusable[i]], &Itexartist[3*indexes[(idxusable[i]+ntexpixel*sol)]], 3*sizeof(float));
            }
        }
        delete[] nodes;
        delete mrf;
        
    }
#endif
    memcpy(Itexdiff, Itexdiffpp, 3*sizeof(float)*ntexpixel);
    
    writeFile(direc, Itexartist, 3*sizeof(float)*ntexpixel, "artisttexturesymgrown.dat");
    writeFile(direc, Itexdiff, 3*sizeof(float)*ntexpixel, "differencetexturesymgrown.dat");
    
    
    
    delete[] locscopy;
    delete[] indexes;
    
    delete[] queryidxs;
    delete[] dataidxs;;
    delete[] nnIdxs;
    delete[] dists;
    delete[] occurrencecount;
    delete[] occurrenceround;
    //delete[] distances;
    
    if (initsymn) delete[] initsymn;
    if (initsymd) delete[] initsymd;
    
    
    
    
	
	
	
	cvReleaseImage(&erodedTexMask);
    cvReleaseImage(&texMask);
	
    delete[] pts3D;
    delete[] pts3Dusable;
    delete[] present;
    //delete[] Itexdiffsm;
    //if (Idx) delete[] Idx;
    if (faceIndex) delete[] faceIndex;
    if (groupIndex) delete[] groupIndex;
    if (normals) delete[] normals;
    //if (switches) delete[] switches;
    if (choices) delete[] choices;
    if (achoices) delete[] achoices;
    if (locs) delete[] locs;
    if (idxusable) delete[] idxusable;
    if (nearestidx) delete[] nearestidx;
	delete[] Idx;
	if (Idxseen) delete[] Idxseen;
    
    delete isect;
	delete lisect;
    
	//exit(0);
    
}

void estimateIllumination(BVHAccel* bvhAccel, Vector* vertexNormals, int* Faces,
                          float* I,	bool* Igroundmask, bool* Ishadowmask, // image data structure
                          float* PixelSpaceAppearance, IplImage* binaryImage,IplImage* erodedImage,
                          float u0, float v0, float ifu, float ifv, float* groundAxis,// camera parameters
                          float SphereRadius, float* Ktranspose, bool* B,// light sphere parameters
                          float* cTheta, float* sTheta, float* cPhi, float* sPhi,
                          int* mtlnums, Vector& albedo, Vector& groundAlbedo, Vector &ambientGroundAlbedo,
                          param& parameters,
                          int width, int height, int nkernels, int ndivs, int shapeIdOffset, int dmapwidth, int dmapheight, // extra parameters
                          string path, string direc, int iternum, float& lightLambdaError,
                          Vector** diffuseLight, Vector& ambientLight, int lightType) {	// output
	
	// Declarations
	int i, j, k, l, count, shapeId;
	bool didIntersect, didLightIntersect;
	float xpix, ypix, lambda, stheta, ctheta, ndots;
	float barycentrics[3];
	bool aboveground=true;
	bool use=true;
    int npixels=width*height;
	
	//bool shouldEstimateAmbient=ambientLight.x<0 && ambientLight.y<0 && ambientLight.z<0; // set these to -1 if you want to force an estimation
    //    bool shouldEstimateAmbient=false;
    bool shouldEstimateAmbient=parameters.estimateAmbient;
	
	int nch=lightType==TYPE_COLOR ? 3 : 1;
	
	int nskernels=nkernels;
	int nlight=nskernels+(shouldEstimateAmbient ? 1 : 0); // ambient + diffuse
	double* H=new double[(nlight*(nlight+1))/2];
	double* Hr=new double[(nlight*(nlight+1))/2];
	double* Hg=new double[(nlight*(nlight+1))/2];
	double* Hb=new double[(nlight*(nlight+1))/2];
	double* Hopt=new double[(nlight*(nlight+1))/2];
	
	int* hsubk=new int[(nlight*(nlight+1))/2];
	int* hsubl=new int[(nlight*(nlight+1))/2];
	double* f=new double[nch*nlight];
	double* fopt=new double[nch*nlight];
	double* fr=new double[nlight];
	double* fg=new double[nlight];
	double* fb=new double[nlight];
	
	
	float shadowweighti=1.0f;
	
	count=0;
	for (k=0; k<nlight; k++) {
		for (l=0; l<=k; l++) {
			hsubk[count]=k;
			hsubl[count]=l;
			count++;
		}
	}
	
	Intersection* isect=new Intersection();
	Intersection* lisect=new Intersection();
	
	/*FILE* ftransfer, *falbedo, *flight;
	 path=direc; path.append("Transfer.dat"); ftransfer=fopen(path.c_str(), "w");
	 path=direc; path.append("AlbedoPixels.txt"); falbedo=fopen(path.c_str(), "w");*/
	
	Vector direction(.0,.0,1.0), lightDirection, color, endcolor, n, nsym, reflection, bilerpedspecalb, ld;
	Vector view;
	PbrtPoint vdtex;
	
	PbrtPoint origin(.0,.0,.0);
	PbrtPoint ipsym, iporig, intersectionPt;
	Ray r(origin,direction,0,INFINITY);
	Ray s(intersectionPt,lightDirection,0,INFINITY);
	// taxi specific
	path=direc;
	FILE* fid;
    
	int nmax=(int)ceilf(npixels/9);
	printf("nmax=%d\n", nmax);
    
	float* BigDiffuseTR=new float[nmax*ndivs];
	float* BigDiffuseTG=new float[nmax*ndivs];
	float* BigDiffuseTB=new float[nmax*ndivs];
	int* pixelIndex=new int[nmax];
	bool* didIntersects=new bool[nmax];
	float* shadowweights=new float[nmax];
	int* shapeIds=new int[nmax];
	float* albedos=new float[3*nmax];
	float* ambalbedos=new float[3*nmax];
	float* pixels=new float[3*nmax];
    float* reds=new float[nmax];
    float* greens=new float[nmax];
    float* blues=new float[nmax];
    float* ralbs=new float[nmax];
    float* galbs=new float[nmax];
    float* balbs=new float[nmax];
    float a1, a2, a3;
    
    printf("end up here\n");
	
	
	int herecount=0;
	int npixelsused=0;
	
	
    printf("and then here\n");
    
    float ambientend=.0f;
    float ambientend2=.0f;
    
	for (i=0; i<npixels; i++) {
		// shoot a ray from the camera center through the pixel (i.e. [x2d,y2d,1])
		ypix=((i % height));
		xpix=((i / height));
		
		r.d.x=(xpix-u0)*ifu;
		r.d.y=(ypix-v0)*ifv;
		r.maxt=INFINITY;
		r.mint=0;
		didIntersect=bvhAccel->IntersectQ(r,isect,barycentrics);
		
		if ((int)ypix % parameters.nstepillumination==0 && (int)xpix % parameters.nstepillumination==0)
        {
            
			// proceed either if it intersected or if it did not intersect and the mask value of ground is high
			if (didIntersect || (!didIntersect && Igroundmask[i])) {
				
				aboveground=true;
				if (!didIntersect) {
					// if the ray does not intersect with the object, it belongs to the
					// ground
					lambda=-groundAxis[3]*1.0f/(groundAxis[0]*r.d.x+groundAxis[1]*r.d.y+groundAxis[2]);
					intersectionPt.x=lambda*r.d.x; intersectionPt.y=lambda*r.d.y; intersectionPt.z=lambda;
					n=Vector(groundAxis[0],groundAxis[1],groundAxis[2]);
					shadowweighti=(Ishadowmask[i] ? 1.0f : parameters.shadowweight); // this is the other way round
					//shadowweighti=1.0f;
					
					if (shadowweighti>100 && herecount==0) {
						printf("here with the light\n");
						herecount++;
					}
					// for future comptuations, use all shadow pixels, but use only every 2nd ground pixel in x- and y- directions
					use=Ishadowmask[i] || ( ((int)ypix)%parameters.nstepgroundillumination==0 &&
                                                  ((int)xpix)%parameters.nstepgroundillumination==0 );
				} else {
					intersectionPt=r.o+r.d*r.maxt;
					shapeId=isect->shapeId-shapeIdOffset;
					n=Normalize(barycentrics[0]*vertexNormals[Faces[3*shapeId]]+
								barycentrics[1]*vertexNormals[Faces[3*shapeId+1]]+
								barycentrics[2]*vertexNormals[Faces[3*shapeId+2]]);
					
                    shadowweighti=1.0f;
                    use=true;
				}
				
				s.o=intersectionPt;
				// for each pixel in 3D
				//    for each light source
				view=-Normalize(Vector(intersectionPt));
				
				if (use) {
                    didIntersects[npixelsused]=didIntersect;
					if (didIntersect) {
						shapeIds[npixelsused]=shapeId;
						
                        
                        memcpy(&albedos[3*npixelsused],&PixelSpaceAppearance[3*i],3*sizeof(float));
                        memcpy(&ambalbedos[3*npixelsused],&PixelSpaceAppearance[3*i],3*sizeof(float));
                        a1=PixelSpaceAppearance[3*i]; a2=PixelSpaceAppearance[3*i+1]; a3=PixelSpaceAppearance[3*i+2];
                        //memcpy(&alb,&PixelSpaceAppearance[3*i],3*sizeof(float));
					} else {
                        //alb[0]=groundAlbedo.x; alb[1]=groundAlbedo.y; alb[2]=groundAlbedo.z;
						shapeIds[npixelsused]=0;
						albedos[3*npixelsused]=groundAlbedo.x; albedos[3*npixelsused+1]=groundAlbedo.y; albedos[3*npixelsused+2]=groundAlbedo.z;
						ambalbedos[3*npixelsused]=ambientGroundAlbedo.x; ambalbedos[3*npixelsused+1]=ambientGroundAlbedo.y; ambalbedos[3*npixelsused+2]=ambientGroundAlbedo.z;
                        a1=groundAlbedo.x; a2=groundAlbedo.y; a3=groundAlbedo.z;
					}
					
					for (j=0; j<ndivs; j++) {
						//         shoot a ray
						stheta=sTheta[j];
						ctheta=cTheta[j];
						
						
						// Distant light sources
#ifdef DISTANT
						s.d.x=stheta*(cPhi[j]);
						s.d.y=stheta*(sPhi[j]);
						s.d.z=ctheta;
#else
						// nearby light sources
						ld.x=SphereRadius*stheta*cPhi[j]; ld.y=SphereRadius*stheta*sPhi[j]; ld.z=SphereRadius*ctheta; // alternative format where lights are closer
                        s.d.x=ld.x-intersectionPt.x;
                        s.d.y=ld.y-intersectionPt.y;
                        s.d.z=ld.z-intersectionPt.z;
                        s.d=Normalize(s.d);
#endif
						
						s.maxt=INFINITY;
						s.mint=0;
						s.o=intersectionPt+s.d*(MIN_LIGHT_DIRECTION_ALPHA);
						
						// first check if the dot product is greater than zero (light source is above primitive
						ndots=Dot(n,s.d);
						BigDiffuseTR[npixelsused*ndivs+j]=.0f;
						BigDiffuseTG[npixelsused*ndivs+j]=.0f;
						BigDiffuseTB[npixelsused*ndivs+j]=.0f;
						//if (ndots>0) {
                        if ( didIntersect || (!didIntersect && ndots>0)) {
							// then check if light source is above ground
							
							/*if (didIntersect) {
							 denom=groundAxis[0]*s.d.x+groundAxis[1]*s.d.y+groundAxis[2]*s.d.z;
							 if (fabs(denom)>1e-6) {
							 lambda=-(groundAxis[3]+
							 groundAxis[0]*intersectionPt.x+
							 groundAxis[1]*intersectionPt.y+
							 groundAxis[2]*intersectionPt.z)*1.0f/denom;
							 aboveground=lambda<1e-3;
							 } else aboveground=true;
							 
							 } else aboveground=true;*/
							// only if light source is above ground and above primitive should you
							// consider using it (this check should be done for object pixels not ground pixels)
							//aboveground=ld.x*groundAxis[0]+ld.y*groundAxis[1]+ld.z*groundAxis[2]+groundAxis[3]>0;
							aboveground=true;
							if (aboveground) {
								didLightIntersect=bvhAccel->IntersectP(s);
								if (!didLightIntersect) {
									//         if it does not intersect,
									//				compute the diffuse contributions --- this is something you'll have to adapt
									//              add the contribution to the running sum for the light seen at the pixel
									BigDiffuseTR[npixelsused*ndivs+j]=a1*fabsf(ndots)*sqrtf(shadowweighti);
									BigDiffuseTG[npixelsused*ndivs+j]=a2*fabsf(ndots)*sqrtf(shadowweighti);
									BigDiffuseTB[npixelsused*ndivs+j]=a3*fabsf(ndots)*sqrtf(shadowweighti);
                                }
							}
						}
						
						
						//         else
						//              do nothing
						//         multiply with the reflectances
						//     add the texture correctly interpolated with the appropriate barycentric coordinates (
					}
                    memcpy(&pixels[3*npixelsused], &I[3*i], 3*sizeof(float));
                    
                    //reds[npixelsused]=(I[3*i]*shadowweighti/a1+I[3*i+1]*shadowweighti/a2+I[3*i+2]*shadowweighti/a3);
                    //greens[npixelsused]=I[3*i+1]*shadowweighti/a2;
                    //blues[npixelsused]=I[3*i+2]*shadowweighti/a3;
                    reds[npixelsused]=I[3*i]*sqrtf(shadowweighti);
                    greens[npixelsused]=I[3*i+1]*sqrtf(shadowweighti);
                    blues[npixelsused]=I[3*i+2]*sqrtf(shadowweighti);
                    ralbs[npixelsused]=a1*sqrtf(shadowweighti);
                    galbs[npixelsused]=a2*sqrtf(shadowweighti);
                    balbs[npixelsused]=a3*sqrtf(shadowweighti);
                    
                    
                    ambientend+=(ralbs[npixelsused]*ralbs[npixelsused])+(balbs[npixelsused]*balbs[npixelsused])+
                    (galbs[npixelsused]*galbs[npixelsused]);
                    ambientend2+=(ralbs[npixelsused]*reds[npixelsused])+(balbs[npixelsused]*blues[npixelsused])+(galbs[npixelsused]*greens[npixelsused]);
					shadowweights[npixelsused]=shadowweighti;
					pixelIndex[npixelsused]=i;
					npixelsused++;
				}
			}
            
        }
        if ((i % 10000)==0) printf("%d, %d/%d\n",i, npixelsused, nmax);
        
    }
	
    // Complete the matrix to the point that it is a multiple of four (append zeros to the end)
    int npixelsmax=(npixelsused/4)*4;
    for (i=npixelsused; i<npixelsmax; i++) {
        reds[3*npixelsused]=.0f; blues[3*npixelsused]=.0f; greens[3*npixelsused]=.0f;
        for (j=0; j<ndivs; j++) {
            BigDiffuseTR[npixelsused*ndivs+j]=.0f;
            BigDiffuseTG[npixelsused*ndivs+j]=.0f;
            BigDiffuseTB[npixelsused*ndivs+j]=.0f;
        }
        npixelsused++;
    }
	
    vFloat* vBigDiffuseTR=(vFloat*)BigDiffuseTR;
    vFloat* vBigDiffuseTG=(vFloat*)BigDiffuseTG;
    vFloat* vBigDiffuseTB=(vFloat*)BigDiffuseTB;
    float* Hout=new float[ndivs*ndivs];
    float* Houtsupport=new float[ndivs*ndivs];
    float* fout=new float[ndivs];
    float* foutsupport=new float[ndivs];
    
    vFloat* vHout=(vFloat*)Hout;
    vFloat* vHoutsupport=(vFloat*)Houtsupport;
    vFloat* vfout=(vFloat*)fout;
    vFloat* vfoutsupport=(vFloat*)foutsupport;
    vFloat* vKtranspose=(vFloat*)Ktranspose;
    
    
    vSgemul(ndivs, npixelsused, ndivs, vBigDiffuseTR, 'T', vBigDiffuseTR, 'N', vHout);
    vSgemm(ndivs, npixelsused, ndivs, vBigDiffuseTG, 'T', vBigDiffuseTG, 'N', vHout, 1.0f, 1.0f, vHout);
    vSgemm(ndivs, npixelsused, ndivs, vBigDiffuseTB, 'T', vBigDiffuseTB, 'N', vHout, 1.0f, 1.0f, vHout);
    
    
//    writeFile(direc, Hout, sizeof(float)*ndivs*ndivs, "myH1.dat");
    vSgemul(ndivs, ndivs, ndivs, vKtranspose, 'N', vHout, 'N', vHoutsupport);
//    writeFile(direc, Houtsupport, sizeof(float)*ndivs*ndivs, "myHsupport.dat");
    vSgemul(ndivs, ndivs, ndivs, vHoutsupport, 'N', vKtranspose, 'T', vHout);
//    writeFile(direc, Hout, sizeof(float)*ndivs*ndivs, "myH2.dat");

    for (int i=0; i<ndivs; i++) {
        Hout[i+ndivs*i]+=parameters.lambda2;
        fout[i]=parameters.lambda1;
        foutsupport[i]=.0f;
    }
//    writeFile(direc, Hout, sizeof(float)*ndivs*ndivs, "myH.dat");
    
    vFloat* vreds=(vFloat*)reds;
    vFloat* vblues=(vFloat*)blues;
    vFloat* vgreens=(vFloat*)greens;
    
    vSgemtx(npixelsused, ndivs, -1.0f, vBigDiffuseTR, vreds, vfoutsupport);
    vSgemtx(npixelsused, ndivs, -1.0f, vBigDiffuseTG, vgreens, vfoutsupport);
    vSgemtx(npixelsused, ndivs, -1.0f, vBigDiffuseTB, vblues, vfoutsupport);
    
    vSgemx(ndivs, ndivs, 1.0f, vKtranspose, vfoutsupport, vfout);
    
    float* hout=new float[ndivs];
    float* houtsupport=new float[ndivs];
    
    for (int i=0; i<ndivs; i++) {
        hout[i]=.0f;
        houtsupport[i]=.0f;
    }
    vFloat* vhout=(vFloat*)hout;
    vFloat* vhoutsupport=(vFloat*)houtsupport;
    vFloat* vralbs=(vFloat*)ralbs;
    vFloat* vgalbs=(vFloat*)galbs;
    vFloat* vbalbs=(vFloat*)balbs;
    
    vSgemtx(npixelsused, ndivs, 1.0f, vBigDiffuseTR, vralbs, vhoutsupport);
    vSgemtx(npixelsused, ndivs, 1.0f, vBigDiffuseTG, vgalbs, vhoutsupport);
    vSgemtx(npixelsused, ndivs, 1.0f, vBigDiffuseTB, vbalbs, vhoutsupport);
    
    vSgemx(ndivs, ndivs, 1.0f, vKtranspose, vhoutsupport, vhout);
    
    //vSgemtx(npixelsused, ndivs, -1.0f/3, vBigDiffuseT, vreds, vfout);
    //vSgemtx(npixelsused, ndivs, -1.0f/3, vBigDiffuseT, vgreens, vfout);
    //vSgemtx(npixelsused, ndivs, -1.0f/3, vBigDiffuseT, vblues, vfout);
    
    count=0;
    for (int k=0; k<nlight; k++) {
        for (int l=0; l<=k; l++) {
            if (k<ndivs) {
                Hopt[count]=Hout[ndivs*k+l];
            } else {
                if (l<ndivs) {
                    Hopt[count]=hout[l];
                } else {
                    Hopt[count]=ambientend;
                }
            }
            count++;
        }
        if (k<ndivs) {
            fopt[k]=fout[k];
        } else {
            fopt[k]=-ambientend2;
        }
    }
    
    writeFile(direc, Hopt, sizeof(double)*nlight*(nlight+1)/2, "myHopt.dat");
    writeFile(direc, Hout, sizeof(float)*ndivs*ndivs, "myHout.dat");
    writeFile(direc, hout, sizeof(float)*ndivs, "myh.dat");
    writeFile(direc, &ambientend, sizeof(float)*1, "myambientend.dat");
    delete[] Hout;
    delete[] Houtsupport;
    delete[] fout;
    delete[] foutsupport;
    delete[] hout;
    delete[] houtsupport;
    
    
	printf("npixelsused=%d\n, capacity=%d\n", npixelsused, nmax);
	path=direc;
	fid=fopen(path.append("BigDiffuseT.dat").c_str(), "w"); fwrite(BigDiffuseTR, npixelsused*ndivs*sizeof(float), 1, fid); fclose(fid);
	path=direc;
	fid=fopen(path.append("shadowweights.dat").c_str(), "w"); fwrite(shadowweights, npixelsused*sizeof(float), 1, fid); fclose(fid);
	path=direc;
	fid=fopen(path.append("albedos.dat").c_str(), "w"); fwrite(albedos, npixelsused*3*sizeof(float), 1, fid); fclose(fid);
	path=direc;
	fid=fopen(path.append("ambalbedos.dat").c_str(), "w"); fwrite(ambalbedos, npixelsused*3*sizeof(float), 1, fid); fclose(fid);
	path=direc;
	fid=fopen(path.append("pixels.dat").c_str(), "w"); fwrite(pixels, npixelsused*3*sizeof(float), 1, fid); fclose(fid);
	path=direc;
	fid=fopen(path.append("pixelIndex.dat").c_str(), "w"); fwrite(pixels, npixelsused*sizeof(int), 1, fid); fclose(fid);
	
	
	path=direc; fid=fopen(path.append("hsubk.dat").c_str(), "w");
	fwrite(hsubk, nlight*(nlight+1)/2, sizeof(int), fid); fclose(fid);
	path=direc; fid=fopen(path.append("hsubl.dat").c_str(), "w");
	fwrite(hsubl, nlight*(nlight+1)/2, sizeof(int), fid); fclose(fid);
	
	delete[] BigDiffuseTR;
	delete[] BigDiffuseTG;
	delete[] BigDiffuseTB;
	delete[] pixelIndex;
	delete[] pixels;
	delete[] reds;
    delete[] greens;
    delete[] blues;
    delete[] ralbs; delete[] galbs; delete[] balbs;
    
	delete[] ambalbedos;
	delete[] albedos;
	delete[] didIntersects;
	delete[] shapeIds;
	delete[] shadowweights;
    
    /*
	
	Engine* workengine=engOpen("matlab -nosplash -nojvm -nodesktop");
	
	mxArray* mxName=mxCreateStringFromNChars_730(direc.c_str(), direc.length());
	mxArray* mxHasSym=mxCreateDoubleMatrix(1,1,mxREAL);
	mxArray* mxNdivs=mxCreateDoubleMatrix(1,1,mxREAL);
	mxArray* mxLambda1=mxCreateDoubleMatrix(1,1,mxREAL);
	mxArray* mxLambda2=mxCreateDoubleMatrix(1,1,mxREAL);
	mxArray* mxAmbientLight=mxCreateDoubleMatrix(1,3,mxREAL);
    double *prNdivs=mxGetPr(mxNdivs);
	double* prLambda1=mxGetPr(mxLambda1), *prLambda2=mxGetPr(mxLambda2);
	double* prHasSym=mxGetPr(mxHasSym);
	double* prAmbientLight=mxGetPr(mxAmbientLight);
	*prHasSym= .0;
	*prNdivs=ndivs;
	*prLambda1=parameters.lambda1;
	*prLambda2=parameters.lambda2;
	prAmbientLight[0]=ambientLight.x; prAmbientLight[1]=ambientLight.y; prAmbientLight[2]=ambientLight.z;
	engPutVariable(workengine, "name", mxName);
	engPutVariable(workengine, "ndivs", mxNdivs);
	engPutVariable(workengine, "lambda1", mxLambda1);
	engPutVariable(workengine, "lambda2", mxLambda2);
	engPutVariable(workengine, "hasSym", mxHasSym);
	engPutVariable(workengine, "ambientLight", mxAmbientLight);
    
    // Initialization
    
    if (lightType==TYPE_GRAYSCALE) {
        if (shouldEstimateAmbient) {
            engEvalString(workengine, "createHfAmbient; clearvars -except name ndivs lambda1 lambda2");
        } else {
            engEvalString(workengine, "createHf; clearvars -except name ndivs lambda1 lambda2");
        }
    } else {
        if (shouldEstimateAmbient) {
            engEvalString(workengine, "createHfColorAmbient; clearvars -except name ndivs lambda1 lambda2");
        } else {
            engEvalString(workengine, "createHfColor; clearvars -except name ndivs lambda1 lambda2");
        }
    }
    
    */
    int niter=lightType==TYPE_GRAYSCALE ? 1 : 1; ///
    
     
    lightLambdaError=.0f;
    
    double xx[nlight];
    
    /*
        if (lightType==TYPE_GRAYSCALE) {
            path=direc; fid=fopen(path.append("H.dat").c_str(), "r");
            fread(Hopt, nlight*(nlight+1)/2, sizeof(double), fid); fclose(fid);
            path=direc; fid=fopen(path.append("f.dat").c_str(),"r");
            fread(fopt, nlight, sizeof(double), fid);
        } else {
            path=direc; fid=fopen(path.append("Hr.dat").c_str(), "r");
            fread(Hr, nlight*(nlight+1)/2, sizeof(double), fid); fclose(fid);
            path=direc; fid=fopen(path.append("Hg.dat").c_str(), "r");
            fread(Hg, nlight*(nlight+1)/2, sizeof(double), fid); fclose(fid);
            path=direc; fid=fopen(path.append("Hb.dat").c_str(), "r");
            fread(Hb, nlight*(nlight+1)/2, sizeof(double), fid); fclose(fid);
            path=direc; fid=fopen(path.append("f.dat").c_str(),"r");
            fread(fopt, 3*nlight, sizeof(double), fid);
        }
     */
        
        MSKenv_t env=NULL;
        MSKtask_t task=NULL;
        MSKrescodee res;
        res=MSK_makeenv(&env,NULL);
        if (res==MSK_RES_OK)
            res=MSK_initenv(env);
        else
            printf("Could not make env\n");
        res=MSK_maketask(env, 0, nlight, &task); // 1 constraint, nkernels/ndivs variables + 1 for ambient
        if (res==MSK_RES_OK)
            res=MSK_putmaxnumvar(task, nlight);
        else
            printf("Could not make task\n");
        if (res==MSK_RES_OK)
            res=MSK_putmaxnumcon(task, 0);
        else
            printf("Could not put max vars\n");
        if (res==MSK_RES_OK)
            res=MSK_appendcons(task,0);
        else
            printf("Could not put max con\n");
        if (res==MSK_RES_OK)
            res=MSK_appendvars(task,nlight);
        else
            printf("Could not put number of constraints\n");
        
        if (res!=MSK_RES_OK)
            printf("Could not put number of variables\n");
        
        for (int d=0; d<nch; d++) { // repeat over 3 channels
            for (k=0; k<nlight && res==MSK_RES_OK; k++) {
                if (res==MSK_RES_OK)
                    res=MSK_putcj(task, k, fopt[nch*k+d]);
                if (res==MSK_RES_OK)
                    res=MSK_putbound(task, MSK_ACC_VAR, k, MSK_BK_LO, .0, +MSK_INFINITY);
            }
            
            if (res==MSK_RES_OK)
                if (lightType==TYPE_GRAYSCALE)
                    res=MSK_putqobj(task, (nlight*(nlight+1))/2, hsubk, hsubl, Hopt);
                else {
                    if (d==0) res=MSK_putqobj(task,(nlight*(nlight+1))/2, hsubk, hsubl, Hr);
                    if (d==1) res=MSK_putqobj(task,(nlight*(nlight+1))/2, hsubk, hsubl, Hg);
                    if (d==2) res=MSK_putqobj(task,(nlight*(nlight+1))/2, hsubk, hsubl, Hb);
                }
                else {
                    printf("Sorry, could not put bounds");
                }
            
            
            if (res==MSK_RES_OK) {
                MSKrescodee trmcode;
                res=MSK_optimizetrm(task, &trmcode);
                if (res==MSK_RES_OK)
                {
                    MSKsolstae solsta;
                    MSK_getsolsta(task, MSK_SOL_ITR, &solsta);
                    switch (solsta) {
                        case MSK_SOL_STA_OPTIMAL:
                        case MSK_SOL_STA_NEAR_OPTIMAL:
                            MSK_getsolutionslice(task, MSK_SOL_ITR, MSK_SOL_ITEM_XX, 0, nlight, xx);
                            for (k=0; k<nkernels; k++) {
                                lightLambdaError+=parameters.lambda1*(float)(fabs(xx[k]))+parameters.lambda2*(float)(xx[k]*xx[k]);
                            }
                            
                            
                            for (j=0; j<ndivs; j++) {
                                if (lightType==TYPE_GRAYSCALE) {
                                    diffuseLight[0][j].x=0; diffuseLight[0][j].y=0; diffuseLight[0][j].z=0;
                                    for (k=0; k<nkernels; k++) {
                                        diffuseLight[0][j].x+=Ktranspose[k*nkernels+j]*xx[k];
                                        diffuseLight[0][j].y+=Ktranspose[k*nkernels+j]*xx[k];
                                        diffuseLight[0][j].z+=Ktranspose[k*nkernels+j]*xx[k];
                                    }
                                } else {
                                    if (d==0) {diffuseLight[0][j].x=0; }
                                    else if (d==1) {diffuseLight[0][j].y=0; }
                                    else if (d==2) {diffuseLight[0][j].z=0; }
                                    for (k=0; k<nkernels; k++) {
                                        if (d==0) diffuseLight[0][j].x+=Ktranspose[k*nkernels+j]*xx[k];
                                        else if (d==1) diffuseLight[0][j].y+=Ktranspose[k*nkernels+j]*xx[k];
                                        else if (d==2) diffuseLight[0][j].z+=Ktranspose[k*nkernels+j]*xx[k];
                                    }
                                }
                            }
                            if (shouldEstimateAmbient) {
                                if (lightType==TYPE_GRAYSCALE) {
                                    ambientLight.x=xx[nlight-1]; ambientLight.y=xx[nlight-1]; ambientLight.z=xx[nlight-1];
                                } else {
                                    if (d==0) { ambientLight.x=xx[nlight-1]; }
                                    else if (d==1) { ambientLight.y=xx[nlight-1]; }
                                    else if (d==2) { ambientLight.z=xx[nlight-1]; }
                                }
                            } else {
                                ambientLight.x=.0f; ambientLight.y=.0f; ambientLight.z=.0f;
                            }
                            break;
                        default:
                            printf("Sorry could not find a solution\n");
                            break;
                    }
                } else {
                    printf("Sorry could not optimize\n");
                    if (res==MSK_RES_ERR_CON_Q_NOT_PSD) {
                        printf("Q is not positive semi-definite\n");
                    }
                    char symname[MSK_MAX_STR_LEN];
                    char desc[MSK_MAX_STR_LEN];
                    
                    MSK_getcodedesc(res, symname, desc);
                    printf("Error %s - '%s'\n",symname,desc);
                }
            } else printf("Sorry could not put in the H matrix");
        }
    
    path=direc; FILE* flight=fopen(path.append("diffuseLight.dat").c_str(),"w");
    for (int j=0; j<ndivs; j++) {
        fwrite(&diffuseLight[0][j].x, 1, sizeof(float), flight);
        fwrite(&diffuseLight[0][j].y, 1, sizeof(float), flight);
        fwrite(&diffuseLight[0][j].z, 1, sizeof(float), flight);
    }
    fclose(flight);
    path=direc; flight=fopen(path.append("ambientLight.dat").c_str(), "w");
    fwrite(&ambientLight.x,1,sizeof(float),flight);
    fwrite(&ambientLight.y,1,sizeof(float),flight);
    fwrite(&ambientLight.z,1,sizeof(float),flight);
    fclose(flight);
	
	/*
	if (workengine)
		engClose(workengine);
	
	mxDestroyArray(mxName);
	mxDestroyArray(mxNdivs);
	mxDestroyArray(mxLambda1);
	mxDestroyArray(mxLambda2);
    mxDestroyArray(mxAmbientLight);
     */
	
	delete isect;
	delete lisect;
	delete[] H;
	delete[] Hr;
	delete[] Hg;
	delete[] Hb;
	delete[] Hopt;
	delete[] hsubk;
	delete[] hsubl;
	//	delete[] K;
	//	delete[] B;
	delete[] f;
	delete[] fr;
	delete[] fg;
	delete[] fb;
	delete[] fopt;
	
}
