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

#import "CameraSetup.h"
//#define NOT_EPNP

@implementation CameraSetup

- (void) thePrint
{
	printf("%d\n", [imageView getOpenglViewNumber]);
	printf("%d\n", [libraryObjectView getOpenglViewNumber]);
}

- (id) init
{
	self=[super init];
	if (!self) return nil;
	[[NSNotificationCenter defaultCenter] addObserver:self selector:@selector(receivedNotification:) name:@"arapNotification" object:nil];
	[[NSNotificationCenter defaultCenter] addObserver:self selector:@selector(receivedNotification:) name:@"pickPointsNotification" object:nil];
	[[NSNotificationCenter defaultCenter] addObserver:self selector:@selector(receivedNotification:) name:@"lassoDeformNotification" object:nil];
	[[NSNotificationCenter defaultCenter] addObserver:self selector:@selector(receivedNotification:) name:@"imageUndoNotification" object:nil];
	[[NSNotificationCenter defaultCenter] addObserver:self selector:@selector(receivedNotification:) name:@"imageRedoNotification" object:nil];
	[[NSNotificationCenter defaultCenter] addObserver:self selector:@selector(receivedNotification:) name:@"groupForwardNotification" object:nil];
	[[NSNotificationCenter defaultCenter] addObserver:self selector:@selector(receivedNotification:) name:@"groupBackwardNotification" object:nil];
	[[NSNotificationCenter defaultCenter] addObserver:self selector:@selector(receivedNotification:) name:@"camParsNotification" object:nil];
	return self;
}

- (void) receivedNotification:(NSNotification*)notification
{
	if ([[notification name] isEqualToString:@"arapNotification"])
	{
		//BasicOpenGLView* myOpenglView=[notification object];
		//printf("Received points notification from %d\n", [myOpenglView getOpenglViewNumber] );
		
		// This notification will come from the imageView, so send the vertices over to the object view;
		
		// following is tricky
        if ([ [notification object] isEqual:imageView]) {
            GLfloat* VV=new GLfloat[ 3*[imageView getNumVertices]];
            [imageView getVertices:VV];
            [libraryObjectView setVertices:VV];
            delete[] VV;
        } else if ([ [notification object] isEqual:libraryObjectView]) {
            GLfloat* VV=new GLfloat[ 3*[libraryObjectView getNumVertices]];
            [libraryObjectView getVertices:VV];
            [imageView setVertices:VV];
            delete[] VV;
        }

	}
    else if ([[notification name] isEqualToString:@"pickPointsNotification"])
    {
        [imageView setOperationMode:MODE_PICKPOINTS];
    } else if ([[notification name] isEqualToString:@"lassoDeformNotification"])
    {
        [libraryObjectView setOperationMode:MODE_LASSODEFORM];
    }
    else if ([[notification name] isEqualToString:@"imageUndoNotification"]) {
        if ([imageView getIsSetInitTransform]) {
        [imageView undo];
        }
    } else if([[notification name] isEqualToString:@"imageRedoNotification"]) {
        if ([imageView getIsSetInitTransform]) {
        [imageView redo];
        }
    } else if([[notification name] isEqualToString:@"groupForwardNotification"]) {
        if ([imageView getIsSetInitTransform]) {
        if (![imageView isGroupsExists]) {
            if ([imageView getOperationMode]==MODE_LASSODEFORM) [imageView setDisplayGroupNum:0]; else [imageView setDisplayGroupNum:0];
        } else {
            if ([imageView getOperationMode]==MODE_LASSODEFORM) {
                [imageView setDisplayGroupNum:[imageView getDisplayGroupNum]+1];
                if ([imageView getDisplayGroupNum]>[imageView getNumGroups]) [imageView setDisplayGroupNum:0];
            }
            else {
                [imageView setDisplayGroupNum:[imageView getDisplayGroupNum]+1];
                if ([imageView getDisplayGroupNum]>[imageView getNumGroups]) [imageView setDisplayGroupNum:0];
            }
        }
        }
        [imageView setNeedsDisplay:YES];
    } else if([[notification name] isEqualToString:@"groupBackwardNotification"]) {
        if ([imageView getIsSetInitTransform]) {
        if (![imageView isGroupsExists]) {
            if ([imageView getOperationMode]==MODE_LASSODEFORM) [imageView setDisplayGroupNum:0]; else [imageView setDisplayGroupNum:0];
        }else {
            if ([imageView getOperationMode]==MODE_LASSODEFORM) {
                [imageView setDisplayGroupNum:[imageView getDisplayGroupNum]-1];
                if ([imageView getDisplayGroupNum]<0) [imageView setDisplayGroupNum:[imageView getNumGroups]];
            }
            else {
                [imageView setDisplayGroupNum:[imageView getDisplayGroupNum]-1];
                if ([imageView getDisplayGroupNum]<0) [imageView setDisplayGroupNum:[imageView getNumGroups]];
            }
        }
        [imageView setNeedsDisplay:YES];
        }
    } else if ([[notification name] isEqualToString:@"camParsNotification"]) {
        [self executeCamParsAction];
    }
}

- (void)camParsAction:(id)sender
{
	[self executeCamParsAction];
}

- (void)executeCamParsAction
{
    //double uc=320.0, vc=240.0, fu=726.0, fv=726.0;
	
	//double uc=IMW/(2.0), vc=IMH/(2.0), fu=(double)FX, fv=(double)FY;
	float ucf, vcf, fuf, fvf;
	[imageView getCameraParametersFu:&fuf Fv:&fvf Uc:&ucf Vc:&vcf];
	double uc=(double)ucf, vc=(double)vcf, fu=(double)fuf, fv=(double)fvf;
	//int isize=[imageView getNumClickedPoints];
    int isize=4;
	int n=isize;
	
#ifdef NOT_EPNP
	CvMat* P=cvCreateMat(3, 4, CV_32FC1);
	CvMat* P3=cvCreateMat(3, 3, CV_32FC1);
	CvMat* A=cvCreateMat(2*n, 12, CV_32FC1);
	
//	printf("n=%d\n",n);
//	printf("K=[%f,0,%f;0,%f,%f;0,0,1];",fu,uc,fv,vc);
	
	if (isize !=0) {
//		printf("PP=[");
		for (int i=0; i<n; i++) {
			GLfloat xx, yy, zz, XX, YY, ZZ;
			
			[imageView getAClickedPointGivenPos:i X:&xx Y:&yy Z:&zz];
			[libraryObjectView getAClickedPointGivenPos:i X:&XX Y:&YY Z:&ZZ];
			xx=(xx-uc)/fu;
			yy=(yy-vc)/fv;
			
//			printf("%f, %f, %f, %f, %f\n", xx, yy, XX, YY, ZZ);
			
			cvSet2D(A, 2*i, 0, cvScalar(-XX));  cvSet2D(A, 2*i, 1, cvScalar(-YY));  cvSet2D(A, 2*i, 2, cvScalar(-ZZ));   cvSet2D(A, 2*i, 3, cvScalar(-1.0));
			cvSet2D(A, 2*i, 8, cvScalar(xx*XX));cvSet2D(A, 2*i, 9, cvScalar(xx*YY));cvSet2D(A, 2*i, 10, cvScalar(xx*ZZ));cvSet2D(A, 2*i, 11, cvScalar(xx));
			cvSet2D(A, 2*i+1, 4, cvScalar(-XX));  cvSet2D(A, 2*i+1, 5, cvScalar(-YY));  cvSet2D(A, 2*i+1, 6, cvScalar(-ZZ));   cvSet2D(A, 2*i+1, 7, cvScalar(-1.0));
			cvSet2D(A, 2*i+1, 8, cvScalar(yy*XX));cvSet2D(A, 2*i+1, 9, cvScalar(yy*YY));cvSet2D(A, 2*i+1, 10, cvScalar(yy*ZZ));cvSet2D(A, 2*i+1, 11, cvScalar(yy));
		}
		
		CvMat* UA=cvCreateMat(2*n, 12, CV_32FC1);
		CvMat* SA=cvCreateMat(12, 12, CV_32FC1);
		CvMat* VA=cvCreateMat(12, 12, CV_32FC1);
		cvSVD(A, SA, UA, VA);
		
		for (int ii=0; ii<3; ii++) {
			for (int jj=0; jj<4; jj++) {
				cvSet2D(P, ii, jj, cvGet2D(VA, 4*ii+jj, 11));
				if (jj!=3) {
					cvSet2D(P3, ii, jj, cvGet2D(VA, 4*ii+jj, 11));
				}
			}
		}
		
		CvMat* U=cvCreateMat(3, 3, CV_32FC1);
		CvMat* Vt=cvCreateMat(3, 3, CV_32FC1);
		CvMat* S=cvCreateMat(3, 3, CV_32FC1);
		CvMat* T1=cvCreateMat(3, 3, CV_32FC1);
		
		cvSVD(P3, S, U, Vt, CV_SVD_V_T);
		cvGEMM(S, Vt, 1.0, NULL, 1.0, T1);
		
		GLfloat* VV=new GLfloat[ 3*[imageView getNumVertices]];
		[imageView getVertices:VV];
		
		for (int i=0; i<[imageView getNumVertices]; i++) {
			GLfloat Xo, Yo, Zo;
			Xo=VV[3*i]*T1->data.fl[0]/S->data.fl[0]+VV[3*i+1]*T1->data.fl[1]/S->data.fl[0]+VV[3*i+2]*T1->data.fl[2]/S->data.fl[0];
			Yo=VV[3*i]*T1->data.fl[3]/S->data.fl[0]+VV[3*i+1]*T1->data.fl[4]/S->data.fl[0]+VV[3*i+2]*T1->data.fl[5]/S->data.fl[0];
			Zo=VV[3*i]*T1->data.fl[6]/S->data.fl[0]+VV[3*i+1]*T1->data.fl[7]/S->data.fl[0]+VV[3*i+2]*T1->data.fl[8]/S->data.fl[0];
			VV[3*i]=Xo; VV[3*i+1]=Yo; VV[3*i+2]=Zo;
		}
		
		[imageView setVertices:VV];
		[libraryObjectView setVertices:VV];
		delete[] VV;
		
		
		for (int ii=0; ii<3; ii++) {
			for (int jj=0; jj<3; jj++) {
				Rest[ii][jj]=*(cvGet2D(U, ii, jj).val);
			}
			test[ii]=(*(cvGet2D(P, ii, 3).val))/S->data.fl[0];
		}
	}
	
	int ii, jj;
/*	printf("R=[");
	for (ii=0; ii<3; ii++)
	{
		for		(jj=0; jj<3; jj++)
			printf("%f ", Rest[ii][jj]);
		printf("\n");
	}
	printf("]\n");
	printf("t=[");
	for (ii=0; ii<3; ii++) {
		printf("%f ", test[ii]);
	}
	printf("]\n");*/
	
	//[imageView setNeedsDisplay:YES];
	[libraryObjectView setNeedsDisplay:YES];
	
#else
	PnP.set_internal_parameters(uc, vc, fu, fv); // provide teh internal parametesr (camera center, focus)
	PnP.set_maximum_number_of_correspondences(n); // provide max number of correspondences
	
	PnP.reset_correspondences(); // reset correspondences
	
//	printf("n=%d\n",n);
//	printf("K=[%f,0,%f;0,%f,%f;0,0,1];",fu,fv,uc,vc);
	
	if (isize != 0)
	{
		int i=0;
//		printf("PP=[");
		for (i=0; i<isize; i++)
		{
			GLfloat xx, yy, zz, XX, YY, ZZ;
			
			[imageView getAClickedPointGivenPos:i X:&xx Y:&yy Z:&zz];
			[libraryObjectView getAClickedPointGivenPos:i X:&XX Y:&YY Z:&ZZ];
            
            //if (i==0) { xx=888; yy=190; XX=5.8881; YY=-8.6034; ZZ=101.6701; }
            //else if (i==1) { xx=1182; yy=95; XX=30.4128; YY=-2.8452; ZZ=112.4779; }
            //else if (i==2) { xx=1459; yy=231; XX=34.2381; YY=-3.6551; ZZ=113.7224; }
            //else if (i==3) { xx=97; yy=64; XX=21.9482; YY=-0.5793; ZZ=109.8494; }
            
//			printf("%f, %f, %f, %f, %f\n", xx, yy, XX, YY, ZZ);
			PnP.add_correspondence((double)XX, (double)YY, (double)ZZ, (double)xx, (double)yy); // add correspondence into system
			
		}
//		printf("];\n");
		double err2 = PnP.compute_pose(Rest, test); // perform the computation, return the estimated matrices and the reproj error
		
		int ii, jj;
/*		printf("R=[");
		for (ii=0; ii<3; ii++)
		{
			for		(jj=0; jj<3; jj++)
				printf("%f ", Rest[ii][jj]);
			printf("\n");
		}
		printf("]\n");
		printf("t=[");
		for (ii=0; ii<3; ii++) {
			printf("%f ", test[ii]);
		}
		printf("]\n");*/
	}
	
#endif
	GLfloat pt[3];
	[libraryObjectView getPoint:pt Type:0 Index:0]; [imageView setPoint:pt Type:0 Index:0];
	[libraryObjectView getPoint:pt Type:3 Index:0]; [imageView setPoint:pt Type:3 Index:0];
	
	n=[libraryObjectView getNumIndices];
	[imageView setNumIndices:n];
	int* rc=new int[n];
	[libraryObjectView getIndices:rc];
	[imageView setIndices:rc];
	delete[] rc;
	//GLfloat* pc=new GLfloat[2*n];
	//[libraryObjectView getPtsConstrained:pc];
	//[imageView setPtsConstrained:];
	//delete[] pc;
	
	
	[imageView setInitRotation:Rest Translation:test];
	[imageView clearClickedPoints];
	[libraryObjectView clearClickedPoints];
	
	// memcopy the library object view contact point and before and after to the image view
	// transfer over the ray constrained indexes
}

- (void) dealloc
{
	[[NSNotificationCenter defaultCenter] removeObserver:self];
	[super dealloc];
}

@end
