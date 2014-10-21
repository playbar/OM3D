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

#include "ANNhelp.h"

void anearest(float* array, int* dataidx, int* queryidx, int ndata, int nquery, int dim, int* nearestidx, float* distances)
{
	// nearestidx should be of size nquery
	double eps=0;
	if (dim>3) {
		eps=1e-4;
	}
	int i=0, j=0;
	int k=1; // we'll prlly only need one index
	
	ANNpointArray		dataPts;				// data points
	ANNpoint			queryPt;				// query point
	ANNidxArray			nnIdx;					// near neighbor indices
	ANNdistArray		dists;					// near neighbor distances
	ANNkd_tree*			kdTree;					// search structure
	
	queryPt = annAllocPt(dim);					// allocate query point
	dataPts = annAllocPts(ndata, dim);			// allocate data points
	nnIdx = new ANNidx[k];						// allocate near neigh indices
	dists = new ANNdist[k];						// allocate near neighbor dists
	
	for (i=0; i<ndata; i++) {
		for (j=0; j<dim; j++) {
			dataPts[i][j]=(ANNcoord)array[dataidx[i]*dim+j];
		}
	}
	kdTree=new ANNkd_tree(dataPts, ndata, dim);
	
	for (i=0; i<nquery; i++) {
		for (j=0; j<dim; j++) {
			queryPt[j]=(ANNcoord)array[queryidx[i]*dim+j];
		}
		kdTree->annkSearch(queryPt, k, nnIdx, dists, eps);
		nearestidx[i]=(int)nnIdx[0];
		distances[i]=(float)dists[0];
		//printf("ANNSearch, %d\n", i);
	}
	
	delete[] nnIdx;
	delete[] dists;
	delete kdTree;
}

void anearestself(float* array, int n, int dim, int* nearestidx, float* distances, int k)
{
	// nearestidx should be of size nquery
	double eps=0;
	if (dim>3) {
		eps=1e-4;
	}
	int i=0, j=0;
	int ndata=n; int nquery=n;
	
	ANNpointArray		dataPts;				// data points
	ANNpoint			queryPt;				// query point
	ANNidxArray			nnIdx;					// near neighbor indices
	ANNdistArray		dists;					// near neighbor distances
	ANNkd_tree*			kdTree;					// search structure
	
	queryPt = annAllocPt(dim);					// allocate query point
	dataPts = annAllocPts(ndata, dim);			// allocate data points
	nnIdx = new ANNidx[k];						// allocate near neigh indices
	dists = new ANNdist[k];						// allocate near neighbor dists
	
	for (i=0; i<ndata; i++) {
		for (j=0; j<dim; j++) {
			dataPts[i][j]=(ANNcoord)array[i*dim+j];
		}
	}
	kdTree=new ANNkd_tree(dataPts, ndata, dim);
	
	for (i=0; i<nquery; i++) {
		for (j=0; j<dim; j++) {
			queryPt[j]=(ANNcoord)array[i*dim+j];
		}
		kdTree->annkSearch(queryPt, k, nnIdx, dists, eps);
		for (int kk=0; kk<k; kk++) {
			nearestidx[k*i+kk]=(int)nnIdx[kk];
			distances[k*i+kk]=(float)dists[kk];
		}
		//printf("ANNSearch, %d\n", i);
	}
	
	delete[] nnIdx;
	delete[] dists;
	delete kdTree;
}


void anearestseparate(float* dataArray, float* queryArray, int ndata, int nquery, int dim, int* nearestidx, float* distances)
{
	// nearestidx should be of size nquery
	double eps=0;
	if (dim>3) {
		eps=1e-4;
	}
	int i=0, j=0;
	int k=1; // we'll prlly only need one index
	
	ANNpointArray		dataPts;				// data points
	ANNpoint			queryPt;				// query point
	ANNidxArray			nnIdx;					// near neighbor indices
	ANNdistArray		dists;					// near neighbor distances
	ANNkd_tree*			kdTree;					// search structure
	
	queryPt = annAllocPt(dim);					// allocate query point
	dataPts = annAllocPts(ndata, dim);			// allocate data points
	nnIdx = new ANNidx[k];						// allocate near neigh indices
	dists = new ANNdist[k];						// allocate near neighbor dists
	
	for (i=0; i<ndata; i++) {
		for (j=0; j<dim; j++) {
			dataPts[i][j]=(ANNcoord)dataArray[i*dim+j];
		}
	}
	
	kdTree=new ANNkd_tree(dataPts, ndata, dim);
	
	for (i=0; i<nquery; i++) {
		for (j=0; j<dim; j++) {
			queryPt[j]=(ANNcoord)queryArray[i*dim+j];
		}
		kdTree->annkSearch(queryPt, k, nnIdx, dists, eps);
		nearestidx[i]=(int)nnIdx[0];
		distances[i]=(float)dists[0];
		printf("ANNSearch, %d\n", i);
	}
	
	delete[] nnIdx;
	delete[] dists;
	delete kdTree;
}


void buildTree(float* array, int* dataidx, int ndata, int dim, ANNkd_tree* kdTree)
{
	int i=0, j=0;
	
	ANNpointArray		dataPts;				// data points
	
	dataPts = annAllocPts(ndata, dim);			// allocate data points
	
	for (i=0; i<ndata; i++) {
		for (j=0; j<dim; j++) {
			dataPts[i][j]=(ANNcoord)array[dataidx[i]*dim+j];
		}
	}
	
	kdTree=new ANNkd_tree(dataPts, ndata, dim);
}

void searchNearest1(float* array, ANNkd_tree* kdTree, int* queryidx, int nquery, int dim, int* nearestidx)
{
	// nearestidx should be of size nquery
	double eps=0;
	int i=0, j=0;
	int k=1; // we'll prlly only need one index
	
	ANNpoint			queryPt;				// query point
	ANNidxArray			nnIdx;					// near neighbor indices
	ANNdistArray		dists;					// near neighbor distances
	
	queryPt = annAllocPt(dim);					// allocate query point
	nnIdx = new ANNidx[k];						// allocate near neigh indices
	dists = new ANNdist[k];						// allocate near neighbor dists
	
	for (i=0; i<nquery; i++) {
		for (j=0; j<dim; j++) {
			queryPt[j]=(ANNcoord)array[queryidx[i]*dim+j];
		}
		kdTree->annkSearch(queryPt, k, nnIdx, dists, eps);
		nearestidx[i]=(int)nnIdx[0];
	}
	
	delete[] nnIdx;
	delete[] dists;
}

