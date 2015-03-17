#include "mex.h"
#include <iostream>
#include <string.h>

#include "matrix.h"
#include "kdtree.h"

using namespace std;

void sparsify (double *M,int32_t M_num,int32_t dim,int32_t* idx_sparse,int32_t &idx_size,double min_dist,double out_dist,int32_t idx_start) {
  
  // copy model data to M_data
  kdtree::KDTreeArray M_data;
  M_data.resize(boost::extents[M_num][dim]);
  for (int32_t m=0; m<M_num; m++)
    for (int32_t n=0; n<dim; n++)
      M_data[m][n] = (float)M[m*dim+n];
  
  // build a kd tree from the model point cloud
  kdtree::KDTree* tree = new kdtree::KDTree(M_data);
 
  // for all data points do
  for (int32_t i=0; i<M_num; i++) {
    
    if (i>=idx_start) {
      
      kdtree::KDTreeResultVector result;
      tree->r_nearest_around_point(i,0,min_dist,result);

      bool neighbor_exists = false;
      for (int32_t j=0; j<result.size(); j++)
        neighbor_exists |= (bool)idx_sparse[result[j].idx];

      if (!neighbor_exists) {
        idx_sparse[i] = 1;
        idx_size++;
      }
      
    // simply add
    } else {
      idx_sparse[i] = 1;
      idx_size++;
    }
  }
  
  // remove outliers
  if (out_dist>0) {
    for (int32_t i=idx_start; i<M_num; i++) {

      if (idx_sparse[i] == 1) {

        kdtree::KDTreeResultVector result;
        tree->r_nearest_around_point(i,0,out_dist,result);

        int32_t num_neighbors = 0;
        for (int32_t j=1; j<result.size(); j++)
          if (idx_sparse[result[j].idx]==1)
            num_neighbors++;
        
        if (num_neighbors==0) {
          idx_sparse[i] = 0;
          idx_size--;
        }
      }    
    }
  }
  
  // release memory of kd tree
  delete tree;
}

void mexFunction (int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[]) {

  // check arguments
  if (nrhs!=3 && nrhs!=4)
    mexErrMsgTxt("3/4 input parameters required (M,min_dist,start_idx,out_dist).");
  if (nlhs!=1)
    mexErrMsgTxt("1 output parameter required (M_sparse).");
  if (!mxIsDouble(prhs[0]) || /*mxGetM(prhs[0])!=2 &&*/ mxGetM(prhs[0])!=3)
    mexErrMsgTxt("Input M (model points) must be a double 2xN or 3xN matrix.");
  if (!mxIsDouble(prhs[1]) || mxGetM(prhs[1])*mxGetN(prhs[1])!=1)
    mexErrMsgTxt("Input min_dist must be a double scalar.");
  if (!mxIsDouble(prhs[2]) || mxGetM(prhs[2])*mxGetN(prhs[2])!=1)
    mexErrMsgTxt("Input start_idx must be a double scalar.");

  // input
  double  *M          =            (double*)mxGetPr(prhs[0]);
  int32_t  M_num      =                      mxGetN(prhs[0]);
  int32_t  dim        =                      mxGetM(prhs[0]);
  double   min_dist   =          *((double*)mxGetPr(prhs[1]));
  int32_t  idx_start  = (int32_t)*((double*)mxGetPr(prhs[2]));
  
  double out_dist = -1;
  if (nrhs==4)
    out_dist = *((double*)mxGetPr(prhs[3]));

  int32_t *idx_sparse = (int32_t*)calloc(M_num,sizeof(int32_t));
  int32_t  idx_size   = 0;

  // sparsify
  sparsify(M,M_num,dim,idx_sparse,idx_size,min_dist,out_dist,idx_start);
  
  // output
  const int dims[] = {dim,idx_size};
  plhs[0]          = mxCreateNumericArray(2,dims,mxDOUBLE_CLASS,mxREAL);
  double *M_sparse = (double*)mxGetPr(plhs[0]);
  int32_t k=0;
  for (int32_t i=0; i<M_num; i++) {
    if (idx_sparse[i]) {
      M_sparse[k*dim+0] = M[i*dim+0];
      M_sparse[k*dim+1] = M[i*dim+1];
      M_sparse[k*dim+2] = M[i*dim+2];
      k++;
    }
  }

  //cout << idx_size << endl;
  
  free(idx_sparse);
}
