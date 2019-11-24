	package com.neocoretechs.machinevision.hough3d;
	/**
	 * 3D matrix
	 * @author jg
	 * Copyright (C) NeoCoreTechs 2019
	 */
	public class Matrix3 {
	    private static boolean DEBUG = false;
		double[] values;
	    
	    Matrix3(double[] values) {
	        this.values = values;
	    }
	    public Matrix3() {
			values = new double[9];
		}
		public Matrix3(double[][] mat) {
			values = new double[9];
		    for (int i = 0; i<3; i++) { 
		    	for(int j = 0; j < 3; j++) 
		    		this.values[i*3+j] = mat[i][j];
		    }
		}
		
	    public void set(int row, int col, double val) {
	    	values[row*3+col] = val;
	    }
		public double get(int row, int col) {
			return values[row*3+col];
		}
		
		public double[][] get() {
			double[][] out = new double[3][3];
		    for (int i = 0; i<3; i++) { 
		    	for(int j = 0; j < 3; j++) 
		    		out[i][j]= this.values[i*3+j];
		    }
		    return out;
		}
		public double[] getValues() { return values; }
		
		public Matrix3 diag() {
		    /* Returns the diagonal of this matrix (as a vector), or
		       if given a vector, returns a diagonal matrix
		    */
		    Matrix3 t = new Matrix3();
		    int i, k, j;
		    for (i = 0; i<3; i++) {    
		      k=0; j=i; 
		      t.values[i*3+k] = this.values[i*3+j];
		    }
		    return t;
		}

		public Matrix3 add(Matrix3 m1){
		    // Return the matrix m = m1 + m2
		    Matrix3 m=new Matrix3();
		    int i,j;
		    for (i=0; i<3; i++) {
		    	for (j=0; j<3; j++) {
		    		m.values[i*3+j] = m1.values[i*3+j] + values[i*3+j]; 
		    	}
		    }
		    return m;
		}

		public Matrix3 subtract(Matrix3 m1){
		    // Return the matrix m = m1 - m2
			Matrix3 m=new Matrix3();
			int i,j;
			for (i=0; i<3; i++) {
			  for (j=0; j<3; j++) {
			    m.values[i*3+j] = values[i*3+j] - m1.values[i*3+j]; 
			  }
			}
			return m;
		}
		  
		Matrix3 multiply(Matrix3 other) {
	        double[] result = new double[9];
	        for (int row = 0; row < 3; row++) {
	            for (int col = 0; col < 3; col++) {
	                for (int i = 0; i < 3; i++) {
	                    result[row * 3 + col] +=
	                        this.values[row * 3 + i] * other.values[i * 3 + col];
	                }
	            }
	        }
	        return new Matrix3(result);
	    }
		
		public static Matrix3 multiply(double d, Matrix3 m1){
			    // Return the matrix m = d*m1
			    Matrix3 m=new Matrix3();
			    int i,j;
			    for (i=0; i<3; i++) {
			      for (j=0; j<3; j++) {
			    	  m.values[i*3+j] = d * m1.values[i*3+j];
			      }
			    }
			    return m;
		}
		
		public Matrix3 multiply(double d){
			Matrix3 m=new Matrix3();
		    // Return the matrix m = d*m1
		    int i,j;
		    for (i=0; i<3; i++) {
		      for (j=0; j<3; j++) {
		    	  m.values[i*3+j] = d * values[i*3+j];
		      }
		    }
		    return m;
		}
		/**
		 *  matrix-vector multiplication (y = A * x)
		 * @param a
		 * @param x
		 * @return
		 */
	    public double[] multiply( double[] x) {
	        double[] y = new double[3];
	        for (int i = 0; i < 3; i++)
	            for (int j = 0; j < 3; j++)
	                y[i] += values[i*3+j] * x[j];
	        return y;
	    }


	    // vector-matrix multiplication (y = x^T A)
	    public static double[] multiply(double[] x, double[][] a) {
	        int m = a.length;
	        int n = a[0].length;
	        if (x.length != m) throw new RuntimeException("Illegal matrix dimensions.");
	        double[] y = new double[n];
	        for (int j = 0; j < n; j++)
	            for (int i = 0; i < m; i++)
	                y[j] += a[i][j] * x[i];
	        return y;
	    }

		public Matrix3 divide(double d){
		    // Return the matrix m = d*m1
			Matrix3 m=new Matrix3();
		    int i,j;
		    for (i=0; i<3; i++) {
		      for (j=0; j<3; j++) {
		    	  m.values[i*3+j] = values[i*3+j] / d;
		      }
		    }
		    return m;
		}
		/**
	     * Computes the determinant of this matrix.
	     * @return the determinant of the matrix
	     */
	    public final double determinant() {
	       return values[0]*(values[1*3+1]*values[2*3+2] - values[1*3+2]*values[2*3+1])
	              + values[1]*(values[1*3+2]*values[2*3+0] - values[1*3+0]*values[2*3+2])
	              + values[2]*(values[1*3+0]*values[2*3+1] - values[1*3+1]*values[2*3+0]);
	    }     
	    
	  /**
	  * This function calculates the inverse of the matrix A.
	  * @return null if matrix is singular
	  */
	  public final Matrix3 invert()  {
	   int N = 3; //Size of the to-be-inverted N x N square matrix 'A'.       
	   // 'A' is the to-be-inverted matrix. should be a perfect identity matrix.      
	   double[][] A = get();
	  /* Its i-th row shows the position of '1' in the i-th row of the pivot that is used  
	   * when performing the LUP decomposition of A. The rest of the elements in that row of  
	   * the pivot would be zero. In this program, we call this array 'P' a 'permutation'. */  
	   int[] P = new int[N+1];  	    
	   double[][] B = new double[N+1][N+1];
	   double[] X = new double[N+1];
	   double[] Y = new double[N+1]; //Temporary spaces.          
	  /* Performing LUP-decomposition of the matrix 'A'. If successful, the 'U' is stored in  
	   * its upper diagonal, and the 'L' is stored in the remaining traigular space. Note that  
	   * all the diagonal elements of 'L' are 1, which are not stored. P is the pivot array
	   * the diagonal of the matrix belongs to U
	   * */  
	   if(LUPdecompose(N, A, P) < 0) 
		   return null;      
	  /* Inverting the matrix based on the LUP decomposed A. The inverse is returned through  
	   * the matrix 'A' itself. */  
	    if(LUPinverse(N, P, A, B, X, Y) < 0) 
	    	return null;      
	  /* Multiplying the inverse-of-A (stored in A) with A (stored in A1). The product is  
	   * stored in 'I'. Ideally, 'I' should be a perfect identity matrix.  
	   *for(i=1; i <= N; i++) for(j = 1; j <= N; j++)  
	   * for(I[i][j] = 0, k = 1; k <= N; k++) I[i][j] += A[i][k]*A1[k][j];    
	   *printf("\nProduct of the calculated inverse-of-A with A:\n");  
	   *for(i = 1; i <= N; i++)   {  
	   *  for(j = 1; j <= N; j++) printf("\t%E", (float)I[i][j]);  
	   *  printf("\n");  
	   *  }
	   */
	    return new Matrix3(A);
	  }
	  
	  /**
	   *        
	   * This function decomposes the matrix 'A' into L, U, and P. If successful,  
	   * the L and the U are stored in 'A', and information about the pivot in 'P'.  
	   * The diagonal elements of 'L' are all 1, and therefore they are not stored. 
	   */  
	  private static int LUPdecompose(int size, double[][] A, int[] P)  {  
	    int i, j, k, kd = 0, T;  
	    double p, t;      
	    // Finding the pivot of the LUP decomposition.
	    for(i=1; i<size; i++) 
	    	P[i] = i; //Initializing.   
	    for(k=1; k<size-1; k++)  {  
	      p = 0;  
	      for(i=k; i<size; i++)  {  
	        t = A[i][k];  
	        if(t < 0) 
	        	t *= -1; //Absolute value of 't'.  
	        if(t > p) {  
	           p = t;  
	           kd = i;  
	        }  
	      }  
	      // Matrix is singular
	      if(p == 0)  { 	  
	    	if(DEBUG )
	    		System.out.println("Matrix3 LUPdecompose(): matrix is singular ");  
	        return -1;  
	      }      
	     /* Exchanging the rows according to the pivot determined above. */  
	      T = P[kd];  
	      P[kd] = P[k];  
	      P[k] = T;  
	      for(i=1; i<size; i++) {  
	         t = A[kd][i];  
	         A[kd][i] = A[k][i];  
	         A[k][i] = t;  
	      }  
	      for(i=k+1; i<size; i++){ //Performing subtraction to decompose A as LU.           
	         A[i][k] = A[i][k]/A[k][k];  
	         for(j=k+1; j<size; j++) 
	        	 A[i][j] -= A[i][k]*A[k][j];  
	      }  
	    } // k loop, At end 'A' contains the L (without the diagonal elements, which are all 1) and the U.   
	    return 0;  
	  }  
	    
	  /**
	   * This function calculates the inverse of the LUP decomposed matrix 'LU' and pivoting  
	   * information stored in 'P'. The inverse is returned through the matrix 'LU' itself.  
	   * 'B', X', and 'Y' are used as temporary spaces. 
	   */  
	  private static int LUPinverse(int size, int[] P, double[][] LU,double[][] B, double[] X, double[] Y) {  
	    int i, j, n, m;  
	    double t;    
	    //Initializing X and Y.  
	    for(n=1; n<size; n++) 
	    	X[n] = Y[n] = 0;      
	    //Solving LUX = Pe, in order to calculate the inverse of 'A'. Here, 'e' is a column  
	    // vector of the identity matrix of size 'size-1'. Solving for all 'e'.   
	    for(i=1; i<size; i++)  {  
	     //Storing elements of the i-th column of the identity matrix in i-th row of 'B'.  
	     for(j = 1; j<size; j++) 
	    	B[i][j] = 0;  
	     B[i][i] = 1;      
	     //Solving Ly = Pb.  
	     for(n=1; n<size; n++) {  
	     	t = 0;  
	     	for(m=1; m<=n-1; m++) 
	     		t += LU[n][m]*Y[m];  
	     	Y[n] = B[i][P[n]]-t;  
	     }      
	     //Solving Ux = y.  
	     for(n=size-1; n>=1; n--) {  
	    	t = 0;  
	      	for(m = n+1; m < size; m++) 
	    	  t += LU[n][m]*X[m];  
	      	X[n] = (Y[n]-t)/LU[n][n];  
	     }
	     //Now, X contains the solution.
	     for(j = 1; j<size; j++) 
	    	 B[i][j] = X[j]; //Copying 'X' into the same row of 'B'.  
	    } // i=1 i<isize
	    //Now, 'B' the transpose of the inverse of 'A'.
	    //Copying transpose of 'B' into 'LU', which would the inverse of 'A'.
	    for(i=1; i<size; i++) 
	    	for(j=1; j<size; j++) 
	    		LU[i][j] = B[j][i];  
	    return 0;  
	  }
	  
		public void transposeInplace() {
		     // transpose in-place
	        for (int i = 0; i < 3; i++) {
	            for (int j = i+1; j < 3; j++) {
	                double temp = values[i*3+j];
	                values[i*3+j] = values[j*3+i];
	                values[j*3+i] = temp;
	            }
	        }
		}
		
		public Matrix3 transpose() {
			Matrix3 out = new Matrix3();
		     // transpose 
	        for (int i = 0; i < 3; i++) {
	            for (int j = i+1; j < 3; j++) {
	                out.values[i*3+j] = values[j*3+i];
	            }
	        }
	        return out;
		}
		
		public static Matrix3 transpose(Matrix3 mat) {
			Matrix3 out = new Matrix3();
		     // transpose 
	        for (int i = 0; i < 3; i++) {
	            for (int j = i+1; j < 3; j++) {
	                out.values[i*3+j] = mat.values[j*3+i];
	            }
	        }
	        return out;
		}
		/**
		 * Transpose 3 element vector to 1xN in 3x3 matrix
		 * @param vec
		 * @return
		 */
		public static Matrix3 transpose(double[] vec) {
			Matrix3 out = new Matrix3();
		     // transpose vector to nx1
	         for (int j = 0; j < 3; j++) {
	             out.values[j] =vec[j];
	         }
	        return out;
		}
		
	    public String toString() {
	    	return String.format("[%15.8f,%15.8f,%15.8f]\r\n[%15.8f,%15.8f,%15.8f]\r\n[%15.8f,%15.8f,%15.8f]\r\n",values[0],values[1],values[2],values[3],values[4],values[5],values[6],values[7],values[8]); 
	    	//return "["+values[0]+" "+values[1]+" "+values[2]+"]\r\n["+values[3]+" "+values[4]+" "+values[5]+"]\r\n["+values[6]+" "+values[7]+" "+values[8]+"]\r\n";
	
	    }
	    
	

	}