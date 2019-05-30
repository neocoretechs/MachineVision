	package com.neocoretechs.machinevision.hough3d;
	/**
	 * 3D matrix
	 * @author jg
	 * Copyright (C) NeoCoreTechs 2019
	 */
	public class Matrix3 {
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
	     * General invert routine.  Inverts m1 and places the result in "this".
	     * Note that this routine handles both the "this" version and the
	     * non-"this" version.
	     *
	     * Also note that since this routine is slow anyway, we won't worry
	     * about allocating a little bit of garbage.
	     */
	    public final Matrix3 invert() {
	    	Matrix3 out = new Matrix3();
	    	double result[] = new double[9];
	    	int row_perm[] = new int[3];
	    	int i;
	    	double[]    tmp = new double[9];  // scratch matrix
	    	// Use LU decomposition and backsubstitution code specifically
	    	// for floating-point 3x3 matrices.
	    	// Copy source matrix to t1tmp
	        tmp[0] = get(0,0);
	        tmp[1] = get(0,1);
	        tmp[2] = get(0,2);

	        tmp[3] = get(1,0);
	        tmp[4] = get(1,1);
	        tmp[5] = get(1,2);

	        tmp[6] = get(2,0);
	        tmp[7] = get(2,1);
	        tmp[8] = get(2,2);
	        // Calculate LU decomposition: Is the matrix singular?
	        if (!luDecomposition(tmp, row_perm)) {
	        	// Matrix has no inverse
	        	throw new RuntimeException("matrix has no inverse");
	        }
	        // Perform back substitution on the identity matrix
	        for(i=0;i<9;i++) result[i] = 0.0;
	        result[0] = 1.0; result[4] = 1.0; result[8] = 1.0;
	        luBacksubstitution(tmp, row_perm, result);
	        out.set(0,0,result[0]);
	        out.set(0,1,result[1]);
	        out.set(0,2,result[2]);

	        out.set(1,0,result[3]);
	        out.set(1,1,result[4]);
	        out.set(1,2,result[5]);

	        out.set(2,0,result[6]);
	        out.set(2,1,result[7]);
	        out.set(2,2,result[8]);
	        return out;
	    }

	    /**
	     * Given a 3x3 array "matrix0", this function replaces it with the
	     * LU decomposition of a row-wise permutation of itself.  The input
	     * parameters are "matrix0" and "dimen".  The array "matrix0" is also
	     * an output parameter.  The vector "row_perm[3]" is an output
	     * parameter that contains the row permutations resulting from partial
	     * pivoting.  The output parameter "even_row_xchg" is 1 when the
	     * number of row exchanges is even, or -1 otherwise.  Assumes data
	     * type is always double.
	     *
	     * This function is similar to luDecomposition, except that it
	     * is tuned specifically for 3x3 matrices.
	     *
	     * Reference: Press, Flannery, Teukolsky, Vetterling,
	     *	      _Numerical_Recipes_in_C_, Cambridge University Press,
	     *	      1988, pp 40-45.
	     *
	     * @return true if the matrix is nonsingular, or false otherwise.
	     */
	    static boolean luDecomposition(double[] matrix0,
					   int[] row_perm) {

		double row_scale[] = new double[3];

		// Determine implicit scaling information by looping over rows
		{
		    int i, j;
		    int ptr, rs;
		    double big, temp;

		    ptr = 0;
		    rs = 0;

		    // For each row ...
		    i = 3;
		    while (i-- != 0) {
			big = 0.0;

			// For each column, find the largest element in the row
			j = 3;
			while (j-- != 0) {
			    temp = matrix0[ptr++];
			    temp = Math.abs(temp);
			    if (temp > big) {
				big = temp;
			    }
			}

			// Is the matrix singular?
			if (big == 0.0) {
			    return false;
			}
			row_scale[rs++] = 1.0 / big;
		    }
		}

		{
		    int j;
		    int mtx;

		    mtx = 0;

		    // For all columns, execute Crout's method
		    for (j = 0; j < 3; j++) {
			int i, imax, k;
			int target, p1, p2;
			double sum, big, temp;

			// Determine elements of upper diagonal matrix U
			for (i = 0; i < j; i++) {
			    target = mtx + (3*i) + j;
			    sum = matrix0[target];
			    k = i;
			    p1 = mtx + (3*i);
			    p2 = mtx + j;
			    while (k-- != 0) {
				sum -= matrix0[p1] * matrix0[p2];
				p1++;
				p2 += 3;
			    }
			    matrix0[target] = sum;
			}

			// Search for largest pivot element and calculate
			// intermediate elements of lower diagonal matrix L.
			big = 0.0;
			imax = -1;
			for (i = j; i < 3; i++) {
			    target = mtx + (3*i) + j;
			    sum = matrix0[target];
			    k = j;
			    p1 = mtx + (3*i);
			    p2 = mtx + j;
			    while (k-- != 0) {
				sum -= matrix0[p1] * matrix0[p2];
				p1++;
				p2 += 3;
			    }
			    matrix0[target] = sum;

			    // Is this the best pivot so far?
			    if ((temp = row_scale[i] * Math.abs(sum)) >= big) {
				big = temp;
				imax = i;
			    }
			}

			if (imax < 0) {
			    throw new RuntimeException("imax negative");
			}

			// Is a row exchange necessary?
			if (j != imax) {
			    // Yes: exchange rows
			    k = 3;
			    p1 = mtx + (3*imax);
			    p2 = mtx + (3*j);
			    while (k-- != 0) {
				temp = matrix0[p1];
				matrix0[p1++] = matrix0[p2];
				matrix0[p2++] = temp;
			    }

			    // Record change in scale factor
			    row_scale[imax] = row_scale[j];
			}

			// Record row permutation
			row_perm[j] = imax;

			// Is the matrix singular
			if (matrix0[(mtx + (3*j) + j)] == 0.0) {
			    return false;
			}

			// Divide elements of lower diagonal matrix L by pivot
			if (j != (3-1)) {
			    temp = 1.0 / (matrix0[(mtx + (3*j) + j)]);
			    target = mtx + (3*(j+1)) + j;
			    i = 2 - j;
			    while (i-- != 0) {
				matrix0[target] *= temp;
				target += 3;
			    }
			}
		    }
		}

		return true;
	    }

	    /**
	     * Solves a set of linear equations.  The input parameters "matrix1",
	     * and "row_perm" come from luDecompostionD3x3 and do not change
	     * here.  The parameter "matrix2" is a set of column vectors assembled
	     * into a 3x3 matrix of floating-point values.  The procedure takes each
	     * column of "matrix2" in turn and treats it as the right-hand side of the
	     * matrix equation Ax = LUx = b.  The solution vector replaces the
	     * original column of the matrix.
	     *
	     * If "matrix2" is the identity matrix, the procedure replaces its contents
	     * with the inverse of the matrix from which "matrix1" was originally
	     * derived.
	     */
	    //
	    // Reference: Press, Flannery, Teukolsky, Vetterling,
	    //	      _Numerical_Recipes_in_C_, Cambridge University Press,
	    //	      1988, pp 44-45.
	    //
	    static void luBacksubstitution(double[] matrix1,
					   int[] row_perm,
					   double[] matrix2) {

		int i, ii, ip, j, k;
		int rp;
		int cv, rv;

		//	rp = row_perm;
		rp = 0;

		// For each column vector of matrix2 ...
		for (k = 0; k < 3; k++) {
		    //	    cv = &(matrix2[0][k]);
		    cv = k;
		    ii = -1;

		    // Forward substitution
		    for (i = 0; i < 3; i++) {
			double sum;

			ip = row_perm[rp+i];
			sum = matrix2[cv+3*ip];
			matrix2[cv+3*ip] = matrix2[cv+3*i];
			if (ii >= 0) {
			    //		    rv = &(matrix1[i][0]);
			    rv = i*3;
			    for (j = ii; j <= i-1; j++) {
				sum -= matrix1[rv+j] * matrix2[cv+3*j];
			    }
			}
			else if (sum != 0.0) {
			    ii = i;
			}
			matrix2[cv+3*i] = sum;
		    }

		    // Backsubstitution
		    //	    rv = &(matrix1[3][0]);
		    rv = 2*3;
		    matrix2[cv+3*2] /= matrix1[rv+2];

		    rv -= 3;
		    matrix2[cv+3*1] = (matrix2[cv+3*1] -
				    matrix1[rv+2] * matrix2[cv+3*2]) / matrix1[rv+1];

		    rv -= 3;
		    matrix2[cv+4*0] = (matrix2[cv+3*0] -
				    matrix1[rv+1] * matrix2[cv+3*1] -
				    matrix1[rv+2] * matrix2[cv+3*2]) / matrix1[rv+0];

		}
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