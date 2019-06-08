package com.neocoretechs.machinevision.hough3d;

import java.util.ArrayList;
/**
* Computing Gaussian Trivariate Kernels for Cluster Voting.
* Let K be a cluster of approximately coplanar samples stored in an octree
* node, with covariance matrix E, and centroid u = (ux; uy; uz)T . Also
* let V = f~v1; ~v2; ~v3g be the unit eigenvectors of E and let L = l1; l2; l3 be
* their respective eigenvalues, so that li <= li+1. The equation of the plane Pi
* passing though u and with normal ~n = ~v1 = (nx; ny; nz)T is given by:
* Ax + By + Cz + D = nx*x + ny*y + nz*z - (nx*ux + ny*uy + nz*uz) = 0
* Using the following plane detection formula in the (theta, phi, rho) Hough Space:
* rho = (x * cos(theta) * sin(phi)) + (y * sin(phi) * sin(theta)) + (z * cos(phi))
* where x, y and z are the Cartesian coordinates of the samples, theta from 0 to 360
* and phi from 0 to 180 are the polar coordinates of the plane's normal vector, and
* and rho is element of real >= 0 and is the distance from the plane to the origin of the coordinate system. 
* Using this, the equation of the plane Pi can be rewritten using spherical coordinates as:
* rho = -D = ux* nx + uy*ny + uz * nz = sqrt(px**2 + py**2 + pz**2)
* theta = arctan(py/px)
* phi  = arccos(pz/p)
* where rho is element of Reals >=0, theta is between  0 and 360 degrees, phi is between 0 and 180 degrees
* and ~p = (px; py; pz)T = rho~n. 
* For the theta calculation, if the angle between ~n and ~u is bigger than 90,
* we reverse ~n's sense (i.e., multiply it by -1). When voting in an accumulator indexed by
* (theta, phi, rho), the vote distribution is based on the uncertainties associated with
* each cluster's best-fitting plane Pi (i.e., the cluster's variances sigma-squared-phi, sigma-squared-theta).
* A cluster with small variances concentrates its votes in a small region of
* the accumulator, while a cluster with large variances spreads its votes over
* a large region
* http://www.inf.ufrgs.br/~oliveira/pubs_files/HT3D/Limberger_Oliveira_3D_HT_Pre-Print_low_res.pdf pg 12
* @author jg
*
*/
public final class kernel_t {
	public static final double root22pi32 = 2.0*Math.sqrt(2.0)*Math.pow(Math.PI,1.5);
	public static final double NONZERO = 0.00001;
	private static final boolean DEBUG = true;

    octree_t node;

    double rho;
    double theta;
    double phi;

    private double constant_normal;
    //double constant_binormal;
    double voting_limit;

    //double theta_index;
    //int phi_index;
    //int rho_index;
    double[] thetaPhiRhoIndex = new double[3];

    //int votes;

    private Matrix3 covariance_rpt_normal;
    private Matrix3 covariance_rpt_inv_normal;
    /**
     * Called from voting kernel_calculation. Calculates voting_limit from trivariate gaussian distance normal.
     * Uses covariance matrix multiplied by jacobian normal multiplied by transposed normal jacobian matrix to produce
     * normal covariance matrix to generate the uncertainty propagation.
     * Element 0,0 of matrix has a fractional non-zero value added, then the matrix is inverted.
     * The square root of the absolute value of the determinant is
     * multiplied by 2.0*Math.sqrt(2.0)*Math.pow(Math.PI,1.5) to produce a scalar constant normal product.
     * The normal covariance matrix is subjected to eigenvalue decomposition and the resulting vector and matrix are
     * extracted to obtain the cluster representativeness .
     * The area importance (w_a) is set at .75 and number of points importance (w_d) is set to 1-w_a.
     * w_a and w_d are the weights associated with relative area and relative number of samples, such that wa + wd = 1.
     * The m_size of the node is divided by root size times w_a added to number of points in this cluster divided by total
     * number of points times w_d to produce the representativeness of the node.
     * The eigenvalues vector is sorted to set up the voting limit calculation (g_min).
     * The radius is defined as 2 standard deviations multiplied by the minimum eigenvalues vector element.
     * The min index column of the eigenvectors matrix is multiplied by the radius to get the rho,phi,theta to pass to the
     * trivariate gaussian distance normal function. the voting_limit is set to the return from that function.
     * @param max_point_distance
     */
   void kernel_load_parameters(double max_point_distance)  {
	  Matrix3 covariance_xyz = node.m_covariance;
	  Matrix3 jacobian_normal = new Matrix3();
      Vector4d n = node.normal1.Normalized();
      // Jacobian Matrix calculation
      //double t=1.0;
      double EPS2 = 0.00001;
      Vector4d p = n.multiply(rho);
      double w = p.x * p.x + p.y * p.y;
      double p2 = w + (p.z * p.z);
      double sqrtW = Math.sqrt(w);
      jacobian_normal.set(0,0, n.x);
	  jacobian_normal.set(0,1, n.y);
	  jacobian_normal.set(0,2, n.z);
      jacobian_normal.set(1,0, sqrtW<EPS2?(p.x * p.z)/EPS2:(p.x * p.z) / (sqrtW * p2));  
	  jacobian_normal.set(1,1, sqrtW<EPS2?(p.y * p.z)/EPS2:(p.y * p.z) / (sqrtW * p2));  
	  jacobian_normal.set(1,2, p2<EPS2?-sqrtW/EPS2:(-sqrtW / p2));
      jacobian_normal.set(2,0, (w<EPS2)?-p.y/EPS2:-p.y / w);
	  jacobian_normal.set(2,1, (w<EPS2)?p.x/EPS2:p.x / w);
	  jacobian_normal.set(2,2, 0.0);
      
	  // Area importance (w_a) 
      double w_a = 0.75;
      // Number-of-points importance (w_d)
      double w_d = 1- w_a;
      node.representativeness = (double)node.m_size/(double)node.m_root.m_size * w_a + 
								(double)node.m_indexes.size()/(double)node.m_root.m_points.size() * (w_d);

      // Uncertainty propagation
	  Matrix3 jacobian_transposed_normal = Matrix3.transpose(jacobian_normal);
      covariance_rpt_normal = jacobian_normal.multiply(covariance_xyz).multiply(jacobian_transposed_normal);
   
      // Cluster representativeness
      covariance_rpt_normal.set(0,0, covariance_rpt_normal.get(0,0)+NONZERO);
      // if invert comes back null then the matrix is singular, which means the samples are coplanar
      covariance_rpt_inv_normal = covariance_rpt_normal.invert();
      if( covariance_rpt_inv_normal == null ) {
    	  if( DEBUG ) {
    		  System.out.println("kernel_t kernel_load_parameters covariance matrix is singular for "+this);
    	  }
    	  voting_limit = 0;
    	  return;
      }
      constant_normal = root22pi32 * Math.sqrt(Math.abs(covariance_rpt_normal.determinant()));
	  EigenvalueDecomposition eigenvalue_decomp = new EigenvalueDecomposition(covariance_rpt_normal);
	  double[] eigenvalues_vector = eigenvalue_decomp.getRealEigenvalues();
	  Matrix3 eigenvectors_matrix = eigenvalue_decomp.getV();
      // Sort eigenvalues
      int min_index = 0;
      if (eigenvalues_vector[min_index] > eigenvalues_vector[1])
         min_index = 1;
      if (eigenvalues_vector[min_index] > eigenvalues_vector[2])
         min_index = 2;

      // Voting limit calculation (g_min)
      double n_of_standard_variations = 2.0;
      double radius = Math.sqrt( eigenvalues_vector[min_index] ) * n_of_standard_variations;
      voting_limit = trivariated_gaussian_dist_normal( eigenvectors_matrix.get(0, min_index) * radius, 
													   eigenvectors_matrix.get(1, min_index) * radius, 
													   eigenvectors_matrix.get(2, min_index) * radius);
      if(DEBUG)
    	  System.out.println("kernel_t kernel_load_parameters voting limit="+voting_limit);
   }

   /**
    * Sampling the Trivariate Gaussian Distribution. Called from voting.gaussian_vote_1d and kernel_load_parameters
    * Once we have computed the variances and covariances associated with
	* theta, phi, and rho, the votes are cast in the spherical accumulator using a
	* trivariate Gaussian distribution. For the multivariate non-degenerate case,
	* i.e., when the covariance matrix E is symmetric and positive definite, its
	* probability density function is of the form:
	* letting ~sigma = x - u be the displacement with respect to the center, and
	* since the votes are already centered at the best-fitting parameters (theta, phi, rho),
	* the equation
	* p(~sigma, E) = (1 / (15.7496 * (determinant(E)**.5))) * exp(-.5 * ~sigmaT * inverse(E) * ~sigma)
	* Casting votes for a given accumulator cell requires two matrix-vector multiplications
	* and one exponentiation, since the determinant of the covariance
	* matrix E and its inverse need to be calculated only once per cluster. While the
	* values of sigma-theta**2 and sigma-phi**2 are never zero, the value of sigma-rho**2
	* will be zero if all samples in the cluster are exactly coplanar. 
	* In such a case, E(theta,sigma,rho) becomes singular, and voting should be done using a 
	* bivariate Gaussian kernel defined over (theta,phi). 
	* We avoid the need to handle such a special case by adding a small value e to sigma-rho**2
	* (e.g., " = 0:001, see line 3 in Algorithm 3 in the paper). Thus, voting can always use a trivariate Gaussian kernel, 
	* without affecting the results.
    * @param rho Rho value, notice parameters are reversed..
    * @param phi
    * @param theta
    * @return
    */
   double trivariated_gaussian_dist_normal(double rho, double phi, double theta) {
      double[] displacement = new double[]{rho, phi, theta};
	  //displacement.x = rho;
	  //displacement.y = phi;
	  //displacement.z = theta
	  //dlib::vector<double,3> displacement(rho,phi,theta);
      if( covariance_rpt_inv_normal == null ) {
    	  //if( DEBUG ) {
    		//  System.out.println("kernel_t trivariated_gaussian_dist_normal covariance matrix is singular for "+this);
    	  //}
    	  return 0;
      }
      //return (std::exp(-0.5 * (dlib::trans(displacement) * covariance_rpt_inv_normal * displacement))/constant_normal);
      // two matrix vector calcs yielding vector
      double [] trans =(Matrix3.transpose(displacement).multiply(covariance_rpt_inv_normal.multiply(displacement)));
      double gaussDist = getExponentTerm(displacement, trans) / constant_normal;
	  //if(DEBUG)
		//  System.out.println("trivariate gaussian dist norml="+gaussDist);
	  return gaussDist;
   }
   /**
    * Computes the term used in the exponent.
    * the votes are already centered at the best-fitting parameters (theta, phi, rho)
    * @param values Values at which to compute density.
    * @param preMultiplied the 2 matrix vector calcs
    * @return the multiplication factor of density calculations.
    */
   private double getExponentTerm(final double[] values, final double[] preMultiplied) {
       //final double[] centered = new double[values.length];
       //for (int i = 0; i < centered.length; i++) {
       //    centered[i] = values[i] - means[i];
       //}
       // I think they are already centered from earlier?
       //final double[] preMultiplied = covariance_rpt_inv_normal.multiply(values/*centered*/);
       double sum = 0;
       for (int i = 0; i < preMultiplied.length; i++) {
           sum += preMultiplied[i] * values[i];//centered[i];
       }
       return Math.exp(-0.5 * sum);
   }
   
   @Override
   public String toString() {
	   return "kernel_t node="+node+
			   " theta="+theta+" phi="+phi+" rho="+rho+
			   " constant_normal="+constant_normal+" voting limit="+voting_limit+
			   " node.normal1="+(node != null ? node.normal1 : "null")+
			   " node.m_size="+(node != null ? node.m_size : "null")+
			   " node points="+(node != null ? node.m_indexes.size() : "null")+
			   " thetaPhiRhoIndex[0]="+thetaPhiRhoIndex[0]+
			   " thetaPhiRhoIndex[1]="+thetaPhiRhoIndex[1]+
			   " thetaPhiRhoIndex[2]="+thetaPhiRhoIndex[2]+
			   "\r\ncovariance:\r\n"+covariance_rpt_normal;
   }
}
