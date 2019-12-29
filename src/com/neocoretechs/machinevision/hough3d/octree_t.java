package com.neocoretechs.machinevision.hough3d;

import java.util.ArrayList;
/**
 * Octree implementation with recursive subdivision. Uses array primitive for child nodes.
 * Maintains arraylist of integer indexes for each node that point back to root m_points for coords
 * of source points in this node.
 * Maintains centroid of data points, number of votes for coplanarity, status as coplanar, tree level, normals to
 * potential planes, and access to root node.
 * Also has the ability to generate a fast covariance matrix of data points, compute the least variance direction, and
 * remove outliers that are farther from prospective plane than a certain tolerance.
 * @author jg
 *
 */
public final class octree_t {
   public static final double EPS = 1.E-3;
   public double mix = Double.MAX_VALUE; //root node values. Not static because we can have multiple octrees
   public double miy = Double.MAX_VALUE;
   public double miz = Double.MAX_VALUE;
   public double max = Double.MIN_VALUE;
   public double may = Double.MIN_VALUE;
   public double maz = Double.MIN_VALUE;
   public double max_distance = 0.0; // absolute longest bound in x,y, or z
   public int point_num = 0;
   public ArrayList<Vector4d> m_points = null;//new ArrayList<Vector4d>();
   public ArrayList<Vector4d> m_colors = null;//new ArrayList<Vector4d>();
   //private static long octreeNum = 0;
   //protected long octoNum;
   Matrix3 fast_covariance_matrix = new Matrix3();
   Matrix3 m_covariance = new Matrix3();

   ArrayList<Integer> m_indexes = new ArrayList<Integer>(); // points to m_points in root node from subnodes
   octree_t[] m_children = null;
   octree_t m_root = null;
   octree_t m_parent = null;
   Vector4d normal1, normal2, normal3;
   Vector4d m_middle = new Vector4d(0,0,0,0);
   Vector4d m_centroid = new Vector4d(0,0,0,0); 
   Vector4d color = new Vector4d(0.5,0.5,0.5);
   double variance1 = 0;
   double variance2 = 0;
   double variance3 = 0;
   double m_size;
   double representativeness = 0;
   short m_level = 0;
   boolean coplanar = false;
   int votes = 0;
   private static boolean DEBUG = false;
   private static boolean DEBUGVARIANCE = false;
   private static boolean DEBUGSUBDIVIDE = false;
 
   public octree_t() {
     //++octreeNum; // increment monotonically increasing number to identify this node for comparison
     //octoNum = octreeNum;
  }
   public Vector4d getNormal1() {
	   return normal1;
   }
   public Vector4d getNormal2() {
	   return normal2;
   }
   public Vector4d getNormal3() {
	   return normal3;
   }
   public double getVariance1() {
	   return variance1;
   }
   public double getVariance2() {
	   return variance2;
   }
   public double getVariance3() {
	   return variance3;
   }
   public Vector4d getCentroid() {
	   return m_centroid;
   }
   public int getVotes() {
	   return votes;
   }
   public void setVotes(int nvotes) {
	   votes = nvotes;
   }
   public ArrayList<Integer> getIndexes() {
	   return m_indexes;
   }
   public octree_t getRoot() {
	   return m_root;
   }
   public octree_t getParent() {
	   return m_parent;
   }
   public octree_t[] getChildren() {
	   return m_children;
   }
   public Vector4d getMiddle() {
	   return m_middle;
   }
   public double getSize() {
	   return m_size;
   }
	public void setNormal1(Vector4d norm1) {
		normal1 = norm1;		
	}

   /**
    * 
    */
   public void clear() {
   if (m_children != null) {
      for(int i = 0; i < 8 ; i++) {
         m_children[i].m_indexes.clear();
         m_children[i].clear();
      }
      m_children = null;
   }
  }
   /**
    * Subdivide octree nodes.
    * RECURSIVE. 
    * Dependent on settings value s_ms, which is minimum size of m_indexes array to limit recursion.
    * s_ms MUST be > 0 and large enough to prevent depth recursion from blowing the stack
    * Dependent on settings value s_level to determine the level at which m_level we check for variance direction
    * and potentially remove outliers that are greater than m_size/10 from the plane we are trying to form.
    * We use the variances from the least_variance_direction method (variance1, variance2, variance3, variance4)
    * to determine thickness and isotropy:
    *  double thickness = variance1 / variance2;
    *  double isotropy  = variance2 / variance3;
    * If thickness < than the max_thickness from settings, and isotropy is > min_isotropy from settings, then we
    * remove outliers, compute least_variance_direction again, and set coplanar to true.
    * @param settings
    */
   public void subdivide() {
		if(DEBUGSUBDIVIDE ) {
			System.out.println("octree_t subdivide...level="+m_level+" indicies="+m_indexes.size()+" centoid="+m_centroid);
		}
   // s_ms verification, obviously the value needs to be > 0 and low values seem to want to blow the stack
	if (m_indexes.size() < (int)hough_settings.s_ms) 
	    return;
   // s_level verification
	if(DEBUGSUBDIVIDE) {
		System.out.println("octree_t subdivide s_level verification in octree..."+(m_level >=hough_settings.s_level)+", "+m_level+" "+hough_settings.s_level);
	}
   if (m_level >= hough_settings.s_level) {
      // principal component analysis
      least_variance_direction();
      // Planarity verification
      double thickness = variance1 / variance2;
      double isotropy  = variance2 / variance3;
      if( DEBUG )
    	  System.out.println("octree_t subdivide thickness and isotropy="+thickness+" "+isotropy+" m_level="+m_level+" indicies="+m_indexes.size()+" centroid="+m_centroid);
      if(thickness < hough_settings.max_thickness && isotropy > hough_settings.min_isotropy) {
    		  // Refitting step
    		  remove_outliers();
    		  if( m_indexes.size() == 0) { // did we remove all points?
    			  coplanar = false;
    			  return;
    		  }
    		  least_variance_direction();
    		  coplanar = true;
    		  return;
      }
   }
   m_children = new octree_t[8];
   double newsize = m_size/2.0;
   for (int i = 0; i < 8 ; i++) {
	  m_children[i] = new octree_t();
      m_children[i].m_size = newsize;
      m_children[i].m_level = (short) (m_level+1);
      m_children[i].m_root = m_root;
      m_children[i].m_parent = this;
      m_children[i].m_indexes.ensureCapacity(m_indexes.size()/4);
   }
   double size4 = m_size/4.0;
   // Calculation of son nodes
   m_children[0].m_middle.x = m_middle.x - size4;
   m_children[1].m_middle.x = m_middle.x - size4;
   m_children[2].m_middle.x = m_middle.x - size4;
   m_children[3].m_middle.x = m_middle.x - size4;
   m_children[4].m_middle.x = m_middle.x + size4;
   m_children[5].m_middle.x = m_middle.x + size4;
   m_children[6].m_middle.x = m_middle.x + size4;
   m_children[7].m_middle.x = m_middle.x + size4;

   m_children[0].m_middle.y = m_middle.y - size4;
   m_children[1].m_middle.y = m_middle.y - size4;
   m_children[2].m_middle.y = m_middle.y + size4;
   m_children[3].m_middle.y = m_middle.y + size4;
   m_children[4].m_middle.y = m_middle.y - size4;
   m_children[5].m_middle.y = m_middle.y - size4;
   m_children[6].m_middle.y = m_middle.y + size4;
   m_children[7].m_middle.y = m_middle.y + size4;

   m_children[0].m_middle.z = m_middle.z - size4;
   m_children[1].m_middle.z = m_middle.z + size4;
   m_children[2].m_middle.z = m_middle.z - size4;
   m_children[3].m_middle.z = m_middle.z + size4;
   m_children[4].m_middle.z = m_middle.z - size4;
   m_children[5].m_middle.z = m_middle.z + size4;
   m_children[6].m_middle.z = m_middle.z - size4;
   m_children[7].m_middle.z = m_middle.z + size4;

   // putting points in its respective children
   for (int i = 0; i < m_indexes.size() ; i++) {
      int index = 0;
      if (m_root.m_points.get(m_indexes.get(i)).x > m_middle.x) {
         index+=4;
      }
      if (m_root.m_points.get(m_indexes.get(i)).y > m_middle.y)
      {
         index+=2;
      }
      if (m_root.m_points.get(m_indexes.get(i)).z > m_middle.z)
      {
         index+=1;
      }
      m_children[index].m_indexes.add(m_indexes.get(i));
      // Calculating centroid distribution (divided by the number of points below)
      m_children[index].m_centroid = m_children[index].m_centroid.add(m_root.m_points.get(m_indexes.get(i)));
   }
   
   for (int i = 0; i < 8 ; i++) {
      m_children[i].m_centroid = m_children[i].m_centroid.divide(m_children[i].m_indexes.size());
      // Recursive subdivision 
      m_children[i].subdivide();
   }
  }
   /**
    * Subdivide octree nodes WITHOUT PCA and outlier removal..
    * RECURSIVE. 
    * Dependent on settings value s_ms, which is minimum size of m_indexes array to limit recursion.
    * s_ms MUST be > 0 and large enough to prevent depth recursion from blowing the stack
    * set coplanar to true at proper level regardless.
    * @param settings
    */
   public void subdivideFast() {
		if(DEBUGSUBDIVIDE ) {
			System.out.println("octree_t subdivideFast...level="+m_level+" indicies="+m_indexes.size()+" centoid="+m_centroid);
		}
   // s_ms verification, obviously the value needs to be > 0 and low values seem to want to blow the stack
	if (m_indexes.size() < (int)hough_settings.s_ms) 
	    return;
   // s_level verification
	if(DEBUGSUBDIVIDE) {
		System.out.println("octree_t subdivideFast s_level verification in octree..."+(m_level >=hough_settings.s_level)+", "+m_level+" "+hough_settings.s_level);
	}
   if (m_level >= hough_settings.s_level) {
    		  coplanar = true;
    		  return;
   }
   m_children = new octree_t[8];
   double newsize = m_size/2.0;
   for (int i = 0; i < 8 ; i++) {
	  m_children[i] = new octree_t();
      m_children[i].m_size = newsize;
      m_children[i].m_level = (short) (m_level+1);
      m_children[i].m_root = m_root;
      m_children[i].m_parent = this;
      m_children[i].m_indexes.ensureCapacity(m_indexes.size()/4);
   }
   double size4 = m_size/4.0;
   // Calculation of son nodes
   m_children[0].m_middle.x = m_middle.x - size4;
   m_children[1].m_middle.x = m_middle.x - size4;
   m_children[2].m_middle.x = m_middle.x - size4;
   m_children[3].m_middle.x = m_middle.x - size4;
   m_children[4].m_middle.x = m_middle.x + size4;
   m_children[5].m_middle.x = m_middle.x + size4;
   m_children[6].m_middle.x = m_middle.x + size4;
   m_children[7].m_middle.x = m_middle.x + size4;

   m_children[0].m_middle.y = m_middle.y - size4;
   m_children[1].m_middle.y = m_middle.y - size4;
   m_children[2].m_middle.y = m_middle.y + size4;
   m_children[3].m_middle.y = m_middle.y + size4;
   m_children[4].m_middle.y = m_middle.y - size4;
   m_children[5].m_middle.y = m_middle.y - size4;
   m_children[6].m_middle.y = m_middle.y + size4;
   m_children[7].m_middle.y = m_middle.y + size4;

   m_children[0].m_middle.z = m_middle.z - size4;
   m_children[1].m_middle.z = m_middle.z + size4;
   m_children[2].m_middle.z = m_middle.z - size4;
   m_children[3].m_middle.z = m_middle.z + size4;
   m_children[4].m_middle.z = m_middle.z - size4;
   m_children[5].m_middle.z = m_middle.z + size4;
   m_children[6].m_middle.z = m_middle.z - size4;
   m_children[7].m_middle.z = m_middle.z + size4;

   // putting points in its respective children
   for (int i = 0; i < m_indexes.size() ; i++) {
      int index = 0;
      if (m_root.m_points.get(m_indexes.get(i)).x > m_middle.x) {
         index+=4;
      }
      if (m_root.m_points.get(m_indexes.get(i)).y > m_middle.y)
      {
         index+=2;
      }
      if (m_root.m_points.get(m_indexes.get(i)).z > m_middle.z)
      {
         index+=1;
      }
      m_children[index].m_indexes.add(m_indexes.get(i));
      // Calculating centroid distribution (divided by the number of points below)
      m_children[index].m_centroid = m_children[index].m_centroid.add(m_root.m_points.get(m_indexes.get(i)));
   }
   
   for (int i = 0; i < 8 ; i++) {
      m_children[i].m_centroid = m_children[i].m_centroid.divide(m_children[i].m_indexes.size());
      // Recursive subdivision 
      m_children[i].subdivideFast();
   }
  }
  /**
   * For each point in this node, Subtract the centroid from the passed point and take the vector4d scalar dot product of that
   * and the normalized 'normal1' vector, which happens in distance2plane, 
   * then compare the absolute value of that to m_size/10. If its greater
   * remove this point index. this presupposes we have called leaset_variance_direction and have done the PCA.
   */
   private void remove_outliers() {
    Vector4d centroid = new Vector4d();
    int origSize = m_indexes.size();
    for(int i = m_indexes.size()-1; i >=0 ; i--) {
      double dp = distance2plane(m_root.m_points.get(m_indexes.get(i)));
      if( dp > (m_size/hough_settings.max_distance2plane)) {
    		if( DEBUG) {
    			System.out.println("octree_t remove_outliers removing..."+m_indexes.get(i)+" "+m_root.m_points.get(m_indexes.get(i))+" "+dp+" > "+(m_size/hough_settings.max_distance2plane));
    		}
         m_indexes.remove(i);
      } else {
         centroid = centroid.add(m_root.m_points.get(m_indexes.get(i)));
      }
    }
    if (m_indexes.size() > 0) {
      m_centroid = centroid.divide(m_indexes.size());
    } else
    	if( DEBUG) {
			System.out.println("octree_t remove_outliers: ***m_indexes has ZERO entries...");
		}
    if( DEBUG ) {
    	System.out.println("octree_t remove_outliers: original m_indexes size="+origSize+" new size="+m_indexes.size());
    }
   }
   /**
    * Generate 3D fast covariance matrix for each point in this node.
    * @return
    */
   private Matrix3 fast_covariance_matrix() {
     int nverts = m_indexes.size();
     double nvertsd = (double)(nverts);
     Matrix3 covariance = new Matrix3();
     covariance.set(0,0, 0.0);
 		if( DEBUGVARIANCE) {
 			System.out.println("octree_t fast_covariance_matrix verticies="+nverts+" centroid="+m_centroid);
 			//for(int k = 0; k < nverts; k++) {
 			//	System.out.println("index="+k+": "+(m_root.m_points.get(m_indexes.get(k))));
 			//}
 		}
 		for (int k = 0; k < nverts; k++)
 			covariance.set(0,0, covariance.get(0,0) + 
    		 (m_root.m_points.get(m_indexes.get(k)).get(0) - m_centroid.get(0)) * 
    		 (m_root.m_points.get(m_indexes.get(k)).get(0) - m_centroid.get(0)));
 		covariance.set(0,0, covariance.get(0, 0) / nvertsd);
 		if(Math.abs(m_covariance.get(0,0)) < EPS)
 			m_covariance.set(0,0,0.0);
 		covariance.set(1,1, 0.0);
 		for(int k = 0; k < nverts; k++)
 			covariance.set(1,1, covariance.get(1,1) + 
    	 		 (m_root.m_points.get(m_indexes.get(k)).get(1) - m_centroid.get(1)) * 
        		 (m_root.m_points.get(m_indexes.get(k)).get(1) - m_centroid.get(1)));
 		covariance.set(1,1, covariance.get(1,1) / nvertsd);
 		if(Math.abs(m_covariance.get(1,1)) < EPS)
 			m_covariance.set(1,1, 0.0);
 		covariance.set(2,2, 0.0);
 		for(int k = 0; k < nverts; k++)
 			covariance.set(2,2, covariance.get(2,2) +
    	 		 (m_root.m_points.get(m_indexes.get(k)).get(2) - m_centroid.get(2)) * 
        		 (m_root.m_points.get(m_indexes.get(k)).get(2) - m_centroid.get(2)));
 		covariance.set(2,2, covariance.get(2,2) / nvertsd);
 		if(Math.abs(m_covariance.get(2,2)) < EPS)
 			m_covariance.set(2,2, 0.0);
 		covariance.set(1,0, 0.0);
 		for(int k = 0; k < nverts; k++)
 			covariance.set(1,0, covariance.get(1, 0) +
    	 		 (m_root.m_points.get(m_indexes.get(k)).get(1) - m_centroid.get(1)) * 
        		 (m_root.m_points.get(m_indexes.get(k)).get(0) - m_centroid.get(0)));
    		  //(m_root.m_points[m_indexes[k]][1] - m_centroid[1]) * (m_root.m_points[m_indexes[k]][0] - m_centroid[0]);
 		covariance.set(1,0, covariance.get(1,0) / nvertsd);
 		if(Math.abs(m_covariance.get(1,0)) < EPS)
 			m_covariance.set(1,0,0.0);
 		covariance.set(2,0,0.0);
 		for(int k = 0; k < nverts; k++)
 			covariance.set(2,0, covariance.get(2, 0) +
    	 		 (m_root.m_points.get(m_indexes.get(k)).get(2) - m_centroid.get(2)) * 
        		 (m_root.m_points.get(m_indexes.get(k)).get(0) - m_centroid.get(0)));
    		  //(m_root.m_points[m_indexes[k]][2] - m_centroid[2]) * (m_root.m_points[m_indexes[k]][0] - m_centroid[0]);
 		covariance.set(2,0, covariance.get(2,0) / nvertsd);
 		if(Math.abs(m_covariance.get(2,0)) < EPS)
 			m_covariance.set(2,0,0.0);
 		covariance.set(2,1,0.0);
 		for(int k = 0; k < nverts; k++)
 			covariance.set(2,1, covariance.get(2,1) +
    	 		 (m_root.m_points.get(m_indexes.get(k)).get(2) - m_centroid.get(2)) * 
        		 (m_root.m_points.get(m_indexes.get(k)).get(1) - m_centroid.get(1)));
    		  //(m_root.m_points[m_indexes[k]][2] - m_centroid[2]) * (m_root.m_points[m_indexes[k]][1] - m_centroid[1]);
 		covariance.set(2,1, covariance.get(2, 1) / nvertsd);
 		if(Math.abs(m_covariance.get(2,1)) < EPS)
 			m_covariance.set(2,1,0.0);
 		covariance.set(0,2, covariance.get(2,0));
 		covariance.set(0,1, covariance.get(1,0));
 		covariance.set(1,2, covariance.get(2,1));
 		return covariance;
   }
   /**
    * Principal component analysis.
    * Since the eigenvalues of the covariance matrix associated to the set of
	* samples inside an octree node represent the proportions of the variances
	* of the sample distribution inside that cell, they can be used to filter out
	* clusters that could not represent planes.
	* when it is all said and done we have set the values of variance1,variance2,and variance3 along with
	* normal1, normal2, and normal3.
    */
    private void least_variance_direction(){
		if( DEBUGVARIANCE) {
			System.out.println("octree_t least_variance_direction...computing covariance, eigenvalues and eigenvectors");
		}
		m_covariance = fast_covariance_matrix();
		EigenvalueDecomposition eigenvalue_decomp = new EigenvalueDecomposition(m_covariance);
		double[] eigenvalues_vector = eigenvalue_decomp.getRealEigenvalues();
		int min_index = 0, max_index = 0, middle_index = 0;
		if(eigenvalues_vector[1] < eigenvalues_vector[min_index]) {
			min_index = 1;
		} else 
			if (eigenvalues_vector[1] > eigenvalues_vector[max_index]) {
				max_index = 1;
			}
		if(eigenvalues_vector[2] < eigenvalues_vector[min_index]) {
			min_index = 2;
		} else
			if (eigenvalues_vector[2] > eigenvalues_vector[max_index]) {
				max_index = 2;
			}
		while (middle_index==min_index || middle_index==max_index)
			middle_index++;

		variance1 = eigenvalues_vector[min_index];
		variance2 = eigenvalues_vector[middle_index];
		variance3 = eigenvalues_vector[max_index];
		if( DEBUGVARIANCE) {
			System.out.println("octree_t least_variance_direction...variance1="+variance1+" variance2="+variance2+" variance3="+variance3);
		}
		Matrix3 eigenvectors_matrix = eigenvalue_decomp.getV();

		normal1 = new Vector4d(eigenvectors_matrix.get(0, min_index),eigenvectors_matrix.get(1, min_index),eigenvectors_matrix.get(2, min_index));
		normal2 = new Vector4d(eigenvectors_matrix.get(0, middle_index), eigenvectors_matrix.get(1, middle_index), eigenvectors_matrix.get(2, middle_index));
		normal3 = new Vector4d(eigenvectors_matrix.get(0, max_index), eigenvectors_matrix.get(1, max_index), eigenvectors_matrix.get(2, max_index));
		if( DEBUGVARIANCE) {
			System.out.println("octree_t least_variance_direction...eigenvector normal1="+normal1+" normal2="+normal2+" normal3="+normal3);
		}
    }
    /**
     * Subtract the centroid from the passed point and take the vector4d scalar dot product of that
     * and the normalized 'normal1' vector.
     * then return the absolute value of that.
     * @param point
     * @return
     */
    double distance2plane( Vector4d point ){
    	return Math.abs(point.subtract(m_centroid).and(normal1.Normalized()));
    }
    /**
     * Convert the spherical params to cartesian coords in 3D.
     * @param normal Vector to hold result of transform
     * @param theta theta value to convert normal.x=sin(phi)*cos(theta)*rho
     * @param phi phi to convert normal.y=sin(phi) *sin(theta) * rho
     * @param rho rho to convert normal.z=cos(phi) * rho
     */
    public static void spherical_to_cartesian(Vector4d normal, double theta, double phi, double rho){
       normal.x = Math.sin(phi) * Math.cos(theta) * rho;
       normal.y = Math.sin(phi) * Math.sin(theta) * rho;
       normal.z = Math.cos(phi) * rho;
    }
    /**
     * Convert the spherical params to cartesian coords in 3D origin at given center, at given scale.
     * @param normal Vector to hold result of transform
     * @param center center point origin of new vector
     * @param theta theta value to convert normal.x= center.x + sin(phi)*cos(theta)*rho+scale
     * @param phi phi to convert normal.y= center.y + sin(phi) *sin(theta) * rho+scale
     * @param rho rho to convert normal.z= center.z + cos(phi) * rho+scale
     * @param scale scale factor of final vector
     */
    public static void spherical_to_cartesian(Vector4d normal, Vector4d center, 
    		double theta, double phi, double rho, double scale){
        normal.x = center.x + (Math.sin(phi) * Math.cos(theta) * (rho+scale));
        normal.y = center.y + (Math.sin(phi) * Math.sin(theta) * (rho+scale));
        normal.z = center.z + (Math.cos(phi) * (rho+scale));
     }
    /**
     * 
     * @param point
     * @return theta, phi, rho double array. rho is defined as vector magnitude via point,getLength()
     */
    public static double[] cartesian_to_spherical(Vector4d point) {
 	   double rho = point.getLength();
 	   double theta = Math.acos(point.z/rho); // betw -1 and 1
 	   double phi = Math.atan2(point.y,point.x);
 	   return new double[]{theta, phi, rho};
    }
    /**
     * If child nodes are not present, add 'this' to the passed node array if this node is marked 'coplanar=true'
     * otherwise recursively perform the operation 
     * @param nodes the array to be filled
     */
    public void get_nodes( ArrayList<octree_t> nodes ) {
    	if (m_children != null) {
    		for (short i = 0; i < 8 ; i++) {
    			m_children[i].get_nodes(nodes);
    		}
    	} else {
    		if (coplanar) {
    			nodes.add(this);
    		}
    	}
    }
    /**
     * If child nodes are not present, add 'this' to the passed node array if this node is marked 'coplanar=true'
     * otherwise recursively perform the operation starting from this
     * @return the nodes arraylist
     */
    public ArrayList<octree_t> get_nodes() {
    	ArrayList<octree_t> outn = new ArrayList<octree_t>();
    	get_nodes(outn);
    	return outn;
    }
    /**
     * Start octree build process
     * @param node father
     */
    public static void buildStart(octree_t node) {
    	node.point_num = 0;
    	node.m_points = new ArrayList<Vector4d>();
    	node.m_colors = new ArrayList<Vector4d>();
    	node.m_root = node;
    }
    /**
     * Call to add points to octree
     * @param node father
     * @param x
     * @param y
     * @param z
     * @param r
     * @param g
     * @param b
     */
    public static void build(octree_t node, double x, double y, double z, double r, double g, double b) {
		Vector4d point = new Vector4d(x,y,z);
		Vector4d color = new Vector4d(r,g,b);
		node.m_points.add(point);
		node.m_colors.add(color);
		node.m_centroid = node.m_centroid.add(point); // set up to average all points on all axis
		node.m_indexes.add(node.point_num++);
		node.mix = Math.min(node.mix,point.x);
		node.miy = Math.min(node.miy,point.y);
		node.miz = Math.min(node.miz,point.z);
		node.max = Math.max(node.max,point.x);
		node.may = Math.max(node.may,point.y);
		node.maz = Math.max(node.maz,point.z);
    }
    /**
     * Finish octree build
     * @param node father
     */
    public static void buildEnd(octree_t node) {
    	node.max_distance = 0.0;
    	node.m_centroid = node.m_centroid.divide(node.point_num);
    	node.m_middle.x = (node.mix+((node.max-node.mix)/2));
    	node.m_middle.y = (node.miy+((node.may-node.miy)/2));
    	node.m_middle.z = (node.miz+((node.maz-node.miz)/2));
    	// establish farthest distance between centroid and any point on any axis.
    	for(Vector4d  vx : node.m_points) {
    		// v = v.subtract(centroid);
    		Vector4d v = vx.subtract(node.m_centroid);
    		node.max_distance = Math.max(node.max_distance,Math.abs(v.x));
    		node.max_distance = Math.max(node.max_distance,Math.abs(v.y));
    		node.max_distance = Math.max(node.max_distance,Math.abs(v.z));
    		// length is vector magnitude from origin
    		hough_settings.max_point_distance = Math.max(hough_settings.max_point_distance,v.getLength());
    	}
    	if( DEBUG )
    		System.out.println("octree centroid="+node.m_centroid+" max vector span="+hough_settings.max_point_distance);
    	node.m_size = node.max_distance * 2.0;
    }
    
  	@Override
  	public String toString() {
	   return "octree_t centroid="+m_centroid+" level="+m_level+" size="+m_size+" points="+m_indexes.size()+" coplanar="+coplanar+" normal1="+normal1+" normal2="+normal2+" normal3="+normal3+" variance1="+variance1+" variance2="+variance2+" variance3="+variance3;
  	}
  	//@Override 
  	//public boolean equals(Object onode) {
  	//   return ((octree_t)onode).octoNum == octoNum;
  	//}
/*
void octree_t::print_points()
{
   glPointSize(4.0);
   glBegin(GL_POINTS);
   glNormal3d(1.0,0.0,0.0);
   glColor3dv(color);
   for (size_t i = 0; i < m_indexes.size() ; i++)
   {
      glVertex3d(m_root->m_points[m_indexes[i]].x, m_root->m_points[m_indexes[i]].y, m_root->m_points[m_indexes[i]].z);
   }
   glEnd();
   glPointSize(1.0);
}

void octree_t::show( bool type, int height )
{
   height--;
   if (height == 0) {
      glColor3dv(color);
      if (type) {
         glPushMatrix();
         glTranslated(m_middle.x, m_middle.y, m_middle.z);
         if (color == Vector4d(0.5,0.5,0.5)) {
            glutWireCube(m_size);
         } else {
            glutSolidCube(m_size);
         }
         glPopMatrix();
      } else {
         print_points();
      }
   }
   if (m_children != NULL) 
   {
      for (short i = 0; i < 8 ; i++)
      {
         m_children[i].show(type, height);
      }
   }
}

void octree_t::show( bool type)
{
   if (coplanar) {
         glColor3dv(color);
         if (type) {
            glPushMatrix();
            glTranslated(m_middle.x, m_middle.y, m_middle.z);
            
            glutWireCube(m_size);
            //if (color != Vector4d(0.5,0.5,0.5)) 
            //   glutWireCube(m_size);
            //else {
            //   glutSolidCube(m_size);
            //}
            
         
            //glutSolidCube(m_size);
            glPopMatrix();
         } else {
            print_points();
         }
      }

   if (m_children != NULL) 
   {
      for (short i = 0; i < 8 ; i++)
      {
         m_children[i].show(type);
      }
   }
}*/


}

