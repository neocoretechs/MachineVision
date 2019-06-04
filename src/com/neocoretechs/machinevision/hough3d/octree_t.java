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
   //private static long octreeNum = 0;
   //protected long octoNum;
   Matrix3 fast_covariance_matrix = new Matrix3();
   Matrix3 m_covariance = new Matrix3();
   ArrayList<Vector4d> m_points = null;//new ArrayList<Vector4d>();
   ArrayList<Vector4d> m_colors= new ArrayList<Vector4d>();
   ArrayList<Integer> m_indexes= new ArrayList<Integer>(); // points to m_points in root node from subnodes
   octree_t[] m_children = null;
   octree_t m_root;
   Vector4d normal1, normal2, normal3;
   Vector4d m_middle = new Vector4d();
   Vector4d m_centroid, color;
   double variance1, variance2, variance3;
   double m_size, representativeness;
   short m_level;
   boolean coplanar;
   int votes;
   private boolean DEBUG = true;
   private boolean DEBUGVARIANCE = false;
   private boolean DEBUGSUBDIVIDE = false;
   /**
    * Default c'tor sets coplanar false, creates a centroid at 0,0,0 and sets color.
    */
   public octree_t() {
     coplanar = false;
     m_centroid = new Vector4d(0,0,0,0);
     color = new Vector4d(0.5,0.5,0.5);
     variance1 = variance2 = variance3 = 0.0;
     votes= 0;
     //++octreeNum; // increment monotonically increasing number to identify this node for comparison
     //octoNum = octreeNum;
  }
   /**
    * Special root node c'tor sets coplanar false, creates a centroid at 0,0,0 and sets color.
    * Also sets m_points to new arraylist of vector4d to hold data points.
    */
   public octree_t(boolean root) {
	 if(root)
		  m_points = new ArrayList<Vector4d>();
     coplanar = false;
     m_centroid = new Vector4d(0,0,0,0);
     color = new Vector4d(0.5,0.5,0.5);
     variance1 = variance2 = variance3 = 0.0;
     votes= 0;
     //++octreeNum; // increment monotonically increasing number to identify this node for comparison
     //octoNum = octreeNum;
  }
   /**
    * 
    */
   void clear() {
   if (m_children != null) {
      for(int i = 0; i < 8 ; i++) {
         m_children[i].m_indexes.clear();
         m_children[i].clear();
      }
      m_children = null;
   }
  }
   /**
    * Subdivide an octree node.
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
  void subdivide( hough_settings settings ) {
		if(DEBUGSUBDIVIDE ) {
			System.out.println("octree subdivide...level="+m_level+" indicies="+m_indexes.size()+" centoid="+m_centroid);
		}
   // s_ms verification, obviously the value needs to be > 0 and low values seem to want to blow the stack
	if (m_indexes.size() < (int)settings.s_ms) 
	    return;
   // s_level verification
	if(DEBUGSUBDIVIDE) {
		System.out.println("octree subdivide s_level verification in octree..."+(m_level >=settings.s_level)+", "+m_level+" "+settings.s_level);
	}
   if (m_level >= settings.s_level) {
      // principal component analysis
      least_variance_direction();
      // Planarity verification
      double thickness = variance1 / variance2;
      double isotropy  = variance2 / variance3;
      if( DEBUG )
    	  System.out.println("thickness and isotropy="+thickness+" "+isotropy);
      if (thickness < settings.max_thickness && isotropy > settings.min_isotropy) {
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
      m_children[i].subdivide(settings);
   }
  }
  /**
   * For each point in this node, Subtract the centroid from the passed point and take the vector4d scalar dot product of that
   * and the normalized 'normal1' vector, then compare the absolute value of that to m_size/10. If its greater
   * remove this point index.
   */
   private void remove_outliers() {
    Vector4d centroid = new Vector4d();
    int origSize = m_indexes.size();
    for(int i = m_indexes.size()-1; i >=0 ; i--) {
      double dp = distance2plane(m_root.m_points.get(m_indexes.get(i)));
      if( dp > (m_size/10.0)) {
    		if( DEBUG) {
    			System.out.println("octree remove_outliers removing..."+m_indexes.get(i)+" "+m_root.m_points.get(m_indexes.get(i))+" "+dp+" > "+(m_size/10.0));
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
			System.out.println("octree remove_outliers: ***m_indexes has ZERO entries...");
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
		System.out.println("octree fast_covariance_matrix verticies="+nverts+" centroid="+m_centroid);
		//for(int k = 0; k < nverts; k++) {
		//	System.out.println("index="+k+": "+(m_root.m_points.get(m_indexes.get(k))));
		//}
	}
   for (int k = 0; k < nverts; k++)
     covariance.set(0,0, covariance.get(0,0) + 
    		 (m_root.m_points.get(m_indexes.get(k)).get(0) - m_centroid.get(0)) * 
    		 (m_root.m_points.get(m_indexes.get(k)).get(0) - m_centroid.get(0)));
   covariance.set(0,0, covariance.get(0, 0) / nvertsd);
   if (Math.abs(m_covariance.get(0,0)) < EPS)
      m_covariance.set(0,0,0.0);
   covariance.set(1,1, 0.0);
   for (int k = 0; k < nverts; k++)
      covariance.set(1,1, covariance.get(1,1) + 
    	 		 (m_root.m_points.get(m_indexes.get(k)).get(0) - m_centroid.get(0)) * 
        		 (m_root.m_points.get(m_indexes.get(k)).get(0) - m_centroid.get(0)));
   covariance.set(1,1, covariance.get(1,1) / nvertsd);
   if (Math.abs(m_covariance.get(1,1)) < EPS)
      m_covariance.set(1,1, 0.0);
   covariance.set(2,2, 0.0);
   for (int k = 0; k < nverts; k++)
      covariance.set(2,2, covariance.get(2,2) +
    	 		 (m_root.m_points.get(m_indexes.get(k)).get(2) - m_centroid.get(2)) * 
        		 (m_root.m_points.get(m_indexes.get(k)).get(2) - m_centroid.get(2)));
   covariance.set(2,2, covariance.get(2,2) / nvertsd);
   if (Math.abs(m_covariance.get(2,2)) < EPS)
      m_covariance.set(2,2, 0.0);
   covariance.set(1,0, 0.0);
   for (int k = 0; k < nverts; k++)
      covariance.set(1,0, covariance.get(1, 0) +
    	 		 (m_root.m_points.get(m_indexes.get(k)).get(1) - m_centroid.get(1)) * 
        		 (m_root.m_points.get(m_indexes.get(k)).get(0) - m_centroid.get(0)));
    		  //(m_root.m_points[m_indexes[k]][1] - m_centroid[1]) * (m_root.m_points[m_indexes[k]][0] - m_centroid[0]);
   covariance.set(1,0, covariance.get(1,0) / nvertsd);
   if (Math.abs(m_covariance.get(1,0)) < EPS)
      m_covariance.set(1,0,0.0);
   covariance.set(2,0,0.0);
   for (int k = 0; k < nverts; k++)
      covariance.set(2,0, covariance.get(2, 0) +
    	 		 (m_root.m_points.get(m_indexes.get(k)).get(2) - m_centroid.get(2)) * 
        		 (m_root.m_points.get(m_indexes.get(k)).get(0) - m_centroid.get(0)));
    		  //(m_root.m_points[m_indexes[k]][2] - m_centroid[2]) * (m_root.m_points[m_indexes[k]][0] - m_centroid[0]);
   covariance.set(2,0, covariance.get(2,2) / nvertsd);
   if (Math.abs(m_covariance.get(2,0)) < EPS)
      m_covariance.set(2,0,0.0);
   covariance.set(2,1,0.0);
   for (int k = 0; k < nverts; k++)
      covariance.set(2,1, covariance.get(2,1) +
    	 		 (m_root.m_points.get(m_indexes.get(k)).get(2) - m_centroid.get(2)) * 
        		 (m_root.m_points.get(m_indexes.get(k)).get(1) - m_centroid.get(1)));
    		  //(m_root.m_points[m_indexes[k]][2] - m_centroid[2]) * (m_root.m_points[m_indexes[k]][1] - m_centroid[1]);
   covariance.set(2,1, covariance.get(2, 1) / nvertsd);
   if (Math.abs(m_covariance.get(2,1)) < EPS)
      m_covariance.set(2,1,0.0);
   covariance.set(0,2, covariance.get(2,0));
   covariance.set(0,1, covariance.get(1,0));
   covariance.set(1,2, covariance.get(2,1));
   return covariance;
  }
   /**
    * Principle component analysis
    */
  private void least_variance_direction(){
		if( DEBUGVARIANCE) {
			System.out.println("octree least_variance_direction...computing covariance");
		}
   m_covariance = fast_covariance_matrix();
	if( DEBUGVARIANCE ) {
		System.out.println("octree least_variance_direction...eigenvalue decomp:");//\r\n"+m_covariance);
	}
   EigenvalueDecomposition eigenvalue_decomp = new EigenvalueDecomposition(m_covariance);
	if( DEBUGVARIANCE) {
		System.out.println("octree least_variance_direction...get real eigenvectors");
	}
   double[] eigenvalues_vector = eigenvalue_decomp.getRealEigenvalues();
	if( DEBUGVARIANCE) {
		System.out.println("octree least_variance_direction...get eigenvalues");
	} 
   int min_index = 0, max_index = 0, middle_index = 0;
   if (eigenvalues_vector[1] < eigenvalues_vector[min_index]) {
      min_index = 1;
   } else if (eigenvalues_vector[1] > eigenvalues_vector[max_index]) {
      max_index = 1;
   }
   if (eigenvalues_vector[2] < eigenvalues_vector[min_index]) {
      min_index = 2;
   } else if (eigenvalues_vector[2] > eigenvalues_vector[max_index]) {
      max_index = 2;
   }

   while (middle_index==min_index || middle_index==max_index) middle_index++;

   variance1 = eigenvalues_vector[min_index];
   variance2 = eigenvalues_vector[middle_index];
   variance3 = eigenvalues_vector[max_index];
	if( DEBUGVARIANCE) {
		System.out.println("octree least_variance_direction...");//variance1="+variance1+" variance2="+variance2+" variance3="+variance3);
	}
   Matrix3 eigenvectors_matrix = eigenvalue_decomp.getV();

   normal1 = new Vector4d(eigenvectors_matrix.get(0, min_index),eigenvectors_matrix.get(1, min_index),eigenvectors_matrix.get(2, min_index));
   normal2 = new Vector4d(eigenvectors_matrix.get(0, middle_index), eigenvectors_matrix.get(1, middle_index), eigenvectors_matrix.get(2, middle_index));
   normal3 = new Vector4d(eigenvectors_matrix.get(0, max_index), eigenvectors_matrix.get(1, max_index), eigenvectors_matrix.get(2, max_index));
	if( DEBUGVARIANCE) {
		System.out.println("octree least_variance_direction...");//eigenvector normal1="+normal1+" normal2="+normal2+" normal3="+normal3);
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
   * If child nodes are not present, add 'this' to the passed node array if this node is marked 'coplanar=true'
   * otherwise recursively perform the operation 
   * @param nodes
   */
  protected void get_nodes( ArrayList<octree_t> nodes ) {
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
  
  @Override
  public String toString() {
	   return "octree_t centroid="+m_centroid+" level="+m_level+" size="+m_size+" points="+m_indexes.size()+" coplanar="+coplanar+" votes="+votes+" representativeness="+representativeness;
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

