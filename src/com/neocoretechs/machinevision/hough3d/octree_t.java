package com.neocoretechs.machinevision.hough3d;

import java.util.ArrayList;

//#include "Thirdparty/SwissArmyKnife/Mathematic.h"
//#include "Thirdparty/SwissArmyKnife/Vector4d.h"
//#include "Thirdparty/MITIE/dlib/dlib/matrix.h"
//#include "settings.h"
//#include "GL/glut.h"
//#include <omp.h>


public class octree_t {
   public static final double EPS = 1.E-3;
   Matrix3 fast_covariance_matrix = new Matrix3();
   Matrix3 m_covariance = new Matrix3();
   ArrayList<Vector4d> m_points = new ArrayList<Vector4d>();
   ArrayList<Vector4d> m_colors= new ArrayList<Vector4d>();
   ArrayList<Integer> m_indexes= new ArrayList<Integer>();
   octree_t[] m_children = null;
   octree_t m_root;
   Vector4d normal1, normal2, normal3;
   Vector4d m_middle, m_centroid, color;
   double variance1, variance2, variance3;
   double m_size, representativeness;
   short m_level;
   boolean coplanar;
   int votes;
   
   public octree_t() {
     coplanar = false;
     m_centroid = new Vector4d(0,0,0,0);
     color = new Vector4d(0.5,0.5,0.5);
     variance1 = variance2 = variance3 = 0.0;
     votes= 0;
  }

   void clear() {
   if (m_children != null) {
      for(int i = 0; i < 8 ; i++) {
         m_children[i].m_indexes.clear();
         m_children[i].clear();
      }
      m_children = null;
   }
  }

  void subdivide( hough_settings settings ) {
   // s_ms verification
	if (m_indexes.size() < (int)settings.s_ms) 
	    return;

   // s_level verification
   if (m_level >= settings.s_level) {
      // principal component analysis
      least_variance_direction();
      // Planarity verification
      double thickness = variance1 / variance2;
      double isotropy  = variance2 / variance3;
      if (thickness < settings.max_thickness && isotropy > settings.min_isotropy) {
         // Refitting step
         remove_outliers();
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
      m_children[index].m_centroid.add(m_root.m_points.get(m_indexes.get(i)));
   }
   
   for (int i = 0; i < 8 ; i++) {
      m_children[i].m_centroid.divide(m_children[i].m_indexes.size());
      // Recursive subdivision 
      m_children[i].subdivide(settings);
   }
  }

   void remove_outliers() {
    Vector4d centroid = new Vector4d();
    for (int i = m_indexes.size()-1; i >=0 ; i--) {
      if (distance2plane(m_root.m_points.get(m_indexes.get(i))) > m_size/10.0) {
         m_indexes.remove(m_indexes.get(0).intValue()+i);
      } else {
         centroid.add(m_root.m_points.get(m_indexes.get(i)));
      }
    }
    if (m_indexes.size() > 0)
      m_centroid = centroid.divide(m_indexes.size());
   }

   Matrix3 fast_covariance_matrix() {
     int nverts = m_indexes.size();
     double nvertsd = (double)(nverts);
     Matrix3 covariance = new Matrix3();
     covariance.set(0,0, 0.0);
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

  void least_variance_direction(){
   m_covariance = fast_covariance_matrix();
   EigenvalueDecomposition eigenvalue_decomp = new EigenvalueDecomposition(m_covariance);
   //dlib::
   double[] eigenvalues_vector = eigenvalue_decomp.getRealEigenvalues();
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

   Matrix3 eigenvectors_matrix = eigenvalue_decomp.getV();

   normal1 = new Vector4d(eigenvectors_matrix.get(0, min_index),eigenvectors_matrix.get(1, min_index),eigenvectors_matrix.get(2, min_index));
   normal2 = new Vector4d(eigenvectors_matrix.get(0, middle_index), eigenvectors_matrix.get(1, middle_index), eigenvectors_matrix.get(2, middle_index));
   normal3 = new Vector4d(eigenvectors_matrix.get(0, max_index), eigenvectors_matrix.get(1, max_index), eigenvectors_matrix.get(2, max_index));
  }

  double distance2plane( Vector4d point ){
   return Math.abs(point.subtract(m_centroid).and(normal1.Normalized()));
  }

  void get_nodes( ArrayList<octree_t> nodes ) {
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

