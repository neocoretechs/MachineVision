package com.neocoretechs.machinevision.hough3d;

import java.util.ArrayList;
//#include "Thirdparty/SwissArmyKnife/Mathematic.h"
//#include "Thirdparty/SwissArmyKnife/Vector4d.h"
//#include "Thirdparty/SwissArmyKnife/Matrix4d.h"
//#include "GL/glut.h"
//#include "octree_t.h"

public class plane_t {
   double ti, m_rotate;
   int pi,ri;
   boolean m_showing;
   
   ArrayList<octree_t> nodes;
   float votes;
   double representativeness;
   double m_theta;
   double m_phi;
   double m_rho;

   Vector4d m_cross;
   Vector4d m_cross2;
   Vector4d m_position;
   Vector4d m_centroid;
   Vector4d m_normal;
   Vector4d m_desloc;
   Vector4d m_scale;
   Vector4d m_color;

   ArrayList<Vector4d> m_points;
   public plane_t() {}

   void calculate() {     
      m_normal.x = Math.sin(m_phi) * Math.cos(m_theta);
      m_normal.y = Math.sin(m_phi) * Math.sin(m_theta);
      m_normal.z = Math.cos(m_phi);
      m_position = m_normal.multiply(m_rho);
      Vector4d c_u = new Vector4d(0.0,0.0,1.0);
      if (c_u.equals(m_normal))
         c_u = new Vector4d(1.0,0.0,0.0);
      m_cross = (c_u.multiply(m_normal)).Normalized();
      m_cross2 = m_normal.multiply(m_cross);
      m_centroid = new Vector4d(0,0,0);
      m_scale = new Vector4d(0,0,0,0);
      m_showing = true;
      m_rotate = 0.0;
      int cont = 0;
	  for (int i = 0; i < nodes.size(); i++) {
         cont += nodes.get(i).m_indexes.size();
		 for (int j = 0; j < nodes.get(i).m_indexes.size(); j++) {
         	m_centroid.add( nodes.get(i).m_root.m_points.get(nodes.get(i).m_indexes.get(j)) );
         }
      }
      if (cont != 0)
         m_centroid.divide((double)cont);
      m_desloc = m_centroid.subtract(m_position);
      if (m_desloc.getLength() != 0) {
         Vector4d normproj = m_normal.multiply(m_desloc.getLength()).multiply(m_desloc.Normalized().and(m_normal));
         m_desloc.subtract(normproj);
      }
   }
   
   boolean isLessthan(plane_t p) { return (representativeness < p.representativeness); }
/*
   inline void draw(double size, bool type, bool pc, bool selected) 
   {

      if (selected) {
         glColor3d(0.0,1.0,0.0);
         glPointSize(6.0);
      }
      else { 
         glColor3d(0.0,0.0,0.5);
         glPointSize(5.0);
      }

      if (pc) {
         if (type) {
            for (octree_t *node : nodes)
            {
               glPushMatrix();
               glTranslated(node->m_middle.x, node->m_middle.y, node->m_middle.z);
               glutSolidCube(node->m_size+0.5);
               glPopMatrix();
            }
         } else {
            for (octree_t *node : nodes)
            {
               glBegin(GL_POINTS);
               for (size_t i = 0; i < node->m_indexes.size() ; i++)
               {
                  glVertex3d(node->m_root->m_points[node->m_indexes[i]].x, node->m_root->m_points[node->m_indexes[i]].y, node->m_root->m_points[node->m_indexes[i]].z);
               }
               glEnd();
            }
         }
      }
      
      if (m_showing) {
         glPushMatrix();
         {
            glBegin(GL_QUADS);
            if (selected)
               glColor4d(0.0,0.0,0.0,0.6);
            else
               glColor4d(m_color.r,m_color.g,m_color.b,0.4);

            Vector4d cross = Matrix4d::Rotation(m_rotate,m_normal) * m_cross;
            Vector4d cross2 = Matrix4d::Rotation(m_rotate,m_normal) * m_cross2;

            glNormal3dv(m_normal);
            glVertex3dv((m_position + cross * size + cross2 * size + m_desloc + cross * m_scale.x + cross2 * m_scale.z).data);
            glVertex3dv((m_position + cross * size - cross2 * size + m_desloc + cross * m_scale.x + cross2 * m_scale.w).data);
            glVertex3dv((m_position - cross * size - cross2 * size + m_desloc + cross * m_scale.y + cross2 * m_scale.w).data);
            glVertex3dv((m_position - cross * size + cross2 * size + m_desloc + cross * m_scale.y + cross2 * m_scale.z).data);
            
            glEnd();
         }
         glPopMatrix();
      }
   }
*/
   double distance2plane( Vector4d point ) {
	return Math.abs((point.subtract(m_position)).and(m_normal.Normalized()));
   }

}
