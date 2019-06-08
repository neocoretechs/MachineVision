package com.neocoretechs.machinevision.hough3d;
import java.util.ArrayList;
/**
 * Describes all aspects of a planar surface derived from the octree subset and theta, phi, rho of point.
 * @author jg
 *
 */
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
   private Vector4d m_scale;
   Vector4d m_color;

   ArrayList<Vector4d> m_points;
   public plane_t() {}
   /**
    * A lot of the planar elements are set in peak_detection.detect, from where this is called.
    * This method called from there after some initialization.
    * From theta, phi, rho and nodes in octree, calculate the centroids, the normals, etc of the planes.
    * Everything affecting the 'm_' members.
    * nodes = accum.convolution_nodes(bin.theta_index, bin.phi_index, bin.rho_index);
    */
   void calculate() { 
	  m_normal = new Vector4d();
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
      m_scale = new Vector4d(1,1,1,1);
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
   
   @Override
   public String toString() {
	   return "plane_t theta="+m_theta+" phi="+m_phi+" rho="+m_rho+" normal="+m_normal+
			   " centroid="+m_centroid+" position="+m_position+" votes="+votes+
			   " representativeness="+representativeness+" octree nodes="+nodes.size()+" m_cross="+m_cross+" m_cross2="+m_cross2+
			   " m_desloc="+m_desloc+" m_scale="+m_scale;
   }
   
   public void draw() {
	   double size = 1;
      Vector4d cross = m_normal.multiply(m_cross);//Matrix4d.Rotation(m_rotate,m_normal) * m_cross;
      Vector4d cross2 = m_normal.multiply(m_cross);//Matrix4d.Rotation(m_rotate,m_normal) * m_cross2;
	  Vector4d vert1a = m_position.add(cross);//m_position + cross
	  Vector4d vert1s = m_position.subtract(cross);//m_position - cross
	  Vector4d vert1ca = cross2.add(size);//size + cross2
	  Vector4d vert1cs = cross2.add(-size);//size - cross2 .. a - b = a + -b
	  Vector4d vert1aa = m_desloc.add(size).add(cross); //size + m_desloc + cross
	  Vector4d vertscx = cross2.add(m_scale.x);// m_scale.x + cross2
	  Vector4d vertscy = cross2.add(m_scale.y);//m_scale.y + cross2
	  Vector4d vert1 = vert1a.multiply(vert1ca);//m_position + cross * size + cross2
	  Vector4d vert2 = vert1a.multiply(vert1cs);//m_position + cross * size - cross2
	  Vector4d vert3 = vert1s.multiply(vert1cs);//m_position - cross * size - cross2
	  Vector4d vert4 = vert1s.multiply(vert1ca);//m_position - cross * size + cross2 
	  vert1 = vert1.multiply(vert1aa).multiply(vertscx).multiply(m_scale.z);
	  vert2 = vert2.multiply(vert1aa).multiply(vertscx).multiply(m_scale.w);
	  vert3 = vert3.multiply(vert1aa).multiply(vertscy).multiply(m_scale.w);
	  vert4 = vert4.multiply(vert1aa).multiply(vertscy).multiply(m_scale.z);
	  System.out.println(vert1);
	  System.out.println(vert2);
	  System.out.println(vert3);
	  System.out.println(vert4);
	  System.out.println();
	  //glVertex3dv((m_position + cross * size + cross2 * size + m_desloc + cross * m_scale.x + cross2 * m_scale.z).data);
      //glVertex3dv((m_position + cross * size - cross2 * size + m_desloc + cross * m_scale.x + cross2 * m_scale.w).data);
      //glVertex3dv((m_position - cross * size - cross2 * size + m_desloc + cross * m_scale.y + cross2 * m_scale.w).data);
      //glVertex3dv((m_position - cross * size + cross2 * size + m_desloc + cross * m_scale.y + cross2 * m_scale.z).data);
   }
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
   /**
    * Only used in plane coloring, therefore efficiency not critical
    * @param point
    * @return
    */
   double distance2plane( Vector4d point ) {
	return Math.abs((point.subtract(m_position)).and(m_normal.Normalized()));
   }

}
