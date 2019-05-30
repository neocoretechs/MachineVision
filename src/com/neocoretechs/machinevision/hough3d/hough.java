package com.neocoretechs.machinevision.hough3d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

public class hough {
  public static boolean DEBUG = false;
 /**
  *  descending sort
  */
 public static Comparator<plane_t> planeComparator = new Comparator<plane_t>() {
    @Override         
    public int compare(plane_t p1, plane_t p2) { 
      return (p2.representativeness < p1.representativeness ? -1 : 
              (p2.representativeness == p1.representativeness ? 0 : 1));
    }
  };
// descending sort function  
//boolean orden (plane_t p1 , plane_t p2) { return (p1.representativeness > p2.representativeness); }

accumulatorball_t kht3d( ArrayList<plane_t> planes, octree_t father, hough_settings settings) //double max_distance, double distance_discretization, int phi_cells_length )
{
   // Subdividing Procedure
   father.subdivide(settings);
   // Initializes the Accumulator
   accumulatorball_t accum = new accumulatorball_t(settings.max_point_distance, settings.rho_num, settings.phi_num);
   // Voting Procedure
   ArrayList<bin_t> used_bins = new ArrayList<bin_t>();
   // Peak Detection Procedure
   peak_detection.detect(planes, accum, used_bins);   
   for (plane_t p : planes) {
      accum.at(p.ti,(short)p.pi,(short)p.ri).peak = true;
   }
   // Sorting planes by representativeness
   for(int i = 0; i < planes.size(); i++) {
      planes.get(i).representativeness = 0;
	  for(int j = 0; j < planes.get(i).nodes.size(); j++)
         planes.get(i).representativeness += planes.get(i).nodes.get(j).representativeness;
   }
   //std::sort(planes.begin(),planes.end(), orden);
   Collections.sort(planes, planeComparator);


if( DEBUG ) {

   // Coloring planes and points
   for (int i = 0; i < planes.size(); i++) {
      Vector4d cor = null;
      switch(i%6) {
      case 0:cor = new Vector4d((int)(255/(int)(i/6+1)),0,0).divide(255.0);break;
      case 1:cor = new Vector4d(0,(int)(255/(int)(i/6+1)),0).divide(255.0);break;
      case 2:cor = new Vector4d(0,0,(int)(255/(int)(i/6+1))).divide(255.0);break;
      case 3:cor = new Vector4d(0,(int)(255/(int)(i/6+1)),(int)(255/(int)(i/6+1))).divide(255.0);break;
      case 4:cor = new Vector4d((int)(255/(int)(i/6+1)),0,(int)(255/(int)(i/6+1))).divide(255.0);break;
      case 5:cor = new Vector4d((int)(255/(int)(i/6+1)),(int)(255/(int)(i/6+1)),0).divide(255.0);break;
      }

      planes.get(i).m_color.set(cor);

	  for (int j = 0; j < planes.get(i).nodes.size(); j++)
         planes.get(i).nodes.get(j).color.set(cor);
   }
   
   
   for(int i = 0; i < father.m_points.size(); i++){
	   for(int p = 0; p < planes.size(); p++) {
         if (planes.get(p).distance2plane(father.m_points.get(i)) < settings.max_distance2plane) {
            father.m_colors.get(i).set(planes.get(p).m_color);
            break;
         }
      }
   }
} // debug

   used_bins.clear();
   return accum;
}

public static void main(String[] args) {
	Vector4d points;
	Vector4d colors;
	Vector4d color_map;
	ArrayList<plane_t> planes_out = new ArrayList<plane_t>();
	hough_settings settings = new hough_settings();
	accumulatorball_t accum;
	octree_t father = new octree_t();

	double size = 0.6;
	double max_distance = 0.0;
	boolean show_settings[];
	boolean realtime_cond = true;
	int index_plane = 0;
	int plane_value = 0;
	int number_planes_show = 1;
	int rho_index_accumulator = 50;
	int alturaoc = 1;
	int cont_frames = 1;
	int octree_height = 0;
	reader_file rf = new reader_file();
	rf.load_point_cloud(settings, father);
	Vector4d centroid = father.m_centroid.divide(father.m_points.size());
	for(Vector4d  v : father.m_points) {
	      v = v.subtract(centroid);
	      max_distance =Math.max(max_distance,Math.abs(v.x));
	      max_distance = Math.max(max_distance,Math.abs(v.y));
	      max_distance = Math.max(max_distance,Math.abs(v.z));
	      settings.max_point_distance = Math.max(settings.max_point_distance,v.getLength());
	   }

	   father.m_centroid = new Vector4d();
	   father.m_size = max_distance * 2.0;
	   hough h = new hough();
	   accum = h.kht3d(planes_out, father, settings);
	   System.out.println("Number of planes detected = "+planes_out.size());
}

}
