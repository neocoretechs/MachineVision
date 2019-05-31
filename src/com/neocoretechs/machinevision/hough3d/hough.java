package com.neocoretechs.machinevision.hough3d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
/**
 * Primary 3D kernel hough transform driver class.
 * Subdivides quadtree, builds accumulator ball, creates bins, detects peaks, sorts planes,
 * optionally sets the color in each plane for display and finally returns the populated accumulator ball.
 * @author jg
 *
 */
public class hough {
  public static boolean DEBUG = true;
  /**
  *  Descending sort based on representativeness
  */
  public static Comparator<plane_t> planeComparator = new Comparator<plane_t>() {
    @Override         
    public int compare(plane_t p1, plane_t p2) { 
      return (p2.representativeness < p1.representativeness ? -1 : 
              (p2.representativeness == p1.representativeness ? 0 : 1));
    }
  };

  /**
   * Primary 3D kernel hough transform driver method.
   * Subdivides quadtree, builds accumulator ball, creates bins, detects peaks, sorts planes,
   * optionally sets the color in each plane for display and finally returns the populated accumulator ball.
   * @param planes ArrayList ot be filled with planes
   * @param father Root node of quadtree
   * @param settings The hough_settings that determine plane construction etc.
   * @return
   */
  accumulatorball_t kht3d( ArrayList<plane_t> planes, octree_t father, hough_settings settings) {//double max_distance, double distance_discretization, int phi_cells_length )
	if( DEBUG) {
		System.out.println("Subdivide...");
	}
   // Subdividing Procedure
   father.subdivide(settings);
   // Initializes the Accumulator, nothing really happens in accumulator until the voting process
	if( DEBUG) {
		System.out.println("Build accumulator...");
	}
   accumulatorball_t accum = new accumulatorball_t(settings.max_point_distance, settings.rho_num, settings.phi_num);
	if( DEBUG) {
		System.out.println("Begin voting...Accum cells="+accum.getData().size());
	}
   // Voting Procedures
   ArrayList<bin_t> used_bins = new ArrayList<bin_t>();
   voting.vote(father, accum, used_bins, settings.max_point_distance);
	if( DEBUG) {
		ArrayList<ArrayList<accum_ball_cell_t>> ab = accum.getData();
		System.out.println("Accum cells="+ab.size());
		for(int i = 0; i < ab.size(); i++) {
			System.out.println("accum cell "+i+" size="+ab.get(i).size());
		}
		System.out.println("Peak detection..");
	}
   // Peak Detection Procedure
   peak_detection.detect(planes, accum, used_bins);
	if( DEBUG) {
		ArrayList<ArrayList<accum_ball_cell_t>> ab = accum.getData();
		System.out.println("Accum cells="+ab.size());
		for(int i = 0; i < ab.size(); i++) {
			System.out.println("accum cell "+i+" size="+ab.get(i).size());
		}
		System.out.println("Peaks detected, accumulate planes..");
	}
   for (plane_t p : planes) {
      accum.at(p.ti,(short)p.pi,(short)p.ri).peak = true;
   }
	if( DEBUG) {
		System.out.println("Sort planes by representativeness...");
	}
   // Sorting planes by representativeness
   for(int i = 0; i < planes.size(); i++) {
      planes.get(i).representativeness = 0;
	  for(int j = 0; j < planes.get(i).nodes.size(); j++)
         planes.get(i).representativeness += planes.get(i).nodes.get(j).representativeness;
   }

   Collections.sort(planes, planeComparator);

  if( DEBUG ) {
	System.out.println("Optional planes coloring for display...");
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
  /**
   * Command line invocation of 3dKHT process, reading from file in command line with path set in hough_settings
   * @param args
   */
  public static void main(String[] args) {
	Vector4d points;
	Vector4d colors;
	Vector4d color_map;
	ArrayList<plane_t> planes_out = new ArrayList<plane_t>();
	hough_settings settings = new hough_settings();
	accumulatorball_t accum;
	octree_t father = new octree_t(true);
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
	reader_file rf = null;
	if(args.length > 0)
		rf = new reader_file(args[0]);
	else
		rf = new reader_file(null);
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
