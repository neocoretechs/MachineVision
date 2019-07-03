package com.neocoretechs.machinevision.hough3d;

import java.awt.Color;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
/**
 * Primary 3D kernel hough transform driver class.
 * A robust segmentation strategy to identify clusters of approximately coplanar samples. Votes are cast for
 * clusters as opposed to for individual samples, greatly accelerating the detection process.
 * This class subdivides quadtree, builds accumulator ball, creates bins, detects peaks, sorts planes,
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
  accumulatorball_t kht3d( ArrayList<plane_t> planes, octree_t father) {//double max_distance, double distance_discretization, int phi_cells_length )
	  long startTime = System.currentTimeMillis();
	  long totalTime = startTime;
	if( DEBUG) {
		System.out.println("Subdivide...");
	}
   // Subdividing Procedure
   father.subdivide();
   if( DEBUG )
	   System.out.println("Elapsed Subdivide took "+(System.currentTimeMillis()-startTime)+" ms.");
   // debug write octree with coplanar flags
   writer_file.writePerp(father);//writeChildren(father);
   // Initializes the Accumulator, nothing really happens in accumulator until the voting process
	if( DEBUG) {
		System.out.println("Build accumulator...");
		startTime = System.currentTimeMillis();
	}
   accumulatorball_t accum = new accumulatorball_t(hough_settings.max_point_distance, hough_settings.rho_num, hough_settings.phi_num);
	if( DEBUG) {
		System.out.println("Elapsed accumulator build took "+(System.currentTimeMillis()-startTime)+" ms.");
		startTime = System.currentTimeMillis();
		System.out.println("Begin voting...Accum cells="+accum.getData().size());
	}
   // Voting Procedures
   ArrayList<bin_t> used_bins = new ArrayList<bin_t>();
   // used bins filled during voting
   voting.vote(father, accum, used_bins, hough_settings.max_point_distance);
	if( DEBUG) {
		System.out.println("Elapsed voting took "+(System.currentTimeMillis()-startTime)+" ms.");
		/*
		ArrayList<ArrayList<accum_ball_cell_t>> ab = accum.getData();
		System.out.println(">>>>Accum cells="+ab.size());
		for(int i = 0; i < ab.size(); i++) {
			System.out.println(">>>accum cell "+i+" size="+ab.get(i).size());
			for(int j = 0; j < ab.get(i).size(); j++) {
				System.out.print(">>accumulator cell "+j+"=");
				System.out.println(ab.get(i).get(j)); // get accum_ball_cell_t
			}
		}
		*/
		startTime = System.currentTimeMillis();
		System.out.println("Peak detection..");
	}
   // Peak Detection Procedure, sets the accumulator_cell 'visited' flag
   peak_detection.detect(planes, accum, used_bins);
	if( DEBUG) {
		System.out.println("Elapsed peak detection took "+(System.currentTimeMillis()-startTime)+" ms.");
		System.out.println("Used bins:"+used_bins.size());
		for(bin_t bin : used_bins) {
			System.out.println("bin="+bin);
		}
		ArrayList<ArrayList<accum_ball_cell_t>> ab = accum.getData();
		System.out.println("Accum cells="+ab.size());
		for(int i = 0; i < ab.size(); i++) {
			System.out.println("accum cell "+i+" size="+ab.get(i).size());
		}
		startTime = System.currentTimeMillis();
		System.out.println("Peaks detected, accumulate planes..");
		
	}
   for (plane_t p : planes) {
      accum.at(p.ti,(short)p.pi,(short)p.ri).peak = true;
   }
	if( DEBUG) {
		System.out.println("Elapsed plane accumulation took "+(System.currentTimeMillis()-startTime)+" ms.");
		startTime = System.currentTimeMillis();
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
      planes.get(i).m_color = cor;
	  for (int j = 0; j < planes.get(i).nodes.size(); j++)
         planes.get(i).nodes.get(j).color = cor;
   }
   for(int i = 0; i < father.m_points.size(); i++){
	   for(int p = 0; p < planes.size(); p++) {
         if (planes.get(p).distance2plane(father.m_points.get(i)) < hough_settings.max_distance2plane) {
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
	accumulatorball_t accum;
	octree_t father = new octree_t(true);
	double size = 0.6;
	double max_distance = 0.0; // absolute longest bound in x,y, or z
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
	if(!rf.load_point_cloud(father)) {
		System.out.println("File failed to load, so thats it");
		return;
	}
	if( DEBUG ) {
		System.out.println("Loaded "+father.m_points.size()+" points...");
	}
	// computed centroid in load_point_cloud
	//Vector4d centroid = father.m_centroid.divide(father.m_points.size());
	// establish farthest distance between centroid and any point on any axis.
	for(Vector4d  vx : father.m_points) {
		// v = v.subtract(centroid);
	      Vector4d v = vx.subtract(father.m_centroid);
	      max_distance = Math.max(max_distance,Math.abs(v.x));
	      max_distance = Math.max(max_distance,Math.abs(v.y));
	      max_distance = Math.max(max_distance,Math.abs(v.z));
	      // length is vector magnitude from origin
	      hough_settings.max_point_distance = Math.max(hough_settings.max_point_distance,v.getLength());
	}
	if( DEBUG )
		System.out.println("octree centroid="+father.m_centroid+" max vector span="+hough_settings.max_point_distance);
	//father.m_centroid = new Vector4d(); ? orig code
	father.m_size = max_distance * 2.0;
	hough h = new hough();
	accum = h.kht3d(planes_out, father);
	System.out.println("Number of planes detected = "+planes_out.size());
	for(int i = 0; i < planes_out.size(); i++) {
		System.out.println(i+"="+planes_out.get(i));
		planes_out.get(i).draw();
	}
	writer_file.writePlanes(planes_out);
  }

}
