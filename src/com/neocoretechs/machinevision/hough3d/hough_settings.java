package com.neocoretechs.machinevision.hough3d;

/**
 * Settings to drive planar detection process.
 * s_ms is minimum size of m_indexes array to limit recursion.
 * s_ms MUST be > 0 and large enough to prevent depth recursion from blowing the stack.
 * s_level determines the level at which m_level we check for variance direction, therefore
 * a higher number will detect more and smaller coplanar, or potential planar, regions.
 * s_level >= level will potentially remove outliers that are greater than m_size/10 from the plane we are trying to form.
 * max_thickness and min_isotropy determine when we remove outliers, compute least_variance_direction again, 
 * and set coplanar to true.
 * @author jg
 *
 */
public class hough_settings {
      // Accumulator discretization
      public static final int phi_num = 30; // accumulator size - phi dimension 30 default
      public static final int rho_num = 300; // accumulator size - rho dimension 300 default
      // relative tolerances associated with plane thickness (s_alpha) and plane isotropy (s_beta)
	  // thickness < settings.max_thickness && isotropy > settings.min_isotropy
	  // s_alpha and s_beta are scaling factors defining relative tolerances for the acceptable amount
	  // of off-plane displacement (i.e., noise) and degree of sample anisotropy on the
	  // cluster. According to our experience, s_alpha = 25 and s_beta = 6 produce good
	  // results
      public static final double max_thickness = 5; //1/25=.04 see above, PCA analysis
      public static final double min_isotropy = .1;  //1/6 = .166 see above, PCA analysis
	  public static final int s_level = 7;//determines at which octree level we check for variance direction and remove outliers
	  public static final int s_ms = 3; // minimum number of points per octree node
      public static double max_point_distance = 1; // used for max_point_distance/rho in accumulator voting as delta_rho rho increment value
	  public static final double max_distance2plane = 5; //divisor for size for max plane distance for octree outlier removal (m_size/max_distance2plane)
	  // this declaration overrides any command line file name input
	  //public static String file = "/users/jg/workspace/robocore/motionclouds/roscoe1";
	  // this pure path declaration allows command line input of file name
	  public static String file = "/users/jg/workspace/robocore/motionclouds/";
	  public static String extension = ".asc";
      // Point cloud examples (let one block uncommented) =======================================
      // Max distance is only used for coloring the point cloud after the plane detection
	  // 9 planes
      /*
      s_level = 1;
      file = "Computer.txt";
      max_distance2plane = 0.025;
	  // 9 planes
      s_level = 4;
      file = "Room.txt";
      max_distance2plane = 0.2;
	  // 11 planes
      s_level = 5;
      file = "Utrecht.txt";
      max_distance2plane = 0.2; 
	  // 14 planes
	*/
	//      file = "Museum.txt";
    //  file = "pc_dataset_living_room_far_3.txt";
	//  max_distance2plane = 0.3;
 
	/*
	  // 6 planes
      s_level = 2;
      file = "Box.txt";
      max_distance2plane = 10.0;      
	  // 13 planes     
      s_level = 7;
      file = "Bremen.txt";
      max_distance2plane = 0.5;      
	*/      
      // ========================================================================================     
   }
