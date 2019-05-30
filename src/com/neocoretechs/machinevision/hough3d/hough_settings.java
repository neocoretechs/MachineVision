package com.neocoretechs.machinevision.hough3d;

/**
 * Settings to drive planar detection process.
 * s_ms is minimum size of m_indexes array to limit recursion.
 * s_ms MUST be > 0 and large enough to prevent depth recursion from blowing the stack.
 * s_level determines the level at which m_level we check for variance direction
 * and potentially remove outliers that are greater than m_size/10 from the plane we are trying to form.
 * max_thickness and min_isotropy determine when we
 * remove outliers, compute least_variance_direction again, and set coplanar to true.
 * max_point_distance seems to affect delta-rho
 * @author jg
 *
 */
public class hough_settings {
      // Accumulator discretization
      public static final int phi_num = 30;
      public static final int rho_num = 300;
	  // Percentage of the number of points from the point cloud to stop subdividing
	  public static final double s_ps = 0.002;
      // relative tolerances associated with plane thickness (s_alpha) and plane isotropy (s_beta)
      public static final double max_thickness = .5; //1/25=.04
      public static final double min_isotropy = .5;  //1/6 = .166
	  public static final int s_level = 4;
	  public static final int s_ms = 2;
      public static double max_point_distance = 1;
	  public static final double max_distance2plane = 0.25;
	  public static String file = "/users/jg/workspace/robocore/motionclouds/roscoe1";
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
