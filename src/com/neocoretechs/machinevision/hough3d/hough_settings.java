package com.neocoretechs.machinevision.hough3d;

/**
 * Settings to drive planar detection process.
 * s_ms is minimum size of m_indexes array to limit recursion.
 * s_ms MUST be > 0 and large enough to prevent depth recursion from blowing the stack.
 * s_level determines the level at which m_level we check for variance direction, therefore
 * a higher number will detect more and smaller coplanar, or potential planar, regions.
 * s_level >= level will potentially remove outliers that are greater than m_size/10 from the plane we are trying to form.<br/>
 * Relative tolerances associated with plane thickness (s_alpha) and plane isotropy (s_beta)<br/>
 * s_alpha and s_beta are scaling factors defining relative tolerances for the acceptable amount
 * of off-plane displacement (i.e., noise) and degree of sample anisotropy on the
 * cluster. According to our experience, s_alpha = 25 and s_beta = 6 produce good
 * results. max_thickness and min_isotropy determine when we remove outliers, 
 * compute least_variance_direction and set coplanar to true.
 * @author jg
 *
 */
public class hough_settings {
      // Accumulator discretization
      public static final int phi_num = 30; // accumulator size - phi dimension 30 default
      public static final int rho_num = 300; // accumulator size - rho dimension 300 default
      /**
       * Relative octree PCA tolerances associated with plane thickness (s_alpha) and plane isotropy (s_beta)<br/>
	   * If thickness is less than max_thickness and isotropy is greater than min_isotropy
	   * then remove outliers, compute least_variance_direction, and set coplanar to true.<br/>
	   * thickness = variance1 / variance2, isotropy  = variance2 / variance3;<br/>
	   * default is 5.
       */
      public static double max_thickness = 5;
      /**
       * Relative octree PCA tolerances associated with plane thickness (s_alpha) and plane isotropy (s_beta)<br/>
	   * If thickness is less than max_thickness and isotropy is greater than min_isotropy
	   * then remove outliers, compute least_variance_direction, and set coplanar to true.<br/>
	   * thickness = variance1 / variance2, isotropy  = variance2 / variance3;<br/>
	   * default is 0, which allows any manner of deformed degenerate plane.
       */
      public static double min_isotropy = 0;
      /**
       * s_level determines maximum octree level we check for variance direction and remove outliers.<br/>
       * Coplanar areas may be formed at higher level if measure for isoptropy passes, but s_level determines smallest cell.<br/>
       * default is level 7 which produces approximately 10x10 point cells.
       */
	  public static int s_level = 7;
	  /**
	   * s_ms determines the minimum number of points per octree node.
	   */
	  public static int s_ms = 5;
	  /**
	   * max_distance2plane is the divisor for size that determines max plane distance for 
	   * octree outlier removal (size_of_octree_cell/max_distance2plane).<br/>
	   * Subtract the centroid from the passed point and take the scalar dot product of that
	   * and the normalized 'normal1' vector. The absolute value of that is the distance to plane.<br/>
	   * If this distance to plane is greater than size_of_octree_cell/max_distance2plane, we will
	   * remove the point from the cell.<br/>
	   * default is 5.
	   */
	  public static double max_distance2plane = 5;
	  //
	  public static double max_point_distance = 1; // used for max_point_distance/rho in accumulator voting as delta_rho rho increment value
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
