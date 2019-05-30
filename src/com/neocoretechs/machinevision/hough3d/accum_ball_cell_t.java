package com.neocoretechs.machinevision.hough3d;
/**
 * An accumulator cell (theta, phi) with an array of (rho) cells
 * @author jg
 *
 */
public class accum_ball_cell_t {
   private static final boolean DEBUG = false;
   accum_cell_t[] bins;
   public accum_ball_cell_t(int size) {
	   if( DEBUG )
		   System.out.println("Initializing accumulator ball cell to "+size+" bins");
      bins = new accum_cell_t[size];
	  for(int i = 0; i < size; i++) {
		  bins[i] = new accum_cell_t();
	  }
   }
}