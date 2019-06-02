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
   @Override
   public String toString() {
	   StringBuilder sb=new StringBuilder("accum_ball_cell_t bins=");
	   sb.append(bins.length);
	   sb.append("\r\n");
	   for(int i = 0; i < bins.length; i++) {
		   sb.append(i);
		   sb.append("=");
		   sb.append(bins[i]);
		   sb.append("\r\n");
	   }
	   return sb.toString();
   }
}