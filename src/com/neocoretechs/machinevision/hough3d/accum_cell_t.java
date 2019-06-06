package com.neocoretechs.machinevision.hough3d;

import java.util.ArrayList;

public final class accum_cell_t {
	/**
	 * An accumulator cell (theta, phi) which constitutes an array of (rho) cells,
	 * AKA bins, in accum_ball_cell_t. The collection of accum_ball_cell_t is at the theta index,
	 * and the phi index is the first index into m_data in acumulatorball_t, which is a phi collection 
	 * of the collections of thetas that hold the array of rho, all of which make up the spherical 
	 * surface of the accumulator ball.
	 * @author jg
	 *
	 */
	  ArrayList<octree_t> ref_node = new ArrayList<octree_t>();
	  private octree_t last_node_voted;
	  boolean peak; // set when we start accumulating planes, for each plane we use accumulator.at(theta, phi, rho) to get here
	  boolean visited; // determines if its a neighbor that has been visited in peak_detection detect
	  boolean voted; // set from cast_vote. If the kernel.node doesnt pass verify_cell
	  boolean top; // set from voting gaussian_vote_3d
	  float last_cast_vote; // set from voting cast_vote
	  float bin; // set from voting cast_vote AND accumulatorball_t convolution_value

	   public accum_cell_t() {
	      last_cast_vote = 0;
	      last_node_voted = null;
	      visited = false;
	      top = false;
	      voted = false;
	      peak = false;
	      bin = 0;
	   }
	   // called from voting cast_vote
	   boolean verify_cell(octree_t ref) {
		  if( last_node_voted == null )
			  return false;
	      return (last_node_voted == ref);//(last_node_voted.equals(ref));
	   }
	   // called from cast_vote
	   void apply_cell(octree_t ref) {
	      last_node_voted = ref;
	   }
	   // called from voting cast_vote
	   public void add_reference(octree_t ref){
	      ref_node.add(ref);
	   }
	   @Override
	   public String toString() { return "accum_cell_t peak="+peak+" visited="+visited+" voted="+voted+
			   " top="+top+" last_cast_vote="+last_cast_vote+" bin="+bin+" octree nodes="+ref_node.size(); }
}
