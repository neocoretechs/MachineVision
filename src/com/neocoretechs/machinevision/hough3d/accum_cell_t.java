package com.neocoretechs.machinevision.hough3d;

import java.util.ArrayList;

public class accum_cell_t {
	/**
	 * An accumulator cell (theta, phi) with an array of (rho) cells
	 * @author jg
	 *
	 */
	  ArrayList<octree_t> ref_node = new ArrayList<octree_t>();
	  octree_t last_node_voted;
	  boolean peak, visited, voted, top;
	  float last_cast_vote, bin;

	   public accum_cell_t() {
	      last_cast_vote = 0;
	      last_node_voted = null;
	      visited = false;
	      top = false;
	      voted = false;
	      peak = false;
	      bin = 0;
	   }

	   //~accum_cell_t()
	   //{
	   //   ref_node.clear();
	   //}

	   boolean verify_cell(octree_t ref) {
	      return (last_node_voted==ref);
	   }
	   void apply_cell(octree_t ref) {
	      last_node_voted = ref;
	   }

	   public void add_reference(octree_t ref){
	      ref_node.add(ref);
	   }
}
