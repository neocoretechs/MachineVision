package com.neocoretechs.machinevision.hough3d;

import java.util.ArrayList;
import java.util.Collections;
/**
 * Detect peaks in the spherical accumulator cells. votes in the bin are updated,
 * used_bins is sorted and used to build planes.
 * 
 * @author jg
 *
 */
public class peak_detection {
	/**
	 * Build the planes from bins and ball. 
	 * @param planes The collection of planes built from bins and the ball
	 * @param accum the accumulator ball
	 * @param used_bins Iterate over accumulator detecting cells not adjacent to already inspected ones
	 */
static void detect(ArrayList<plane_t> planes, accumulatorball_t accum, ArrayList<bin_t> used_bins){
   // Gaussian Convolution of Voted Cells 
   for (bin_t bin : used_bins) {
      //accum_cell_t &cell = accum.at(bin.theta_index, bin.phi_index, bin.rho_index);
      bin.votes = accum.convolution_value( bin.theta_index, bin.phi_index, bin.rho_index);
   }
   // Sort the Auxiliary Array (AA - Convoluted Voted Cells) in descending order 
   //std::stable_sort
   Collections.sort(used_bins);
   // Iterate over accumulator detecting cells not adjacent to already inspected ones
   for (bin_t bin : used_bins) {
      if (!accum.visited_neighbor(bin.theta_index, bin.phi_index, bin.rho_index)) {
         plane_t p = new plane_t();
         // fill new plane instance with p.m_theta, p.m_phi, p.m_rho,
         accum.get_values( p, bin.theta_index, bin.phi_index, bin.rho_index);
         p.nodes = accum.convolution_nodes(bin.theta_index, bin.phi_index, bin.rho_index);
         p.representativeness = 0;
         p.votes = bin.votes;
         p.calculate();
         p.ti = bin.theta_index;
         p.pi = bin.phi_index;
         p.ri = bin.rho_index;
         planes.add(p);
      }
      accum.set_visited( bin.theta_index, bin.phi_index, bin.rho_index );
   }
}
}
