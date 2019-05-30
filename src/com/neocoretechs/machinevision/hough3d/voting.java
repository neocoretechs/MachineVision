package com.neocoretechs.machinevision.hough3d;

import java.util.ArrayList;

//#include "voting.h"
//#include "Thirdparty/SwissArmyKnife/Matrix4d.h"
//#include <ctime>

public class voting {
	/**
	 * 
	 * @param root
	 * @param accumulator
	 * @param used_bins
	 * @param max_point_distance
	 */
	public voting(octree_t root, accumulatorball_t accumulator, ArrayList<bin_t> used_bins, double max_point_distance){
	   // Calculate kernels recursively
	   ArrayList<kernel_t> used_kernels = new ArrayList<kernel_t>();
	   kernels(root, accumulator, used_kernels, max_point_distance);
	   // Vote for all kernels
	   for (short i = 0; i < used_kernels.size(); i++)
	      gaussian_vote_3d(used_kernels.get(i), accumulator, used_bins); 
	}
	/**
	 * 
	 * @param used_bins
	 * @param cell
	 * @param kernel
	 * @param votes
	 * @param t
	 * @param p
	 * @param r
	 * @return
	 */
boolean cast_vote(ArrayList<bin_t> used_bins, accum_cell_t cell, kernel_t kernel, double votes, double t, int p, int r){
   // Cluster representativeness
   votes = votes * kernel.node.representativeness;
   
   // Test if the cell has already been voted by this kernel 
   if (cell.verify_cell(kernel.node)) 
   {
      // Test if the previous vote was smaller than the current 
      if (cell.last_cast_vote < votes) {
         // Remove 
         cell.bin += -cell.last_cast_vote+votes;
         cell.last_cast_vote = (float) votes;
      }
      else {
         return false;
      }

   // First node vote
   } else {
      
      // Store how many votes will be cast
      cell.last_cast_vote = (float) votes;
      // Increment votes
      cell.bin += votes;
      // Add reference of the node that votes for this cell
      cell.add_reference(kernel.node);

      if (!cell.voted)
      {
         used_bins.add(new bin_t(t, (short)p, (short)r));
         cell.voted = true;
      }
      // Track last node that votes for this cell
      cell.apply_cell(kernel.node);
   }
   
   return true;
}

/**
 * Voting in 1 dimension (rho)
 * @param accum
 * @param kernel
 * @param used_bins
 * @param theta_index
 * @param phi_index
 * @param rho_start_index
 * @param theta
 * @param phi
 * @return
 */
boolean gaussian_vote_1d(accumulatorball_t accum, kernel_t kernel, ArrayList<bin_t> used_bins, 
                             double theta_index, int phi_index, int rho_start_index, 
                             double theta, double phi){
   
   int rho_index, p, inc_rho_index;
   double rho, t, inc_rho, gauss;
   boolean voted = false;

   float votes;

   p = phi_index;
   t = theta_index;
   inc_rho = accum.m_delta_rho;
   inc_rho_index = 1;


   // Voting in the RHO array the direction "positive"
   for (rho_index = rho_start_index, rho = 0.0 ;; rho_index += inc_rho_index, rho += inc_rho)
   {
      if (rho_index < 0) inc_rho_index *= -1;
      if (accum.process_rho(t, p, rho_index) == false) break;

      gauss = kernel.trivariated_gaussian_dist_normal(rho, phi, theta);
      if (gauss < kernel.voting_limit) break;

      if ((votes = (float)(gauss)) >= (float)(0.0)) {
         voted = cast_vote(used_bins, accum.at(t, (short)p, (short)rho_index), kernel, votes, t, p, rho_index);
      } else break;
   }


   // Voting in the RHO array the direction "negative"
   inc_rho_index = -1;
   for (rho_index = rho_start_index-1, rho = -inc_rho;;rho_index += inc_rho_index, rho -= inc_rho)
   {
      if (rho_index < 0) inc_rho_index *= -1;
      if (accum.process_rho(t, p, rho_index) == false) break;

      gauss = kernel.trivariated_gaussian_dist_normal(rho, phi, theta);
      if (gauss < kernel.voting_limit) break;

      if ((votes = (float)(gauss)) >= (float)(0.0)) {
         voted = cast_vote(used_bins, accum.at(t, (short)p,(short) rho_index), kernel, votes, t, p, rho_index);
      } else break;
   }
   return voted;
}

/**
 * Voting in 2 dimensions (theta, phi)
 * @param accum
 * @param kernel
 * @param used_bins
 * @param theta_start_index
 * @param phi_start_index
 * @param rho_start_index
 * @param phi_start
 * @param inc_phi_index
 */
void gaussian_vote_2d( accumulatorball_t accum, kernel_t kernel, ArrayList<bin_t> used_bins, 
                       double theta_start_index, int phi_start_index, int rho_start_index,
                       double phi_start, int inc_phi_index){
   double inc_phi = (double)inc_phi_index * accum.m_delta_angle;
   int phi_index, rho_index = rho_start_index;
   boolean voted, voting_ended = false;
   double theta_index, theta, phi, theta_init;

   // Voting in the PHI array in the direction "inc_phi_index"
   for (phi_index = phi_start_index, phi = phi_start; !voting_ended ; phi_index += inc_phi_index, phi += inc_phi) {
      
      theta_index = theta_start_index;
      if (accum.process_phi(theta_index, phi_index)) inc_phi_index *= -1;
      theta_init = accum.fix_theta(theta_index,phi_index);
      double inc_theta = accum.delta_theta(phi_start_index);
      double inc_theta_index = accum.delta_theta_index(phi_index);
      double curr_theta_index;
      voted = false;
      voting_ended = true;
      // Voting in the THETA array in the direction "positive"
      for (curr_theta_index = theta_init, theta = 0.0 ;; curr_theta_index+=inc_theta_index, theta += inc_theta) {
         
         accum.process_theta(curr_theta_index); 
         accum.initialize(curr_theta_index, phi_index);
         voted = gaussian_vote_1d(accum, kernel, used_bins, curr_theta_index, phi_index, rho_index, theta, phi);
         if (voted) {
            voting_ended = false;
         } else break;
      }
      
      // Voting in the THETA array in the direction "negative"
      for (curr_theta_index = theta_init - inc_theta_index, theta = -inc_theta;; curr_theta_index -= inc_theta_index, theta -= inc_theta) {
         accum.process_theta(curr_theta_index);
         accum.initialize(curr_theta_index, phi_index);
         voted = gaussian_vote_1d(accum, kernel, used_bins, curr_theta_index, phi_index, rho_index, theta, phi);
         if (voted) {
            voting_ended = false;
         } else break;
      }
   }
}

/**
 * Voting in 3 dimensions (theta, phi and rho)
 * @param kernel
 * @param accum
 * @param used_bins
 */
void gaussian_vote_3d(kernel_t kernel, accumulatorball_t accum, ArrayList<bin_t> used_bins){  
      accum.at(kernel.theta_index, (short)kernel.phi_index,(short)kernel.rho_index).top = true;   
      gaussian_vote_2d(accum, kernel, used_bins, kernel.theta_index, kernel.phi_index, kernel.rho_index, 0, +1);
      int phi_index = kernel.phi_index-1;
      double theta_index = kernel.theta_index;
      accum.process_phi(theta_index, phi_index);
      gaussian_vote_2d(accum, kernel, used_bins, theta_index, phi_index, kernel.rho_index, -accum.m_delta_angle, -1);
}

/**
 * Calculates gaussian kernel parameters
 * @param node
 * @param accum
 * @param used_kernels
 * @param max_point_distance
 */
void kernel_calculation(octree_t node, accumulatorball_t accum, ArrayList<kernel_t> used_kernels, double max_point_distance){
   kernel_t kernel = new kernel_t();
   kernel.node = node;   
   if ((node.m_centroid.Normalized().and(node.normal1)) < 0)  node.normal1.multiply(-1.0);
   kernel.phi = Math.acos(node.normal1.z);
   kernel.theta = Math.atan2(node.normal1.y, node.normal1.x);
   kernel.rho = node.m_centroid.x * node.normal1.x + node.m_centroid.y * node.normal1.y + node.m_centroid.z * node.normal1.z;
   accum.process_limits(kernel.theta_index, kernel.phi_index, kernel.rho_index);
   accum.get_index(kernel.theta, kernel.phi, kernel.rho,  kernel.theta_index, kernel.phi_index, kernel.rho_index);
   kernel.theta_index = accum.fix_theta(kernel.theta_index,kernel.phi_index);
   kernel.kernel_load_parameters(max_point_distance);
   used_kernels.add(kernel);
}

/**
 * search nodes for kernel calculation
 * @param root
 * @param accumulator
 * @param used_kernels
 * @param max_point_distance
 */
void kernels(octree_t root, accumulatorball_t accumulator , ArrayList<kernel_t> used_kernels, double max_point_distance) {
   if (root.coplanar) {
      kernel_calculation(root, accumulator, used_kernels, max_point_distance);
   }
   // If not leaf
   else if (root.m_children != null)
   {
      // Call children
      for (short i = 0; i < 8 ; i++) {
         kernels(root.m_children[i], accumulator , used_kernels, max_point_distance);
      }
   }
}

}
