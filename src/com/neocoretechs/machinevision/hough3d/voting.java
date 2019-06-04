package com.neocoretechs.machinevision.hough3d;
import java.util.ArrayList;
/**
 * Cast votes for elements in the accumulator ball using gaussian kernel, our used bins and octree nodes.
 * @author jg
 *
 */
public final class voting {
	private static boolean DEBUG = false;

	/**
	 * Init by calculating kernels recursively, then casting votes for all kernels
	 * @param root
	 * @param accumulator
	 * @param used_bins
	 * @param max_point_distance
	 */
	public static void vote(octree_t root, accumulatorball_t accumulator, ArrayList<bin_t> used_bins, double max_point_distance){
	   // Calculate kernels recursively
	   ArrayList<kernel_t> used_kernels = new ArrayList<kernel_t>();
	   kernels(root, accumulator, used_kernels, max_point_distance);
	   // Vote for all kernels
	   for (short i = 0; i < used_kernels.size(); i++)
	      gaussian_vote_3d(used_kernels.get(i), accumulator, used_bins); 
	}
	/**
	 * Cast a vote, this is where elements are finally added to used_bins upon first vote
	 * indicated by cell.verify_cell(kernel.node);
	 * 
	 * Update the majority of elements in 'cell' including bin, last_cast_vote, voted, bin
	 * @param used_bins
	 * @param cell accumulator ball cell
	 * @param kernel
	 * @param votes Number of votes to cast
	 * @param tpr array of theta, phi, rho indexes
	 * @return false if last vote was smaller than this vote, true if otherwise or first vote
	 */
	private static boolean cast_vote(ArrayList<bin_t> used_bins, accum_cell_t cell, kernel_t kernel, double votes, double t, int p, int r){
		// Cluster representativeness
		votes = votes * kernel.node.representativeness; 
		// Test if the cell has already been voted by this kernel 
		if (cell.verify_cell(kernel.node)) {
			// Test if the previous vote was smaller than the current 
			if (cell.last_cast_vote < votes) {
				// Remove 
				cell.bin += ((-cell.last_cast_vote) + votes);
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
			if (!cell.voted) {
				used_bins.add(new bin_t(t, (short)p, (short)r));
				cell.voted = true;
			}
			// Track last node that votes for this cell, set last_node_voted to kernel.node
			cell.apply_cell(kernel.node);
		}
		return true;
	}

	/**
	 * Voting in 1 dimension (rho). uses gaussian normal dist
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
	private static boolean gaussian_vote_1d(accumulatorball_t accum, kernel_t kernel, ArrayList<bin_t> used_bins, 
                             double[] tpr, // theta_index, int phi_index, int rho_start_index, 
                             double theta, double phi){
		int rho_index,p, inc_rho_index;
		double rho, t, inc_rho, gauss;
		boolean voted = false;
		float votes;
		p = (int) tpr[1];//phi_index
		t = tpr[0];//theta_index;
		//double[] theta_phi_index = new double[]{tpr[0]/*theta_index*/, tpr[1]/*phi_index*/, 0.0};
		inc_rho = accum.m_delta_rho;
		inc_rho_index = 1;
		// Voting in the RHO array the direction "positive"
		for (rho_index = (int) tpr[2]/*rho_start_index*/, rho = 0.0 ;; rho_index += inc_rho_index, rho += inc_rho) {
			if (rho_index < 0) inc_rho_index *= -1;
			double[] tprx = new double[]{t, p, rho_index};
			if (!accum.process_rho(tprx)) {//if (accum.process_rho(t, p, rho_index) == false) break;
				t = tprx[0]; p = (int) tprx[1]; rho_index = (int) tprx[2];
				break;
			}
			t = tprx[0]; p = (int) tprx[1]; rho_index = (int) tprx[2];
			gauss = kernel.trivariated_gaussian_dist_normal(rho, phi, theta);
			if (gauss < kernel.voting_limit) 
				break;
			if ((votes = (float)(gauss)) >= (float)(0.0)) {
				voted = cast_vote(used_bins, accum.at(t, (short)p, (short)rho_index), kernel, votes,t, p, rho_index);
			} else 
				break;
		}
		if( DEBUG )
			System.out.println("voting gaussian_vote_1d after voting RHO DIR POS voted="+voted);
		// Voting in the RHO array the direction "negative"
		inc_rho_index = -1;
		for(rho_index = (int) (tpr[2]-1/*rho_start_index-1*/), rho = -inc_rho;;rho_index += inc_rho_index, rho -= inc_rho) {
			if (rho_index < 0) inc_rho_index *= -1;
			double[] tprx = new double[]{t, p, rho_index};
			if (!accum.process_rho(tprx)) {
			  t = tprx[0]; p = (int) tprx[1]; rho_index = (int) tprx[2];
    		  break;
			}
			t = tprx[0]; p = (int) tprx[1]; rho_index = (int) tprx[2];
			// gaussian normal dist
			gauss = kernel.trivariated_gaussian_dist_normal(rho, phi, theta);
			if (gauss < kernel.voting_limit)
				break;
			if ((votes = (float)(gauss)) >= (float)(0.0)) {
				if( DEBUG ) {
					System.out.println("voting gaussian_vote_1d RHO DIR NEG theta index="+t+" phi index="+p+" rho index="+rho_index);
				}
				voted = cast_vote(used_bins, accum.at(t, (short)p,(short)rho_index), kernel, votes, t, p, rho_index);
			} else 
				break;
		}
		if( DEBUG )
			System.out.println("voting gaussian_vote_1d after voting RHO DIR NEG returning voted="+voted);
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
	private static void gaussian_vote_2d( accumulatorball_t accum, kernel_t kernel, ArrayList<bin_t> used_bins, 
                       double[] tpr,//double theta_start_index, int phi_start_index, int rho_start_index,
                       double phi_start, int inc_phi_index){
		double inc_phi = (double)inc_phi_index * accum.m_delta_angle;
		//int phi_index, rho_index = rho_start_index;
		boolean voted, voting_ended = false;
		double /*theta_index,*/ theta, phi, theta_init;
		double[] theta_phi_index = new double[]{tpr[0],tpr[1],tpr[2]};
		// Voting in the PHI array in the direction "inc_phi_index"
		for(theta_phi_index[1]/*phi_index*/ = tpr[1]/*phi_start_index*/, phi = phi_start; !voting_ended ;
		   theta_phi_index[1]/*phi_index*/ += inc_phi_index, phi += inc_phi) {    
			theta_phi_index[0] = tpr[0];//theta_start_index;
			if (accum.process_phi(theta_phi_index)) inc_phi_index *= -1;
			theta_init = accum.fix_theta(theta_phi_index[0], (int) theta_phi_index[1]/*theta_index,phi_index*/);
			double inc_theta = accum.delta_theta((int)tpr[1]/*phi_start_index*/);
			double inc_theta_index = accum.delta_theta_index((int) theta_phi_index[1]/*phi_index*/);
			double curr_theta_index;
			voted = false;
			voting_ended = true;
			// Voting in the THETA array in the direction "positive"
			for(curr_theta_index = theta_init, theta = 0.0 ;; curr_theta_index+=inc_theta_index, theta += inc_theta) {      
				curr_theta_index = accum.process_theta(curr_theta_index); 
				accum.initialize(curr_theta_index, (int) theta_phi_index[1]/*phi_index*/);
				double[] currThetaPhi = new double[]{curr_theta_index, theta_phi_index[1], theta_phi_index[2]};
				voted = gaussian_vote_1d(accum, kernel, used_bins, currThetaPhi/*curr_theta_index,(int)phi_index,(int)rho_index*/, theta, phi);
				// update current index array
				curr_theta_index = currThetaPhi[0];
				theta_phi_index[1] = currThetaPhi[1];
				theta_phi_index[2] = currThetaPhi[2];
				if (voted) {
					voting_ended = false;
				} else 
					break;
			}
			// Voting in the THETA array in the direction "negative"
			for(curr_theta_index = theta_init - inc_theta_index, theta = -inc_theta;; curr_theta_index -= inc_theta_index, theta -= inc_theta) {
				curr_theta_index = accum.process_theta(curr_theta_index);
				accum.initialize(curr_theta_index, (int) theta_phi_index[1]/* phi_index*/);
				double[] currThetaPhi = new double[]{curr_theta_index, theta_phi_index[1], theta_phi_index[2]};
				voted = gaussian_vote_1d(accum, kernel, used_bins, currThetaPhi/*curr_theta_index, (int)phi_index, (int) rho_index*/, theta, phi);
				// update current index array
				curr_theta_index = currThetaPhi[0];
				theta_phi_index[1] = currThetaPhi[1];
				theta_phi_index[2] = currThetaPhi[2];
				if (voted) {
					voting_ended = false;
				} else 
					break;
			}
		}
	}

	/**
	 * Voting in 3 dimensions (theta, phi and rho)
	 * @param kernel
	 * @param accum
	 * @param used_bins
	 */
	private static void gaussian_vote_3d(kernel_t kernel, accumulatorball_t accum, ArrayList<bin_t> used_bins){  
		accum.at(kernel.thetaPhiRhoIndex[0]/*theta_index*/, (short)kernel.thetaPhiRhoIndex[1]/*phi_index*/,(short)kernel.thetaPhiRhoIndex[2]/*rho_index*/).top = true;
		gaussian_vote_2d(accum, kernel, used_bins, kernel.thetaPhiRhoIndex/* kernel.theta_index, kernel.phi_index, kernel.rho_index*/, 0, +1);
		//int phi_index = kernel.phi_index-1;
		//double theta_index = kernel.theta_index;
		double[] theta_phi_index = new double[]{kernel.thetaPhiRhoIndex[0], kernel.thetaPhiRhoIndex[1]-1, kernel.thetaPhiRhoIndex[2]};
		accum.process_phi(theta_phi_index/*theta_index, phi_index*/);
		gaussian_vote_2d(accum, kernel, used_bins, theta_phi_index/*theta_phi_index[0] theta_index,(int)theta_phi_index[1] phi_index, kernel.rho_index*/, -accum.m_delta_angle, -1);
		// may have modified the rho value and update needed in kernel
		kernel.thetaPhiRhoIndex[2]=theta_phi_index[2];
	}

	/**
	 * Calculates gaussian kernel parameters.
	 * creates a new kernel_t, assigns octree node to it, computes normals in node, does process_limits, get_index in
	 * accumulator with kernel theta,phi,rho indexes and values, does a kernel_load_parameters on kernel, and finally
	 * adds new kernel to used_kernels.
	 * @param node
	 * @param accum
	 * @param used_kernels
	 * @param max_point_distance
	 */
	private static void kernel_calculation(octree_t node, accumulatorball_t accum, ArrayList<kernel_t> used_kernels, double max_point_distance){
		kernel_t kernel = new kernel_t();
		kernel.node = node;   
		if ((node.m_centroid.Normalized().and(node.normal1)) < 0)  
			node.normal1.multiply(-1.0);
		kernel.phi = Math.acos(node.normal1.z);
		kernel.theta = Math.atan2(node.normal1.y, node.normal1.x);
		kernel.rho = node.m_centroid.x * node.normal1.x + node.m_centroid.y * node.normal1.y + node.m_centroid.z * node.normal1.z;
		accum.process_limits(kernel.thetaPhiRhoIndex/*kernel.theta_index, kernel.phi_index, kernel.rho_index*/);
		// fill kernel.theta_index, kernel.phi_index, kernel.rho_index from the calculated values of kernel theta, phi, rho
		accum.get_index(kernel/*kernel.theta, kernel.phi, kernel.rho */);
		// get_index forms thetaPhiRhoIndex in kernel
		kernel.thetaPhiRhoIndex[0]/*theta_index*/ = accum.fix_theta(kernel.thetaPhiRhoIndex[0]/*theta_index*/,(int) kernel.thetaPhiRhoIndex[1]/*phi_index*/);
		kernel.kernel_load_parameters(max_point_distance);
		used_kernels.add(kernel);
	}

	/**
	 * Recursively search nodes for kernel calculation. If 'root', the passed node, is coplanar, perform kernel_calculation,
	 * otherwise, if not a leaf node recursively call this method on the children of 'root'.
	 * In effect, recursively traverse octree until we find region marked as coplanar, then call kernel_calculation which
	 * creates a new kernel_t, assigns octree node to it, computes normals in node, does process_limits, get_index in
	 * accumulator with kernel theta,phi,rho indexes and values, does a kernel_load_parameters on kernel, and finally
	 * adds new kernel to used_kernels.
	 * @param root
	 * @param accumulator
	 * @param used_kernels
	 * @param max_point_distance
	 */
	private static void kernels(octree_t root, accumulatorball_t accumulator , ArrayList<kernel_t> used_kernels, double max_point_distance) {
		if (root.coplanar) {
			kernel_calculation(root, accumulator, used_kernels, max_point_distance);
		} else // If not leaf
			if (root.m_children != null) {
				// Call children
				for (short i = 0; i < 8 ; i++) {
					kernels(root.m_children[i], accumulator , used_kernels, max_point_distance);
				}
			}
	}

}
