package com.neocoretechs.machinevision;

import java.util.ArrayList;
import java.util.List;

import com.neocoretechs.machinevision.hough3d.accumulatorball_t;
import com.neocoretechs.machinevision.hough3d.bin_t;
import com.neocoretechs.machinevision.hough3d.hough_settings;
import com.neocoretechs.machinevision.hough3d.octree_t;
/**
 * Build the accumulator ball up to the collection of accum_cell_t in accumulatorball_t
 * such that we can do a nearest neighbors. Neighborhood size is typically 27
 * ArrayList<accum_cell_t> get_neighbors( double theta_index, int phi_index, int rho_index, int neighborhood_size )
 * @author jg
 *
 */
public class AccumulatorBall {
	private static boolean DEBUG = true;

	public AccumulatorBall(List<octree_t> node) {
		long startTime = 0;
		if( DEBUG ) {
			System.out.println("Build accumulator...");
			startTime = System.currentTimeMillis();
		}
	   accumulatorball_t accum = new accumulatorball_t(hough_settings.max_point_distance, hough_settings.rho_num, hough_settings.phi_num);

		ArrayList<double[]> kernels = new ArrayList<double[]>();
		ArrayList<bin_t> used_bins = new ArrayList<bin_t>();
		for(octree_t inode: node) {
			kernel_calculation(inode, accum, kernels);
		}
		for(double[] k : kernels) {
		      ballBuild3(k, accum, used_bins); 
		}
		if( DEBUG) {
			System.out.println("Elapsed accumulator build took "+(System.currentTimeMillis()-startTime)+" ms.");
		}
	}
	private static boolean ballBuild1(accumulatorball_t accum, double[] kernel, ArrayList<bin_t> used_bins, 
            double[] tpr, // theta_index, int phi_index, int rho_start_index, 
            double theta, double phi){
		int rho_index,p, inc_rho_index;
		double rho, t, inc_rho, gauss;
		boolean voted = false;
		double votes;
		p = (int) tpr[1];//phi_index
		t = tpr[0];//theta_index;
		//double[] theta_phi_index = new double[]{tpr[0]/*theta_index*/, tpr[1]/*phi_index*/, 0.0};
		inc_rho = accum.m_delta_rho;
		inc_rho_index = 1;
		// RHO array the direction "positive"
		for (rho_index = (int) tpr[2]/*rho_start_index*/, rho = 0.0 ;; rho_index += inc_rho_index, rho += inc_rho) {
			if (rho_index < 0) inc_rho_index *= -1;
			double[] tprx = new double[]{t, p, rho_index};
			if (!accum.process_rho(tprx)) {//if (accum.process_rho(t, p, rho_index) == false) break;
				t = tprx[0]; p = (int) tprx[1]; rho_index = (int) tprx[2];
				break;
			}
			t = tprx[0]; p = (int) tprx[1]; rho_index = (int) tprx[2];
			//gauss = kernel.trivariated_gaussian_dist_normal(rho, phi, theta);
			//if( DEBUG ) {
			//	System.out.println("voting gaussian_vote_1d RHO DIR POS theta index="+t+" phi index="+p+" rho index="+rho_index+" gauss="+gauss+" will cast vote="+(gauss < kernel.voting_limit));
			//}
			//if (gauss < kernel.voting_limit) 
			//	break;
			//if ((votes = gauss) >= 0.0) {
			//	voted = cast_vote(used_bins, accum.at(t, (int)p, (int)rho_index), kernel, votes,t, p, rho_index);
			//} else 
			//	break;
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
			//gauss = kernel.trivariated_gaussian_dist_normal(rho, phi, theta);
			//if( DEBUG ) {
			//	System.out.println("voting gaussian_vote_1d RHO DIR NEG theta index="+t+" phi index="+p+" rho index="+rho_index+" gauss="+gauss+" will cast vote="+(gauss < kernel.voting_limit));
			//}
			//if (gauss < kernel.voting_limit)
			//	break;
			//if ((votes = gauss) >= 0.0) {
			//	voted = cast_vote(used_bins, accum.at(t, (int)p,(int)rho_index), kernel, votes, t, p, rho_index);
			//} else 
			//	break;
		}
		if( DEBUG )
			System.out.println("voting gaussian_vote_1d returning voted="+voted);
		return voted;
	}
	private static void ballBuild2( accumulatorball_t accum, double[] kernel, ArrayList<bin_t> used_bins, 
					double[] tpr,//double theta_start_index, int phi_start_index, int rho_start_index,
					double phi_start, int inc_phi_index){
		boolean voted;
		boolean voting_ended = false;
		double inc_phi = (double)inc_phi_index * accum.m_delta_angle;
		//int phi_index, rho_index = rho_start_index;
		double /*theta_index,*/ theta, phi, theta_init;
		double[] theta_phi_index = new double[]{tpr[0],tpr[1],tpr[2]};
		//  PHI array in the direction "inc_phi_index"
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
			//the THETA array in the direction "positive"
			for(curr_theta_index = theta_init, theta = 0.0 ;; curr_theta_index+=inc_theta_index, theta += inc_theta) {      
				curr_theta_index = accum.process_theta(curr_theta_index); 
				accum.initialize(curr_theta_index, (int) theta_phi_index[1]/*phi_index*/);
				double[] currThetaPhi = new double[]{curr_theta_index, theta_phi_index[1], theta_phi_index[2]};
				voted = ballBuild1(accum, kernel, used_bins, currThetaPhi/*curr_theta_index,(int)phi_index,(int)rho_index*/, theta, phi);
				if (voted) {
					voting_ended = false;
				} else 
					break;
			}
			// in the THETA array in the direction "negative"
			for(curr_theta_index = theta_init - inc_theta_index, theta = -inc_theta;; curr_theta_index -= inc_theta_index, theta -= inc_theta) {
				curr_theta_index = accum.process_theta(curr_theta_index);
				accum.initialize(curr_theta_index, (int) theta_phi_index[1]/* phi_index*/);
				double[] currThetaPhi = new double[]{curr_theta_index, theta_phi_index[1], theta_phi_index[2]};
				voted = ballBuild1(accum, kernel, used_bins, currThetaPhi/*curr_theta_index, (int)phi_index, (int) rho_index*/, theta, phi);
				if (voted) {
					voting_ended = false;
				} else 
					break;
			}
		}
	}

	private static void ballBuild3(double[] kernel, accumulatorball_t accum, ArrayList<bin_t> used_bins){
		//if(kernel.thetaPhiRhoIndex[2] < 0)
		//	return;
		// at affects accum_ball_cell_t list m_data in accumulatorball_t
		accum.at(kernel[0]/*theta_index*/, (int)kernel[1]/*phi_index*/,(int)kernel[2]/*rho_index*/);
		ballBuild2(accum, kernel, used_bins, kernel/* kernel.theta_index, kernel.phi_index, kernel.rho_index*/, 0, +1);
		//int phi_index = kernel.phi_index-1;
		//double theta_index = kernel.theta_index;
		double[] theta_phi_index = new double[]{kernel[0], kernel[1]-1, kernel[2]};
		accum.process_phi(theta_phi_index/*theta_index, phi_index*/);
		ballBuild2(accum, kernel, used_bins, theta_phi_index, -accum.m_delta_angle, -1);
	}
	
	private static void kernel_calculation(octree_t node, accumulatorball_t accum, List<double[]> kernels){
		if ((node.getCentroid().Normalized().and(node.getNormal1())) < 0)  
			node.setNormal1(node.getNormal1().multiply(-1.0));
		double[] tpr = octree_t.cartesian_to_spherical(node.getCentroid());
		// theta = 0, phi = 1, rho = 2
		// fill kernel.thetaPhiRhoIndex from the calculated values of kernel theta, phi, rho
		tpr = accum.get_index(tpr[0], tpr[1], tpr[2]);
		if( accum.process_limits(tpr/*kernel.theta_index, kernel.phi_index, kernel.rho_index*/)) {
			// get_index forms thetaPhiRhoIndex in kernel
			tpr[0]/*theta_index*/ = accum.fix_theta(tpr[0]/*theta_index*/,(int) tpr[1]/*phi_index*/);
		}
		kernels.add(tpr);
	}

}
