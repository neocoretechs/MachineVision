package com.neocoretechs.machinevision.hough3d;
/**
 * A structure that stores the spherical coordinates of an accumulator cell and the votes it has cast.
 * @author jg
 *
 */
public class bin_t implements Comparable{
  double theta_index;
  int phi_index;
  int rho_index;
  double votes;
   //public bin_t(){}
  /**
   * 
   * @param theta Theta Index
   * @param phi Phi Index
   * @param rho Rho Index
   */
   public bin_t(double theta, int phi, int rho) {
	   this.theta_index = theta;
	   this.phi_index = phi;
	   this.rho_index = rho;
	   votes = 0;
   }
  // bool const operator < (const bin_t bin) const { return (votes > bin.votes); }
   @Override
   public int compareTo(Object o) {
	   if(votes == ((bin_t)o).votes) return 0;
	   if(votes > ((bin_t)o).votes) return 1;
	   return -1;
   }
   @Override
   public String toString() {
	   return "bin_t theta_index="+theta_index+" phi_index="+phi_index+" rho_index="+rho_index+" votes="+votes;
   }
}