package com.neocoretechs.machinevision.hough3d;
// A structure that stores the coordinates of an accumulator cell
public class bin_t implements Comparable{
  double theta_index;
  short phi_index;
  short rho_index;
  float votes;
   public bin_t(){}
   public bin_t(double theta, short phi, short rho) {
	this.theta_index = theta;
	this.phi_index = phi;
	this.rho_index = rho;
   }

  // bool const operator < (const bin_t bin) const { return (votes > bin.votes); }
 
@Override
public int compareTo(Object o) {
	if(votes == ((bin_t)o).votes) return 0;
	if(votes > ((bin_t)o).votes) return 1;
	return -1;
}
}