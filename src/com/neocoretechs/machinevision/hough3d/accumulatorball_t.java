package com.neocoretechs.machinevision.hough3d;

import java.util.ArrayList;
/**
 * http://www.inf.ufrgs.br/~oliveira/pubs_files/HT3D/Limberger_Oliveira_3D_HT_Pre-Print_low_res.pdf
 * @author jg
 *
 */
public class accumulatorball_t {
	static final short[] offset_x = {0, 0,  0,  0,  0, +1, -1,      +1, -1, +1, -1, +1, -1, -1, +1,  0,  0,  0,  0,     +1, +1, +1, -1, +1, -1, -1, -1,};
    static final short[] offset_y = {0  +1, -1,  0,  0,  0,  0,      +1, -1, -1, +1,  0,  0,  0,  0, +1, -1, -1, +1,     +1, +1, -1, +1, -1, -1, +1, -1,};
    static final short[] offset_z = {0, 0,  0, +1, -1,  0,  0,       0,  0,  0,  0, +1, -1, +1, -1, +1, -1, +1, -1,     +1, -1, +1, +1, -1, +1, -1, -1,};
	private static final boolean DEBUG = false;

  int neighbors_size;

   double m_theta_max;
   double m_phi_max;

   short m_phi_length;         // Accumulator size (phi dimension)
   short m_phi_length_half;
   short m_rho_length;         // Accumulator size (rho dimension)
   short m_theta_length;       // Accumulator size (theta dimension)

   double m_delta_rho;         // Discretization step to the distance (rho)
   double m_delta_angle;       // Discretization step to the angles (theta & phi)

   ArrayList<ArrayList<accum_ball_cell_t>> m_data = new ArrayList<ArrayList<accum_ball_cell_t>>();
   /**
    * 
    * @param max_distance
    * @param rho_num
    * @param phi_num
    */
   public accumulatorball_t(double max_distance, int rho_num, int phi_num) {
   	neighbors_size = 27;
	m_theta_max = Math.PI*2;
	m_phi_max = Math.PI;

	m_rho_length = (short) rho_num;
	m_phi_length = (short) phi_num;
	m_phi_length_half = (short) (phi_num / 2);
	m_data.ensureCapacity(m_phi_length + 1);
	m_delta_angle = Math.PI / (double)m_phi_length;
	m_delta_rho = max_distance / (double)rho_num;
	/*
	for (int p = 0; p <= m_phi_length; p++)
	{
		double m_phi = (double)p / (double)m_phi_length * m_phi_max;
		int length = Math.max(1, (int)(Math.round((double)m_phi_length*2.0 * Math.sin(m_phi))));
		m_data[p].resize(length, NULL);
	}
	*/
	}


   /**
   * Assume monotonically increasing phi_index that supports ArrayList add to end vs insert at phi_index
   */
   void initialize(double theta_index, int phi_index) {
      int t = get_theta_index(theta_index, phi_index);
      ArrayList<accum_ball_cell_t> tempCell = m_data.get(phi_index);
      if(t == -1 || tempCell == null) {
		//m_data(phi_index) = 
    	ArrayList<accum_ball_cell_t> tempAccum = new ArrayList<accum_ball_cell_t>();
    	m_data.add(phi_index, tempAccum);
    	if( t == -1 )
    		tempAccum.add(new accum_ball_cell_t(m_rho_length));
    	else
    		// m_data[phi_index][t] =  new accum_ball_cell_t(m_rho_length);
    		tempAccum.add(t, new accum_ball_cell_t(m_rho_length));
      }
   }

   boolean visited_neighbor(double theta_index, short phi_index, short rho_index) {
      ArrayList<accum_cell_t> neighbors = get_neighbors(theta_index,phi_index,rho_index, 27);
      for (accum_cell_t cell : neighbors) {
         if (cell.visited) return true;
      }
      return false;
   }

   ArrayList<octree_t> convolution_nodes(double theta_index, short phi_index, short rho_index) {
      ArrayList<octree_t> nodes = new ArrayList<octree_t>();
      ArrayList<accum_cell_t> neighbors = get_neighbors(theta_index, phi_index, rho_index, 27);
      for (accum_cell_t cell : neighbors) {
         for (octree_t n : cell.ref_node) {
        	 if( !nodes.contains(n) )
               nodes.add(n);
         }
      }
      return nodes;
   }
   
   float convolution_value(double theta_index, int phi_index, int rho_index) {
      float acc_value = 0;
      ArrayList<accum_cell_t> neighbors = get_neighbors(theta_index, (short)phi_index, (short)rho_index, 5);
      
      acc_value = (float) (neighbors.get(0).bin * 0.2);
      for (int i = 1; i < neighbors.size(); i++) {
         acc_value += neighbors.get(i).bin * 0.1333;
      }
      return acc_value;
   }

   double normalization_factor(int phi_index) {
      return 360.0 / (double)m_data.get(phi_index).size();
   }

   void set_visited(double theta_index, short phi_index, short rho_index) {
      ArrayList<accum_cell_t> neighbors = get_neighbors(theta_index, phi_index, rho_index, 27);
      for (accum_cell_t cell : neighbors) {
            cell.visited = true;
      }
   }

   double delta_theta(int phi_index) {
      return m_theta_max / (double)(m_data.get(phi_index).size());
   }

   double delta_theta_index(int phi_index)
   {
      return 1.0 / (double)(m_data.get(phi_index).size());
   }

   void process_theta(double[] theta_phi_index /*double theta_index*/) {
      //return (theta_index+1.0) - (int)(theta_index+1.0);
	   theta_phi_index[0] = (theta_phi_index[0]+1.0) - (int)(theta_phi_index[0]+1.0);
   }

   boolean process_phi(double[] theta_phi_index /*double theta_index, int phi_index*/) {
      /*theta_index*/ process_theta(theta_phi_index/*theta_index*/);  
      if (theta_phi_index[1] /*phi_index*/ < 0) {
         theta_phi_index[1] = Math.abs(theta_phi_index[1]);
         /*theta_index*/ theta_phi_index[0] = (theta_phi_index[0]+0.5) - (int)((theta_phi_index[0]+0.5));
         return true;
      }
      else if (/*phi_index*/ theta_phi_index[1] > m_phi_length) {
         theta_phi_index[1] = m_phi_length + m_phi_length - theta_phi_index[1];
         /*theta_index*/ theta_phi_index[0] = (theta_phi_index[0]+0.5) - (int)((theta_phi_index[0]+0.5));
         return true;
      }
      return false;
   }

   boolean process_rho(double[] theta_phi_index /*double theta_index, int phi_index, int rho_index*/) {
      if (/*rho_index*/theta_phi_index[2] < 0) {
         theta_phi_index[2] = Math.abs(theta_phi_index[2]);
         /*phi_index*/ theta_phi_index[1] = m_phi_length - theta_phi_index[1];
         /*theta_index*/theta_phi_index[0] = (theta_phi_index[0]+0.5) - (int)((theta_phi_index[0]+0.5));
         return true;
      } else
    	  if (/*rho_index*/theta_phi_index[2] >= m_rho_length) { 
    		  return false;
    	  }
      return true;
   }

   
   ArrayList<accum_cell_t> get_neighbors( double theta_index, short phi_index, short rho_index, int neighborhood_size ){
      ArrayList<accum_cell_t> result = new ArrayList<accum_cell_t>(neighborhood_size);
      int p, r;
      double t, theta;

      if (phi_index==0 || phi_index==m_phi_length) 
		theta = 0.5;
      else 
		theta = theta_index;
      // we are going to be changing the references in theta_phi_index inside the 'process_' methods
      double[] theta_phi_index = new double[3];
      //  center[1]   /  direct-linked[7]  semi-direct-linked[19] diagonal-linked[27] 
      for (short i=0; i != neighborhood_size; ++i){
         //t = theta;
         //p = phi_index + offset_y[i];
         //r = rho_index + offset_z[i];
          theta_phi_index[0] = theta;
          theta_phi_index[1] = phi_index + offset_y[i];
          theta_phi_index[2] = rho_index + offset_z[i];
         //process_phi(t,p);
          process_phi(theta_phi_index);
         //t = fix_theta(t,p);
          theta_phi_index[0] = fix_theta(theta_phi_index[0], (int) theta_phi_index[1]);
         //t += delta_theta_index(p) * (double)(offset_x[i]);
          theta_phi_index[0] += delta_theta_index((int) theta_phi_index[1]) * (double)(offset_x[i]);
         //if (process_limits(t,p,r) == false) continue;
         if( !process_limits(theta_phi_index) )
        	 continue;
        accum_cell_t cell = at(theta_phi_index[0], (short)theta_phi_index[1], (short)theta_phi_index[2] /*t,(short)p,(short)r*/);
         if (!result.contains(cell)) {
            result.add(cell);
         }
      } 
      return result;
   }

   boolean process_limits(double[] theta_phi_index /*double theta_index, int phi_index, int rho_index*/) {
      process_phi(theta_phi_index /*theta_index, phi_index*/);
      return process_rho(theta_phi_index);
   }
   
   accum_cell_t at(double theta, short phi, short rho) {
      int t = get_theta_index(theta,phi);
      ArrayList<accum_ball_cell_t> tempCell = m_data.get(phi);
      if(t == -1 || tempCell == null) {
		//m_data(phi_index) = 
    	tempCell = new ArrayList<accum_ball_cell_t>();
    	m_data.add(phi, tempCell);
    	if( t == -1 ) {
    		accum_ball_cell_t tempAbc = new accum_ball_cell_t(m_rho_length);
    		tempCell.add(tempAbc);
    		return tempAbc.bins[rho];
    	}
    	// m_data[phi_index][t] =  new accum_ball_cell_t(m_rho_length);
    	tempCell.add(t, new accum_ball_cell_t(m_rho_length));
      }
      //if(t == -1 || m_data.get(phi).get(t) == null)
      //   m_data.get(phi).get(t).set(new accum_ball_cell_t(m_rho_length));
      return tempCell.get(t).bins[rho];
   }
	/**
	* do a get from m_data for phi index, if null return -1 else multiply theta by size of phi index and round
	*/
   int get_theta_index(double theta, int phi_index){
	ArrayList<accum_ball_cell_t> tempAccum = m_data.get(phi_index);
	if( tempAccum == null )
		return -1;
	return ((int)(Math.round(theta * (double)(tempAccum.size()))) % tempAccum.size());
   }

   void get_index(kernel_t kernel) {
      kernel.theta_index = kernel.theta/Math.PI*2 + 0.5;
      kernel.phi_index = (int) Math.round(kernel.phi / m_delta_angle);
      kernel.rho_index =  (int) Math.round(kernel.rho / m_delta_rho);
   }

   void get_values(plane_t plane, double theta_index, int phi_index, int rho_index) {
      plane.m_theta = (theta_index-0.5) * Math.PI*2;
      plane.m_phi =   (double)(phi_index) * m_delta_angle;
      plane.m_rho =   (double)(rho_index) * m_delta_rho;
   }

   void spherical_to_cartesian(Vector4d normal, double theta, double phi, double rho){
      normal.x = Math.sin(phi) * Math.cos(theta) * rho;
      normal.y = Math.sin(phi) * Math.sin(theta) * rho;
      normal.z = Math.cos(phi) * rho;
   }

   double fix_theta(double theta, int phi) {
      double p_size = m_data.get(phi).size();
      int t = (int) Math.round((theta) * p_size);
      return (t==1)?(0.0):t/p_size;
   }

}
