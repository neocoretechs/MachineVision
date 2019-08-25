package com.neocoretechs.machinevision.test;

import java.io.DataOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import com.neocoretechs.machinevision.hough3d.Vector4d;
import com.neocoretechs.machinevision.hough3d.hough_settings;
import com.neocoretechs.machinevision.hough3d.octree_t;
import com.neocoretechs.machinevision.hough3d.reader_file;
import com.neocoretechs.machinevision.hough3d.writer_file;

public class GenNav {
	 private static boolean DEBUG = true;

	public static void main(String[] args) {
			octree_t father = new octree_t();
			reader_file rf = null;
			if(args.length > 0)
				rf = new reader_file(args[0]);
			else
				rf = new reader_file(null);
			if(!rf.load_point_cloud(father)) {
				System.out.println("File failed to load, so thats it");
				return;
			}
			if( DEBUG ) {
				System.out.println("Loaded "+father.m_points.size()+" points...");
			}
			// computed centroid in load_point_cloud
			//Vector4d centroid = father.m_centroid.divide(father.m_points.size());
			hough_settings.s_level = 5;
			hough_settings.min_isotropy = .15;
			  long startTime = System.currentTimeMillis();
			  long totalTime = startTime;
			if( DEBUG) {
				System.out.println("Subdivide...");
			}
		   // Subdividing Procedure
		   father.subdivide();
		   if( DEBUG )
			   System.out.println("Elapsed Subdivide took "+(System.currentTimeMillis()-startTime)+" ms.");
		   ArrayList<octree_t> nodes = father.get_nodes();
		   final Comparator<octree_t> yComp = new Comparator<octree_t>() {         
				  @Override         
				  public int compare(octree_t jc1, octree_t jc2) {             
					      return (jc2.getMiddle().y < jc1.getMiddle().y ? -1 :                     
					              (jc2.getMiddle().y == jc1.getMiddle().y ? 0 : 1));           
				  }     
		   };
		   final Comparator<octree_t> xComp = new Comparator<octree_t>() {         
				  @Override         
				  public int compare(octree_t jc1, octree_t jc2) {             
					      return (jc2.getMiddle().x < jc1.getMiddle().x ? -1 :                     
					              (jc2.getMiddle().x == jc1.getMiddle().x ? 0 : 1));           
				  }     
		   }; 
		   Collections.sort(nodes, yComp);
		   DataOutputStream dos = null;
		   File f = new File(hough_settings.file+"NavLine"+hough_settings.extension);
		   try {
			   dos = new DataOutputStream(new FileOutputStream(f));
			   double y = nodes.get(0).getMiddle().y;
			   int iPosStart = 0;
			   int iPosEnd = 0;
			   for(int i = 0 ; i < nodes.size(); i++) {
				   if( y != nodes.get(i).getMiddle().y) {
					   iPosEnd = i;
					   genRow(dos, iPosStart, iPosEnd, nodes);
					   iPosStart = i;
					   y = nodes.get(i).getMiddle().y;
				   }
			   }
			   iPosEnd = nodes.size();
			   genRow(dos, iPosStart, iPosEnd, nodes);
			   // col and min depth marker
			   Collections.sort(nodes, xComp);
			   double x = nodes.get(0).getMiddle().x;
			   iPosStart = 0;
			   iPosEnd = 0;
			   for(int i = 0 ; i < nodes.size(); i++) {
				   if( x != nodes.get(i).getMiddle().x) {
					   iPosEnd = i;
					   genColHist(dos, iPosStart, iPosEnd, nodes);
					   iPosStart = i;
					   x = nodes.get(i).getMiddle().x;
				   }
			   }
			   iPosEnd = nodes.size();
			   genColHist(dos, iPosStart, iPosEnd, nodes);
		   } catch (FileNotFoundException e) {
				e.printStackTrace();
				return;  
		  } catch (IOException e) {
			e.printStackTrace();
		  } finally {
				try {
					if( dos != null ) {
						dos.flush();
						dos.close();
					}
				} catch (IOException e) {}		
		  }
		   
	}
	
	public static void genRow(DataOutputStream dos, int yStart, int yEnd, ArrayList<octree_t> nodelA) throws IOException{
		if(DEBUG)
			System.out.println("genRow from="+yStart+" to="+yEnd);
			List<octree_t> subNodes = nodelA.subList(yStart, yEnd);
			final Comparator<octree_t> xComp = new Comparator<octree_t>() {         
					  @Override         
					  public int compare(octree_t jc1, octree_t jc2) {             
						      return (jc2.getMiddle().x < jc1.getMiddle().x ? -1 :                     
						              (jc2.getMiddle().x == jc1.getMiddle().x ? 0 : 1));           
					  }     
			}; 
			Collections.sort(subNodes, xComp);
			// sorted now by x and y
			for(int i = 0; i < subNodes.size(); i++) {
				if( i < subNodes.size()-1) {
					writer_file.line3D(dos, (int)subNodes.get(i).getCentroid().x, (int)subNodes.get(i).getCentroid().y, (int)subNodes.get(i).getCentroid().z*10,
							(int)subNodes.get(i+1).getCentroid().x, (int)subNodes.get(i+1).getCentroid().y, (int)subNodes.get(i+1).getCentroid().z*10,
							0, 255, 255);
				}
			}
	
	}
	public static void genColHist(DataOutputStream dos, int xStart, int xEnd, ArrayList<octree_t> nodelA) throws IOException{
		if(DEBUG)
			System.out.println("genColHist from="+xStart+" to="+xEnd);
			List<octree_t> subNodes = nodelA.subList(xStart, xEnd);
			final Comparator<octree_t> yComp = new Comparator<octree_t>() {         
					  @Override         
					  public int compare(octree_t jc1, octree_t jc2) {             
						      return (jc2.getMiddle().y < jc1.getMiddle().y ? -1 :                     
						              (jc2.getMiddle().y == jc1.getMiddle().y ? 0 : 1));           
					  }     
			}; 
			Collections.sort(subNodes, yComp);
			double zMin = Double.MAX_VALUE;
			octree_t cnode = null;
			// sorted now by x and y
			for(int i = 0; i < subNodes.size(); i++) {
				if( i < subNodes.size()-1) {
					writer_file.line3D(dos, (int)subNodes.get(i).getCentroid().x, (int)subNodes.get(i).getCentroid().y, (int)subNodes.get(i).getCentroid().z*10,
							(int)subNodes.get(i+1).getCentroid().x, (int)subNodes.get(i+1).getCentroid().y, (int)subNodes.get(i+1).getCentroid().z*10,
							0, 255, 255);
				}
				if(subNodes.get(i).getCentroid().z < zMin) {
					zMin = subNodes.get(i).getCentroid().z;
					cnode = subNodes.get(i);
				}
			}
			//  min depth
			Vector4d cen1 = new Vector4d();
			Vector4d cen2 = new Vector4d();
			cen1.x = cnode.getCentroid().x - (cnode.getSize()/2);
			cen1.y = cnode.getCentroid().y - (cnode.getSize()/2);
			cen1.z = zMin*10;//cnode.getCentroid().z - (cnode.getSize()/2);
			cen2.x = cnode.getCentroid().x + (cnode.getSize()/2);
			cen2.y = cnode.getCentroid().y + (cnode.getSize()/2);
			cen2.z = zMin*10;//cnode.getCentroid().z + (cnode.getSize()/2);
			// xmin, ymin, xmax, ymin
			writer_file.line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen1.y, (int)cen1.z, 0, 255, 255);
			// xmax, ymin, xmax, ymax
			writer_file.line3D(dos, (int)cen2.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, 0, 255, 255);
			// xmax, ymax, xmin, ymax
			writer_file.line3D(dos, (int)cen2.x, (int)cen2.y, (int)cen2.z, (int)cen1.x, (int)cen2.y, (int)cen2.z, 0, 255, 255);
			// xmin, ymax, xmin, ymin
			writer_file.line3D(dos, (int)cen1.x, (int)cen2.y, (int)cen2.z, (int)cen1.x, (int)cen1.y, (int)cen1.z, 0, 255, 255);
	
	}
	
}
