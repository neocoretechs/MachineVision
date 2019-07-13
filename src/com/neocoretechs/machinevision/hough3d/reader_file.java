package com.neocoretechs.machinevision.hough3d;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
/**
 * Read file and prepare dataset for detection of planar regions.
 * @author jg
 *
 */
public class reader_file {

   String fileName;
   private boolean DEBUG = false;
   /**
    * 
    * @param fil
    */
   public reader_file(String fil) {
	fileName = fil;
   }
/**
   * Read CloudCompare point cloud viewer compatible file
   * for each point - X,Y,Z,R,G,B ascii delimited by space
   * the processed array chunks of [x][y][0]=R, [x][y][1]=G, [x][y][2]=B, [x][y][3]=D
   */
  private boolean read_file(octree_t node) {
   BufferedReader dis = null;
   //file.open(settings.file + settings.extension);
   File f = new File(hough_settings.file+fileName+hough_settings.extension);
   if( !f.exists() ) {
	   f = new File(hough_settings.file+hough_settings.extension);
	   if(!f.exists()) {
		   if( fileName != null )
			   f = new File(fileName);
		   //throw new RuntimeException
		   System.out.println("Cant find any permutation of file:"+f);
		   return false;
	   }
   }
   System.out.println(f.getPath()+" isfile?="+f.isFile());
   try {
		dis = new BufferedReader(new FileReader(f));
		String inLine;
		while((inLine = dis.readLine()) != null) {
			String[] splitLine = inLine.split(" ");
			octree_t.build(node, Double.parseDouble(splitLine[0]),Double.parseDouble(splitLine[1]),Double.parseDouble(splitLine[2]),
					Double.parseDouble(splitLine[3]),Double.parseDouble(splitLine[4]),Double.parseDouble(splitLine[5]));
		}
	} catch (FileNotFoundException e) {
			e.printStackTrace();
			return false;
	} catch (IOException e) {
			e.printStackTrace();
			return false;
	} finally {
		try {
				if( dis != null ) {
					dis.close();
				}
		} catch (IOException e) {}		
	}
   octree_t.buildEnd(node);
   return true;
  }
 /**
  * Create root node and read file.
  * @param settings
  * @param node
  */
  public boolean load_point_cloud(octree_t node) {
   System.out.print("Loading Point Cloud...");
   octree_t.buildStart(node);
   boolean rf = read_file(node); 
   System.out.println("Size: "+node.m_points.size()+" min="+octree_t.mix+","+octree_t.miy+","+octree_t.miz+" max="+octree_t.max+","+octree_t.may+","+octree_t.maz);
   //settings.s_ms = settings.s_ps * node.m_points.size();
   return rf;
  }
}
