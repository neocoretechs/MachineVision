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
   double mix = Double.MAX_VALUE;
   double miy = Double.MAX_VALUE;
   double miz = Double.MAX_VALUE;
   double max = Double.MIN_VALUE;
   double may = Double.MIN_VALUE;
   double maz = Double.MIN_VALUE;
   String fileName;
   public reader_file(String fil) {
	fileName = fil;
   }
/**
   * Read CloudCompare point cloud viewer compatible file
   * for each point - X,Y,Z,R,G,B ascii delimited by space
   * the processed array chunks of [x][y][0]=R, [x][y][1]=G, [x][y][2]=B, [x][y][3]=D
   */
  void read_file(hough_settings settings, octree_t node) {
   BufferedReader dis = null;
   int point_num = 0;
   //file.open(settings.file + settings.extension);
   File f = new File(settings.file+fileName+settings.extension);
   if( !f.exists() ) {
	   f = new File(settings.file+settings.extension);
	   if(!f.exists()) {
		   if( fileName != null )
			   f = new File(fileName);
		   throw new RuntimeException("Cant find any permutation of file:"+f);
	   }
   }
   System.out.println(f.getPath()+" isfile?="+f.isFile());
   try {
		dis = new BufferedReader(new FileReader(f));
		String inLine;
		while((inLine = dis.readLine()) != null) {
			String[] splitLine = inLine.split(" ");
			Vector4d point = new Vector4d(Double.parseDouble(splitLine[0]),Double.parseDouble(splitLine[1]),Double.parseDouble(splitLine[2]));
			Vector4d color = new Vector4d(Double.parseDouble(splitLine[3]),Double.parseDouble(splitLine[4]),Double.parseDouble(splitLine[5]));
			node.m_points.add(point);
			node.m_centroid.add(point);
			node.m_indexes.add(point_num++);
			node.m_colors.add(color.divide(255.0));
			mix = Math.min(mix,point.x);
			miy = Math.min(miy,point.y);
			miz = Math.min(miz,point.z);
			max = Math.max(max,point.x);
			may = Math.max(may,point.y);
			maz = Math.max(maz,point.z);
		}
	} catch (FileNotFoundException e) {
			e.printStackTrace();
			return;
	} catch (IOException e) {
			e.printStackTrace();
	} finally {
		try {
				if( dis != null ) {
					dis.close();
				}
		} catch (IOException e) {}		
	}
  }
 /**
  * Create root node and read file.
  * @param settings
  * @param node
  */
  void load_point_cloud(hough_settings settings, octree_t node) {
   System.out.print("Loading Point Cloud...");
   node.m_middle.set(new Vector4d(0,0,0));
   node.m_level = 0;
   node.m_root = node;
   read_file(settings, node); 
   System.out.println("Size: "+node.m_points.size()+" min="+mix+","+miy+","+miz+" max="+max+","+may+","+maz);
   //settings.s_ms = settings.s_ps * node.m_points.size();
  }
}
