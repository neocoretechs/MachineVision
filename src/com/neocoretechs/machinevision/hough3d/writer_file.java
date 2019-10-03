package com.neocoretechs.machinevision.hough3d;

import java.io.DataOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class writer_file {
	private static boolean DEBUG = false;

	/**
	 * Write CloudCompare point cloud viewer compatible file
	 * for each point - X,Y,Z,R,G,B ascii delimited by space
	 * @param planes the processed array chunks
	 */
	public static void writePlanes(ArrayList<plane_t> planes) {
		DataOutputStream dos = null;
		File f = new File(hough_settings.file+"houghout"+hough_settings.extension);
		try {
			dos = new DataOutputStream(new FileOutputStream(f));
			for(plane_t p : planes) {
				for(octree_t o : p.nodes) {
					writeChildren(o, dos);
				}
			}
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			return;
		} finally {
			try {
				if( dos != null ) {
					dos.flush();
					dos.close();
				}
			} catch (IOException e) {
			}		
		}
		
	}
	/**
	 * does get_nodes, which does the recursion and only adds coplanar nodes. Output only coplanar nodes
	 * @param node
	 */
	public static void writeChildren(octree_t node) {
		DataOutputStream dos = null;
		File f = new File(hough_settings.file+"octree"+hough_settings.extension);
		ArrayList<octree_t> nodes = new ArrayList<octree_t>();
		node.get_nodes(nodes);
		int totalPoints = 0;
		try {
			dos = new DataOutputStream(new FileOutputStream(f));
				for(octree_t oct : nodes) {
					if( oct.m_level < hough_settings.s_level)
						continue;
					totalPoints+=oct.m_indexes.size();
					if( DEBUG )
						System.out.println("writer_file writeChildren writing octree node "+oct);
					  for(int i = 0; i < oct.m_indexes.size() ; i++) {
							dos.writeBytes(String.valueOf( oct.m_root.m_points.get(oct.m_indexes.get(i)).x));
							dos.writeByte(' ');
							dos.writeBytes(String.valueOf( oct.m_root.m_points.get(oct.m_indexes.get(i)).y));
							dos.writeByte(' ');
							dos.writeBytes(String.valueOf(oct.m_root.m_points.get(oct.m_indexes.get(i)).z)); // Z
							dos.writeByte(' ');
							dos.writeBytes(String.valueOf(255));
							dos.writeByte(' ');
							dos.writeBytes(String.valueOf(oct.m_level*10));
							dos.writeByte(' ');
							dos.writeBytes(String.valueOf(oct.m_level*10));
							dos.writeByte('\r');
							dos.writeByte('\n');
						 }
				}
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
			} catch (IOException e) {
			}		
		}
		if( DEBUG ) {
			System.out.println("***octree_t debug write "+f.getPath()+" coplanar regions="+nodes.size()+" total points="+totalPoints);
		}
		
	}
	
	public static void writeChildren(octree_t oct, DataOutputStream dos) {
		try {
		  for(int i = 0; i < oct.m_indexes.size() ; i++) {
			dos.writeBytes(String.valueOf( oct.m_root.m_points.get(oct.m_indexes.get(i)).x));
			dos.writeByte(' ');
			dos.writeBytes(String.valueOf( oct.m_root.m_points.get(oct.m_indexes.get(i)).y));
			dos.writeByte(' ');
			dos.writeBytes(String.valueOf(oct.m_root.m_points.get(oct.m_indexes.get(i)).z)); // Z
			dos.writeByte(' ');
			dos.writeBytes(String.valueOf((int)oct.color.get(0)));
			dos.writeByte(' ');
			dos.writeBytes(String.valueOf((int)oct.color.get(1)));
			dos.writeByte(' ');
			dos.writeBytes(String.valueOf((int)oct.color.get(2)));
			dos.writeByte('\r');
			dos.writeByte('\n');
		  }
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			return;
		} catch (IOException e) {
			e.printStackTrace();
		}
		if (oct.m_children != null) {
		     for (short i = 0; i < 8 ; i++) {
		         writeChildren(oct.m_children[i], dos);
		     }
		}
	}
	/**
	 * Write the eigenvectors of the octree points in each coplanar cell below the subdivide level s_level
	 * as: normal 1 white, normal2 red, and normal3 blue, with a diamond of green connecting the outside of the normal2 and normal3.
	 * The white normal at its eigenvalue and the red and blue axes projected at + and - the eigenvalue generated in octree PCA stage. 
	 * So the prospective plane is imagined by the combination of the red and blue perpendicular axes with 
	 * their green diamond outline with centroid at center, and a white 
	 * vector segment at 90 degrees to that as the principal axis of the PCA.
	 * @param node the node to begin outputting and all its children that are coplanar below s_level
	 * @throws IOException 
	 */
	public static void writePerp(octree_t node, String fileName) {
		DataOutputStream dos = null;
		File f = new File(hough_settings.file+fileName+hough_settings.extension);
		ArrayList<octree_t> nodes = new ArrayList<octree_t>();
		node.get_nodes(nodes);
	  try {
		dos = new DataOutputStream(new FileOutputStream(f));
		Vector4d cen1 = new Vector4d();
		Vector4d cen2 = new Vector4d();
		Vector4d cen3 = new Vector4d();
		Vector4d cen4 = new Vector4d();
		for(octree_t cnode : nodes )  {
			if( cnode.m_level < hough_settings.s_level)
				continue;
			// spherical of centroid
			genPerp(cnode, cen1, cen2, cen3, cen4);
			line3D(dos, (int)cnode.m_centroid.x, (int)cnode.m_centroid.y, (int)cnode.m_centroid.z, (int)cen1.x, (int)cen1.y, (int)cen1.z, 255, 255, 255);
			// axis of middle variance, perp to normal
			line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, 255, 0, 0);
			// long axis
			// generate point at end of long segment
			line3D(dos, (int)cen3.x, (int)cen3.y, (int)cen3.z, (int)cen4.x, (int)cen4.y, (int)cen4.z, 0, 0, 255);
			// outside
			line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen4.x, (int)cen4.y, (int)cen4.z, 0, 255, 0);
			line3D(dos, (int)cen4.x, (int)cen4.y, (int)cen4.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, 0, 255, 0);
			line3D(dos, (int)cen2.x, (int)cen2.y, (int)cen2.z, (int)cen3.x, (int)cen3.y, (int)cen3.z, 0, 255, 0);
			line3D(dos, (int)cen3.x, (int)cen3.y, (int)cen3.z, (int)cen1.x, (int)cen1.y, (int)cen1.z, 0, 255, 0);
		}
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
			} catch (IOException e) {
			}		
		}
	}
	
	public static void genPerp(octree_t cnode, Vector4d cen1, Vector4d cen2, Vector4d cen3, Vector4d cen4) {
		double[] tpr = octree_t.cartesian_to_spherical(cnode.normal1);
		if( DEBUG )
			System.out.println("centroid="+cnode.m_centroid+" variance1="+cnode.variance1+" tpr:"+tpr[0]+" "+tpr[1]+" "+tpr[2]);
		// generate normal to plane from centroid scaled to variance1 eigenvalue
		octree_t.spherical_to_cartesian(cen1, cnode.m_centroid, tpr[0], tpr[1], tpr[2], cnode.variance1);
		// axis of middle variance, perp to normal
		tpr = octree_t.cartesian_to_spherical(cnode.normal2);
		if( DEBUG )
			System.out.println("variance2="+cnode.variance2+" tpr:"+tpr[0]+" "+tpr[1]+" "+tpr[2]);
		// mid axis normal2
		octree_t.spherical_to_cartesian(cen1, cnode.m_centroid, tpr[0], tpr[1], tpr[2], -cnode.variance2);
		// generate point at end of segment
		octree_t.spherical_to_cartesian(cen2, cnode.m_centroid, tpr[0], tpr[1], tpr[2], cnode.variance2);
		if( DEBUG )
			System.out.println("cen1="+cen1+" "+" cen2="+cen2);
		// long axis
		tpr = octree_t.cartesian_to_spherical(cnode.normal3);
		if( DEBUG )
			System.out.println("variance3="+cnode.variance3+" tpr:"+tpr[0]+" "+tpr[1]+" "+tpr[2]);
		// subtract length of longest axis normal3
		octree_t.spherical_to_cartesian(cen3, cnode.m_centroid, tpr[0], tpr[1], tpr[2], -cnode.variance3);
		// generate point at end of long segment
		octree_t.spherical_to_cartesian(cen4, cnode.m_centroid, tpr[0], tpr[1], tpr[2], cnode.variance3);
		if( DEBUG )
			System.out.println("cen3="+cen3+" "+" cen4="+cen4);
	}
	/**
	 * Write a sloped envelope at middle +/- size
	 * @param node
	 * @param fileName
	 */
	public static void writeEnv(octree_t node, String fileName) {
		DataOutputStream dos = null;
		File f = new File(hough_settings.file+fileName+hough_settings.extension);
		double zincr = .1;
		ArrayList<octree_t> nodes = new ArrayList<octree_t>();
		node.get_nodes(nodes);
	  try {
		dos = new DataOutputStream(new FileOutputStream(f));
		Vector4d cen1 = new Vector4d();
		Vector4d cen2 = new Vector4d();
		for(octree_t cnode : nodes )  {
			if( cnode.m_level < hough_settings.s_level)
				continue;
			cen1.x = cnode.m_middle.x - (cnode.m_size/2);
			cen1.y = cnode.m_middle.y - (cnode.m_size/2);
			cen1.z = cnode.m_middle.z - (cnode.m_size/2);
			cen2.x = cnode.m_middle.x + (cnode.m_size/2);
			cen2.y = cnode.m_middle.y + (cnode.m_size/2);
			cen2.z = cnode.m_middle.z + (cnode.m_size/2);
			// xmin, ymin, xmax, ymin
			line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen1.y, (int)cen1.z, 0, 255, 255);
			// xmax, ymin, xmax, ymax
			line3D(dos, (int)cen2.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, 0, 255, 255);
			// xmax, ymax, xmin, ymax
			line3D(dos, (int)cen2.x, (int)cen2.y, (int)cen2.z, (int)cen1.x, (int)cen2.y, (int)cen2.z, 0, 255, 255);
			// xmin, ymax, xmin, ymin
			line3D(dos, (int)cen1.x, (int)cen2.y, (int)cen2.z, (int)cen1.x, (int)cen1.y, (int)cen1.z, 0, 255, 255);
		}
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
			} catch (IOException e) {
			}		
		}
	}
	public static void writeEnv(List<double[]> maxEnv, String fileName) {
		DataOutputStream dos = null;
		File f = new File(hough_settings.file+fileName+hough_settings.extension);
	  try {
		dos = new DataOutputStream(new FileOutputStream(f));
		for(double[] cenv : maxEnv)  {
			// xmin, ymin, xmax, ymin
			line3D(dos, (int)cenv[0], (int)cenv[1], 0, (int)cenv[2], (int)cenv[1], 0, 0, 255, 255);
			// xmax, ymin, xmax, ymax
			line3D(dos, (int)cenv[2], (int)cenv[1], 0, (int)cenv[2], (int)cenv[3], 0, 0, 255, 255);
			// xmax, ymax, xmin, ymax
			line3D(dos, (int)cenv[2], (int)cenv[3], 0, (int)cenv[0], (int)cenv[3], 0, 0, 255, 255);
			// xmin, ymax, xmin, ymin
			line3D(dos, (int)cenv[0], (int)cenv[3], 0, (int)cenv[0], (int)cenv[1], 0, 0, 255, 255);
		}
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
			} catch (IOException e) {
			}		
		}
	}
	/**
	 * Generate a 3d line between 2 point in given color and write to stream
	 * @param dos
	 * @param startx
	 * @param starty
	 * @param startz
	 * @param endx
	 * @param endy
	 * @param endz
	 * @param r
	 * @param g
	 * @param b
	 * @throws IOException
	 */
	public static void line3D(DataOutputStream dos, 
			int startx, int starty, int startz, 
			int endx, int endy, int endz,
			int r, int g, int b) throws IOException {
        int dx = endx - startx;
        int dy = endy - starty;
        int dz = endz - startz;

        int ax = Math.abs(dx);
        int ay = Math.abs(dy);
        int az = Math.abs(dz);
        //Queue<Coord3D> result = new ArrayDeque<>(Math.max(Math.max(ax, ay), az));

        ax <<= 1;
        ay <<= 1;
        az <<= 1;
        
        int signx = dx == 0 ? 0 : (dx >> 31 | 1); // signum with less converting to/from float
        int signy = dy == 0 ? 0 : (dy >> 31 | 1); // signum with less converting to/from float
        int signz = dz == 0 ? 0 : (dz >> 31 | 1); // signum with less converting to/from float

        int x = startx;
        int y = starty;
        int z = startz;

        int deltax, deltay, deltaz;
        if (ax >= Math.max(ay, az)) /* x dominant */ {
            deltay = ay - (ax >> 1);
            deltaz = az - (ax >> 1);
            while (true) {
                //result.offer(new Coord3D(x, y, z));
				dos.writeBytes(String.valueOf(x));
				dos.writeByte(' ');
				dos.writeBytes(String.valueOf(y));
				dos.writeByte(' ');
				dos.writeBytes(String.valueOf(z)); // Z
				dos.writeByte(' ');
				dos.writeBytes(String.valueOf(r));
				dos.writeByte(' ');
				dos.writeBytes(String.valueOf(g));
				dos.writeByte(' ');
				dos.writeBytes(String.valueOf(b));
				dos.writeByte('\r');
				dos.writeByte('\n');
                if (x == endx) {
                    return;
                }

                if (deltay >= 0) {
                    y += signy;
                    deltay -= ax;
                }

                if (deltaz >= 0) {
                    z += signz;
                    deltaz -= ax;
                }

                x += signx;
                deltay += ay;
                deltaz += az;
            }
        } else if (ay >= Math.max(ax, az)) /* y dominant */ {
            deltax = ax - (ay >> 1);
            deltaz = az - (ay >> 1);
            while (true) {
                //result.offer(new Coord3D(x, y, z));
				dos.writeBytes(String.valueOf(x));
				dos.writeByte(' ');
				dos.writeBytes(String.valueOf(y));
				dos.writeByte(' ');
				dos.writeBytes(String.valueOf(z)); // Z
				dos.writeByte(' ');
				dos.writeBytes(String.valueOf(r));
				dos.writeByte(' ');
				dos.writeBytes(String.valueOf(g));
				dos.writeByte(' ');
				dos.writeBytes(String.valueOf(b));
				dos.writeByte('\r');
				dos.writeByte('\n');
                if (y == endy) {
                    return;
                }

                if (deltax >= 0) {
                    x += signx;
                    deltax -= ay;
                }

                if (deltaz >= 0) {
                    z += signz;
                    deltaz -= ay;
                }

                y += signy;
                deltax += ax;
                deltaz += az;
            }
        } else {
            deltax = ax - (az >> 1);
            deltay = ay - (az >> 1);
            while (true) {
                //result.offer(new Coord3D(x, y, z));
				dos.writeBytes(String.valueOf(x));
				dos.writeByte(' ');
				dos.writeBytes(String.valueOf(y));
				dos.writeByte(' ');
				dos.writeBytes(String.valueOf(z)); // Z
				dos.writeByte(' ');
				dos.writeBytes(String.valueOf(r));
				dos.writeByte(' ');
				dos.writeBytes(String.valueOf(g));
				dos.writeByte(' ');
				dos.writeBytes(String.valueOf(b));
				dos.writeByte('\r');
				dos.writeByte('\n');
                if (z == endz) {
                    return;
                }

                if (deltax >= 0) {
                    x += signx;
                    deltax -= az;
                }

                if (deltay >= 0) {
                    y += signy;
                    deltay -= az;
                }

                z += signz;
                deltax += ax;
                deltay += ay;
            }
        }
    }


}
