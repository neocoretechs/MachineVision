package com.neocoretechs.machinevision;

import java.awt.AlphaComposite;
import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.RenderingHints;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicInteger;

import javax.imageio.ImageIO;
import javax.swing.JFrame;

import org.jtransforms.dct.FloatDCT_1D;
import org.jtransforms.dct.FloatDCT_2D;
import org.jtransforms.utils.IOUtils;

import com.neocoretechs.machinevision.hough3d.hough_settings;
import com.neocoretechs.machinevision.hough3d.octree_t;
import com.neocoretechs.robocore.SynchronizedThreadManager;

public class ImgProcessor {
	
	public static final int INBUF_SIZE = 65535;
	private static float[] coeffs = null;
	private static int camHeight = 480;
	private static int camWidth = 640;
    private static BufferedImage imageLx = null;
	static int[] dataL; // array return from canny with magnitudes
	static octree_t nodel = null;
	static octree_t noder = null;
	static String outDir = "/users/jg/workspace/robocore/motionclouds";
	private static boolean DEBUG = true;
	
	/**
	 * @param args
	 */
	public static void main(String[] args) {
		String fileName;
		//PlayerFrame displayPanel;
		//JFrame frame = null;
		if(args.length<1 || args.length>2) {
			System.out.println("Usage: java com.neocoretechs.machinevision.ImgProcessor [source file] <compare file>\n");
			return;
		} else {
			if(args.length == 1) {
				/*
				 * The following will write the result array to a file which is input file name +.csv
				 */
				fileName = args[0];
				if(DEBUG) {
			    	System.out.println("Processing image file:"+outDir+"/"+fileName+".jpg");
					System.out.println("Resulting processed template:"+outDir+"/"+fileName+".csv");
				}
				float[] a = processFile(fileName);
				FileOutputStream fos;
				try {
					fos = new FileOutputStream(outDir+"/"+fileName+".csv");
				} catch (FileNotFoundException e1) {
					e1.printStackTrace();
					return;
				}
				try {
				for(int i = 0; i < a.length; i++) { // significant coefficients for 256 theta by 512 (+1/2 height) high hough
					fos.write((String.valueOf(a[i])+"\r\n").getBytes());
				}
				} catch (IOException e) {
					e.printStackTrace();
				} finally {
					try {
						fos.flush();
						fos.close();
					} catch (IOException e) {}
				}
				if(DEBUG)
					System.out.println("Number of processed vectors="+a.length);
			} else {
			//frame = new JFrame("Player");
			//displayPanel = new PlayerFrame();

			//frame.getContentPane().add(displayPanel, BorderLayout.CENTER);

			// Finish setting up the frame, and show it.
			//frame.addWindowListener(new WindowAdapter() {
			//	public void windowClosing(WindowEvent e) {
			//		System.exit(0);
			//	}
			//});
			//displayPanel.setVisible(true);
			//frame.pack();
			//frame.setVisible(true);
			//frame.setSize(new Dimension(672, 836));
			
			fileName = args[0];
			if(DEBUG) {
				System.out.println("Preprocessed template:"+outDir+"/"+args[1]+".csv");
		    	System.out.println("Comparison image file:"+outDir+"/"+fileName+".jpg");
			}
			float[] a = processFile(fileName);
			// read in file 2 and do an RMS error compr
			File cmprf1 = new File(fileName);
			File cmprf2 = new File(args[1]);
			String rmsFile = cmprf1.getName()+cmprf2.getName();
			FileReader fis = null;
			try {
				fis = new FileReader(outDir+"/"+args[1]+".csv");
				String cos = null;
				BufferedReader dr = new BufferedReader(fis);
				float[] b = new float[a.length];
				int i = 0;
				for(; i < a.length; i++) {
					//System.out.println("Res index:"+i+"="+a[i]);
					cos = dr.readLine();
					if(cos == null) break;
					b[i]=Float.valueOf(cos);
				}
				fis.close();
				if(DEBUG)
					System.out.println("Number of processed vectors="+a.length+" RMS elements="+Math.min(a.length,i+1));
				System.out.println("RMSE:"+IOUtils.computeRMSE(a, b, Math.min(a.length,i+1)));//, rmsFile));
			} catch (FileNotFoundException e1) {
				System.out.println("Preprocessed template:"+outDir+"/"+args[1]+".csv could NOT be located, exiting..");
				System.exit(0);
			} catch (IOException e1) {
				e1.printStackTrace();
			}
		} // if
		}
		System.exit(0);
	}
	
	
	@SuppressWarnings("unused")
	public static float[] processFile(String filename) {
		float summ = 0;
	    FileInputStream fin = null;
	    File f = new File(outDir+"/"+filename+".jpg");
	    try {
		    fin = new FileInputStream(f);
		    imageLx = ImageIO.read(f);
		    //BufferedImage img = resizeImage(oimg, 512, 512);
			//displayPanel.lastFrame = img;
		    //BufferedImage bimage = new BufferedImage(img.getWidth(null), img.getHeight(null), BufferedImage.TYPE_INT_ARGB);
		    // Draw the image on to the buffered image
		    //Graphics2D bGr = bimage.createGraphics();
		    //bGr.drawImage(img, 0, 0, null);
		    //bGr.dispose();
		    //gf.setWidth(bimage.getWidth());
		    //gf.setHeight(bimage.getHeight());
		    //gf.filter(img, bimage);
    	    synchronized(imageLx) {
       	     CannyEdgeDetector ced = new CannyEdgeDetector();
       	     ced.setLowThreshold(0.5f);
       	     ced.setHighThreshold(1f);
       	     ced.setSourceImage(imageLx);
       	     dataL = ced.semiProcess();
       	    }
		    //displayPanel.lastFrame = ced.getEdgesImage();
			//displayPanel.invalidate();
			//displayPanel.updateUI();
			nodel = new octree_t();
			octree_t.buildStart(nodel);
			genOctree();
	     	octree_t.buildEnd(nodel);
     		// set our new maximal level
     		hough_settings.s_level = 5;
     		// set the distance to plane large to not throw out outliers 
     		// this is a divisor so we set to 1
     		hough_settings.max_distance2plane = 1;
     		hough_settings.min_isotropy = .01; // prevent elongated or vertical funky planes
	     	nodel.subdivide();
			if( DEBUG  ) {
				System.out.println("Loaded "+nodel.m_points.size()+" points...");
			}
			// computed centroid in load_point_cloud
			//Vector4d centroid = father.m_centroid.divide(father.m_points.size());
		   ArrayList<octree_t> nodes = nodel.get_nodes();
		   coeffs = new float[nodes.size()];
		   int i = 0;
		   for(octree_t node: nodes) {
			   double[] sph1 = octree_t.cartesian_to_spherical(node.getCentroid()); //1=phi, 2=rho
			   double degPhi = Math.toDegrees(sph1[1]);
			   if( degPhi < 0.0) {
				    degPhi += 360.0;
			   }
			   // axis of middle variance, perp to normal
			   double[] sph2 = octree_t.cartesian_to_spherical(node.getNormal2());
			   double degPhi2 = Math.toDegrees(sph2[1]);
			   if( degPhi2 < 0.0) {
				    degPhi2 += 360.0;
			   }
			   // our rho for 2 is eigenvalue
			   // form the vector from normal2 and its magnitude (eigenvalue) toward centroid to origin 0,0 center
			   long val = ((short)degPhi2)<<48 | ((short)node.getVariance2())<<32 | ((short)degPhi)<<16 | (short)sph1[2];
			   if(DEBUG)
				   System.out.println("val "+i+"="+val+" (float)val="+(float)val+" -- "+(short)degPhi2+" "+(short)node.getVariance2()+" "+(short)degPhi+" "+(short)sph1[2]);
			   coeffs[i++] = (float)val;
			   
		   }
		   FloatDCT_1D fdct1d = new FloatDCT_1D(i);
		   fdct1d.inverse(coeffs, false);
	    } catch(Exception e) {
	    	e.printStackTrace();
	    } finally {
	    	try { fin.close(); } catch(Exception ee) {}
	    } // try

	    //System.out.println("Stop. Sum="+summ);  
	    return coeffs;
	}

	public static void genOctree() {
		final AtomicInteger yStart = new AtomicInteger();
		long etime = System.currentTimeMillis();
		yStart.set(0);
		int numThreads = camHeight/10;
		int execLimit = camHeight;
		  //
		  // spin all threads necessary for execution
		  //
		Future<?>[] jobs = new Future<?>[execLimit];
		  for(int syStart = 0; syStart < execLimit; syStart++) {
			  SynchronizedThreadManager sftpm = SynchronizedThreadManager.getInstance();
			  jobs[syStart] = SynchronizedThreadManager.getInstance().submit(new Runnable() {
			  @Override
			  public void run() {
					imageToOctrees(dataL, imageLx, yStart.getAndIncrement(), camWidth, camHeight, nodel);
			  } // run
		    }); // spin
		  } // for syStart
		  SynchronizedThreadManager.waitForCompletion(jobs);
		  System.out.println("Process time one="+(System.currentTimeMillis()-etime));
	}
	
	/**
	 * Translate 2 edge detected image integer linear arrays of edge data and their corresponding RGB images
	 * into 2 octrees where the Z is set to 1. Intended to be 1 scan line in multithreaded parallel thread group.
	 * @param imageL image result of Canny edge detector
	 * @param imageLx2 RGB image source 
	 * @param yStart The Y scan line to process, this should be Atomic Integer incrementing per thread assignment
	 * @param width width of images
	 * @param height height of images
	 * @param nodel octree root node, filled by this method partially
	 */
	public static final void imageToOctrees(int[] imageL,  
											BufferedImage imageLx2, 
											int yStart, 
											int width, int height,
											octree_t nodel) {
		int[] imgsrcL = new int[width]; // image scan line 
   		int[] imgsrcLx = new int[width]; // image scan line 
 		synchronized(imageL) {
 			// gradient magnitude
 			System.arraycopy(Arrays.copyOfRange(imageL, yStart*width, (yStart+1)*width), 0, imgsrcL, 0, width);
 		}
		synchronized(imageLx2) {
			imageLx2.getRGB(0, yStart, width, 1, imgsrcLx, 0, width);
		}	
	  	for(int xsrc = 0; xsrc < width; xsrc++) {
	  		// If the left image pixel which is the target to receive the depth value is not edge, continue
			if(imgsrcL[xsrc] == 0)
					continue;
			double ks = xsrc - (width/2);
			double ms = yStart - (height/2);
			double os = 1;//(Bf/2) - imgsrcL[xsrc]; // gradient intensity

			synchronized(nodel) {
				octree_t.build(nodel, (double)ks, (double)ms, os, 
					((imgsrcLx[xsrc] & 0x00FF0000) >> 16 ), 
					((imgsrcLx[xsrc] & 0x0000FF00) >> 8), 
					((imgsrcLx[xsrc] & 0x000000FF) ));
			}
		} //++xsrc  move to next x subject in left image at this scanline y
	}
	  /**
     * This function resize the image file and returns the BufferedImage object that can be saved to file system.
     */
    public static BufferedImage resizeImage(final Image image, int width, int height) {
    	final BufferedImage bufferedImage = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
    	final Graphics2D graphics2D = bufferedImage.createGraphics();
    	graphics2D.setComposite(AlphaComposite.Src);
    	graphics2D.setRenderingHint(RenderingHints.KEY_INTERPOLATION,RenderingHints.VALUE_INTERPOLATION_BILINEAR);
    	graphics2D.setRenderingHint(RenderingHints.KEY_RENDERING,RenderingHints.VALUE_RENDER_QUALITY);
    	graphics2D.setRenderingHint(RenderingHints.KEY_ANTIALIASING,RenderingHints.VALUE_ANTIALIAS_ON);
    	graphics2D.drawImage(image, 0, 0, width, height, null);
    	graphics2D.dispose();
    	return bufferedImage;
    }
    /**
     * Generate a 2D gaussian norm distribution based on A
     * @param A Amplitude (maximum height) of center of matrix
     * @param dim The dimensions of the array, if not odd number, it will be coerced into one
     * @return 2D array of dim x dim gaussian distribution elements with A at center
     */
    public static int[][] gaussianNormal(double A, int dim) {
    	double x,y;
		double p;
		int cx = 0,cy = 0;
		if( (dim & 1) != 1 ) ++dim;
		int[][] res = new int[dim][dim];
		int corrLeft = (int) Math.floor(dim/2);
		int corrRight = corrLeft + 1;
		System.out.println("gaussian distro dims L,R:"+corrLeft+","+corrRight);
		// outer loop i is for x elements
		for(int i = -corrLeft; i < corrRight; i++) {
			cy = 0;
			x = ((double)i)/10;
			// inner loop j for y elements
			for(int j = -corrLeft; j < corrRight; j++) {
				y = ((double)j)/10;
				// 2d gaussian normal mu=0 sigma=1
				p = A * Math.exp(-( ((x*x)/2) + ((y*y)/2) ));
				res[cx][cy++] = (int)p;
				//System.out.print((int)p);
				//System.out.print(" ");
			}
			++cx;
			//System.out.println();
		}
		for(int i = 0; i < dim; i++) {
			for(int j = 0; j < dim; j++)
				System.out.print(res[i][j]+" ");
			System.out.println();
		}
		System.out.println();
		return res;
    }
    
    

}
