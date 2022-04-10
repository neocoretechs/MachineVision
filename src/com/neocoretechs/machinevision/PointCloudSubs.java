package com.neocoretechs.machinevision;

import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.math.BigInteger;
import java.nio.ByteBuffer;

import java.time.LocalDate;
import java.time.LocalTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.BrokenBarrierException;
import java.util.concurrent.CyclicBarrier;
import java.util.concurrent.atomic.AtomicInteger;

import javax.imageio.ImageIO;

import org.jtransforms.dct.DoubleDCT_1D;
import org.jtransforms.dct.FloatDCT_1D;
import org.jtransforms.utils.IOUtils;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import com.neocoretechs.machinevision.hough3d.Vector4d;
import com.neocoretechs.machinevision.hough3d.hough_settings;
import com.neocoretechs.machinevision.hough3d.octree_t;
import com.neocoretechs.machinevision.hough3d.writer_file;
import com.neocoretechs.robocore.SynchronizedFixedThreadPoolManager;

/**
 * Listen for pointcloud messages.
 * Perform the DCT on the
 * polar left node bit interchanged theta,phi keys and if the RMS error of the 2 DCT vectors is more than 1.3 sigma or so
 * declare that the image has changed and transmit the new point cloud.
 * 
 * @author jg
 *
 */
public class PointCloudSubs extends AbstractNodeMain {
	private static boolean DEBUG = false;
	private static  boolean DEBUGTEST2 = false;
	private static boolean WRITEFILES = false;
	private static boolean WRITEPLANARS = false;
	private static boolean SAMPLERATE = false;
	protected static boolean TIMER = false;
	private static boolean WRITERMS = true;
	private final static String outDir = "/users/jg/workspace/robocore/motionclouds";
	static int imageWidth, imageHeight;
	final static int leftBound = -207;
	final static int rightBound = 207;
	protected static int files = 0;
	final static int CHANNELS = 1;
	// optical parameters
    final static float f = 4.4f; // focal length mm
    final static float B = 100.0f; // baseline mm
    final static float FOVD = 60; // degrees field of view
    final static double FOV = 1.04719755; // radians FOV
    final static int camWidth = 640; // pixels
    final static int camHeight = 480;
    final static double DCTRMSSigma = 1.38; // number of standard deviations from DCT RMS mean before we decide its a new image
	double mean, sigma, variance;
	double meanRMS, sigmaRMS, varianceRMS;
    private BufferedImage imageLx = null;
	octree_t[] nodel = new octree_t[CHANNELS];
	float[] depth;// = new float[camWidth*camHeight];
	double[] coeffs;
	double[] prevDCT = null;
	//ArrayList<List<int[]>> leftYRange;// = new ArrayList<List<int[]>>(); // from/to position in array for sorted Y centroid processing

	//CyclicBarrier latch = new CyclicBarrier(camHeight/corrWinSize+1);
	//CyclicBarrier latchOut = new CyclicBarrier(camHeight/corrWinSize+1);
	CyclicBarrier latch = new CyclicBarrier(2);
	CyclicBarrier latchOut = new CyclicBarrier(2);
	CyclicBarrier latch2 = new CyclicBarrier(2);
	CyclicBarrier latchOut2 = new CyclicBarrier(2);
	CyclicBarrier latch3 = new CyclicBarrier(2);
	CyclicBarrier latchOut3 = new CyclicBarrier(2);
	CyclicBarrier latch4 = new CyclicBarrier(2);
	CyclicBarrier latchOut4 = new CyclicBarrier(2);
	//int yStart;
	int threads = 0;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("subs_pointcloud");
	}
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		//final Log log = connectedNode.getLog();
		Subscriber<stereo_msgs.DisparityImage2> subsbat = connectedNode.newSubscriber("/sensor_msgs/PointCloud", stereo_msgs.DisparityImage2._TYPE);
		setUp();
		spinUp(connectedNode);
		subsbat.addMessageListener(new MessageListener<stereo_msgs.DisparityImage2>() {
		@Override
		public void onNewMessage(stereo_msgs.DisparityImage2 message) {
				++files;
				depth = message.getD();
				int goodDisp = 0;
				for(int i = 0; i < depth.length; i++)
					if( depth[i] > 0 )
						++goodDisp;
				System.out.println(message.getHeader().getFrameId()+" @ "+message.getHeader().getSeq()+" "+message.getHeader().getStamp());
				System.out.println("Uncorrelated points="+(depth.length-goodDisp)+" correlated="+goodDisp+", "+(100.0-((float)(depth.length-goodDisp)/(float)depth.length)*100.0)+"% correlated @ "+LocalDate.now()+" "+LocalTime.now()); //2016/11/16 12:08:43);
				ByteBuffer cbL;
				byte[] bufferL, datax;
				
				// signal commencement processing by placing the last in queue
				try {
						cbL = message.getImage().getData();
						bufferL = cbL.array();
						InputStream in = new ByteArrayInputStream(bufferL);
						imageLx = ImageIO.read(in);
						in.close();
						if( imageLx.getType() != BufferedImage.TYPE_3BYTE_BGR)
							System.out.println("NOT 3 byte BGR, but:"+imageLx.getType());
						imageWidth = imageLx.getWidth();
						imageHeight = imageLx.getHeight();
						datax = (byte[]) imageLx.getData().getDataElements(0, 0, imageWidth, imageHeight, null);
						if(WRITEFILES)
							writeFile(datax, depth, "/robotEye"+files);
						//
	        			latchOut2.reset();
	        			latch2.reset();
	        			latchOut.reset();
	        			latch.reset();
	        			// the various channel code is to support multiple input channels
	        			// In practice here we are only using channel 0
		        	    for(int channels = 0; channels < CHANNELS; channels++) {
		        	    	nodel[channels] = new octree_t();
		        	    	octree_t.buildStart(nodel[channels]);
		        	    }
	        			// metrics
	        			//metric1 = metric2 = metric3 = metric4 = metric5 = 0;
	        	     	// first step end multi thread barrier synch
	        			latch.await();
	        	     	latchOut.await();    	
	        	     	for(int channels = 0; channels < CHANNELS; channels++) {
	        	     	  synchronized(nodel[channels]) {
	        	     		octree_t.buildEnd(nodel[channels]);
	        	     		nodel[channels].subdivide();
	        	     		if( WRITEFILES || WRITEPLANARS) {
	        	     			System.out.println("Writing planars"+channels+"L"+files);
	        	     			writer_file.writePerp(nodel[channels], "planars"+channels+"L"+files);
	        	     		}     		
	        	     	  }
	        	     	}
	        	     	// If we using multiple channels, we would have to aggregate the channels to one array
						coeffs = new double[nodel[0].get_nodes().size()];
	        	     	// signal the process to fill coeffs array with the chromospatial keys
	        	     	latch2.await();
	        	     	latchOut2.await();
	        			//FloatDCT_1D fdct1d = new FloatDCT_1D(coeffs.length);
	        			//fdct1d.forward(coeffs, false);
	        			DoubleDCT_1D fdct1d = new DoubleDCT_1D(coeffs.length);
	        			fdct1d.forward(coeffs, false);
						if(prevDCT != null) {
						  double rms = IOUtils.computeRMSE(coeffs, prevDCT, Math.min(coeffs.length, prevDCT.length)/10);
						  meanRMS += rms;
						  meanRMS /= 2.0;
						  varianceRMS += Math.pow((meanRMS - rms),2);
						  varianceRMS /= 2.0;
						  sigmaRMS = Math.sqrt(varianceRMS);
						  System.out.println("DCT RMS Mean="+meanRMS+" variance="+varianceRMS+" standard deviation="+sigmaRMS);
      	     		  	  System.out.println("RMS="+rms+" RMS err="+(meanRMS+(sigmaRMS*DCTRMSSigma))+" file index:"+files);
      				  	  // if we are more than n standard deviation from the overall mean, do something
  					  	  if( rms > (meanRMS+(sigmaRMS*DCTRMSSigma)) ) { //sigmaRMS*2 is 2 standard deviations
      	     				System.out.println("<<<< IMAGE SHIFT ALERT!!!! >>>> rms="+rms+" err="+(meanRMS+(sigmaRMS*DCTRMSSigma))+" file index:"+files);
       	     		  	  //int totRMS = 0;
      	     			  //for(int irms = 0; irms < Math.min(a.length, prevDCT.length); irms++ ) {
      	     			  //	if(a[irms] != prevDCT[irms])
      	     			  //		++totRMS;
      	     			  //		//System.out.println("RMS a("+irms+")="+a[irms]+" pre("+irms+")="+prevDCT[irms]);
      	     			  //}
      	     			  //System.out.println(totRMS+" DCT elements differ");
      	     				if( WRITERMS ) {
      	     					synchronized(nodel) {
      	     						System.out.println("Writing RMS"+(int)rms+"."+files);
      	     						writeFile(nodel,"/RMS"+(int)rms+"."+files);
      	     					}
      	     				}
  					  	  }
						}
					  	prevDCT = coeffs;
					  	genNav2(nodel[0].get_nodes());
				} catch (IOException e1) {
					System.out.println("Could not convert image payload due to:"+e1.getMessage());
					return;
				} catch (InterruptedException | BrokenBarrierException e) {
					//e.printStackTrace();
					//System.out.println("<<BROKEN BARRIER>> "+files);
				}
		}
		});	
	}
	/**
	 * Spin all parallel processing threads
	 * @param  
	 */
	public void spinUp(final ConnectedNode connectedNode) {
		/*
		 * Main worker thread for image data. 
		 * Wait at synch barrier for completion of all processing threads, then display disparity 
		 */
		final AtomicInteger[] yStart = new AtomicInteger[CHANNELS];
		for(int channels = 0; channels < CHANNELS; channels++) {
			yStart[channels] = new AtomicInteger(0);
		}
		SynchronizedFixedThreadPoolManager.init(16, camHeight, new String[]{"BUILDOCTREEA"});
		SynchronizedFixedThreadPoolManager.init(16, 64, new String[]{"PROCESSREGIONA"});
		final int execLimit = camHeight;
		SynchronizedFixedThreadPoolManager.spin(new Runnable() {
				@Override
				public void run() {
					while(true) {
					//
					// Spin the threads for each chunk of image
					// Since we run a continuous loop inside run we have to have a way
					// to synchronize everything at the beginning of a new image, then
					// await completion of all processing threads and signal a reset to
					// resume processing at the start of another new image, hence the two barrier
					// synchronization latches
					//
					try {
					  latch.await();
					  long etime = System.currentTimeMillis();
					  //
					  // spin all threads necessary for execution
					  //
					  for(int channels = 0; channels < CHANNELS; channels++) {
						  yStart[channels].set(0);
					  }
					  SynchronizedFixedThreadPoolManager.resetLatch(execLimit, "BUILDOCTREEA");
					  for(int syStart = 0; syStart < execLimit; syStart++) {
					    	SynchronizedFixedThreadPoolManager.spin(new Runnable() {
					    		@Override
					    		public void run() {
					    			// Since we run a continuous loop inside run we have to have a way
					    			// to synchronize everything at the beginning of a new image, then
					    			// await completion of all processing threads and signal a reset to
					    			// resume processing at the start of another new image, hence the two barrier
					    			// synchronization latches
					    			imageToOctree(/*dataL,*/ imageLx, 
										yStart[0].getAndIncrement(), camWidth, camHeight, 
										nodel[0], depth);
					    		} // run
					    	},"BUILDOCTREEA"); // spin
					  }
					  SynchronizedFixedThreadPoolManager.waitForGroupToFinish("BUILDOCTREEA");
					  if( TIMER )
						  System.out.println("Process time one="+(System.currentTimeMillis()-etime));
					  latchOut.await();
					  //
					  latch2.await();
					  etime = System.currentTimeMillis();

					  //leftYRange.clear();
					  //radixTree.clear();
					  final ArrayList<List<int[]>> leftYRange = new ArrayList<List<int[]>>(); // from/to position in array for sorted Y centroid processing
					  //
					  final List<ArrayList<octree_t>> tnodel = Collections.synchronizedList(new ArrayList<ArrayList<octree_t>>());
					  //Arrays.fill(depth, 0);
					  // build the work partition array
					  for(int channels = 0; channels < CHANNELS; channels++){
						  yStart[channels].set(0);
						  tnodel.add(nodel[channels].get_nodes());		
						  List<int[]> yr = Collections.synchronizedList(new ArrayList<int[]>());
						  leftYRange.add(yr);		
						  //System.out.println("Node list length channel "+channels+" octree="+tnodel.get(channels).size());
						  int iPosStart = 1;
						  int iPosEnd = 1;
						  int incr = tnodel.get(channels).size()/64;
						  for(int i = 0; i < incr; i++){
							  iPosEnd +=64;
							  yr.add(new int[]{iPosStart, iPosEnd-1});
							  iPosStart = iPosEnd;
						  }
						  iPosEnd = tnodel.get(channels).size();
						  yr.add(new int[]{iPosStart, iPosEnd});
					  }
					  // use channel 0
					  SynchronizedFixedThreadPoolManager.resetLatch(leftYRange.get(0).size(), "PROCESSREGIONA");
					  for(int syStart = 0; syStart < leftYRange.get(0).size(); syStart++) {
							SynchronizedFixedThreadPoolManager.spin(new Runnable() {
								@Override
								public void run() {
									// set the left nodes with depth
									processRegions(yStart[0].getAndIncrement(), leftYRange.get(0), 
											camWidth, camHeight, tnodel.get(0), coeffs);
								} // run
							},"PROCESSREGIONA"); // spin
					    } // for syStart
					  SynchronizedFixedThreadPoolManager.waitForGroupToFinish("PROCESSREGIONA");
					  if( TIMER )
							System.out.println("Process time two="+(System.currentTimeMillis()-etime));
					  latchOut2.await();
					// global barrier break
					} catch (BrokenBarrierException | InterruptedException e) {
						//System.out.println("<<BARRIER BREAK>> "+files);
					}
					} // while true
			} // run
		}); // spin
	}
	/**
	 * Perform the setup tasks to begin processing
	 */
	public void setUp() {
     	/**
 	     * Relative octree PCA tolerances associated with plane thickness (s_alpha) and plane isotropy (s_beta)<br/>
 		 * If thickness is less than max_thickness and isotropy is greater than min_isotropy
 		 * then remove outliers, compute least_variance_direction, and set coplanar to true.<br/>
 		 * thickness = variance1 / variance2, isotropy  = variance2 / variance3;<br/>
 		 * default is 5.
 	     */
 		hough_settings.max_thickness = 10;
 	    /**
 	     * s_level determines maximum octree level we check for variance direction and remove outliers.<br/>
 	     * Coplanar areas may be formed at higher level if measure for isoptropy passes, but s_level determines smallest cell.<br/>
 	     * default is level 7 which produces approximately 10x10 point cells.
 	     */
 		hough_settings.s_level = 6;
 		/**
 		 * max_distance2plane is the divisor for size that determines max plane distance for 
 		 * octree outlier removal (size_of_octree_cell/max_distance2plane).<br/>
 		 * Subtract the centroid from the passed point and take the scalar dot product of that
 		 * and the normalized 'normal1' vector. The absolute value of that is the distance to plane.<br/>
 		 * If this distance to plane is greater than size_of_octree_cell/max_distance2plane, we will
 		 * remove the point from the cell.<br/>
 		 * default is 5.
 		 */
 		hough_settings.max_distance2plane = 1;
 	    /**
 	     * Relative octree PCA tolerances associated with plane thickness (s_alpha) and plane isotropy (s_beta)<br/>
 		 * If thickness is less than max_thickness and isotropy is greater than min_isotropy
 		 * then remove outliers, compute least_variance_direction, and set coplanar to true.<br/>
 		 * thickness = variance1 / variance2, isotropy  = variance2 / variance3;<br/>
 		 * default is 0, which allows any manner of deformed degenerate plane.
 	     */
 		hough_settings.min_isotropy = .3;
 		/**
 		 * s_ms determines the minimum number of points per octree node.
 		 */
 		hough_settings.s_ms = 5;
	}
	/**
	 * Translate 2 PixelsToModel integer linear arrays of edge data and their corresponding RGB images
	 * into 2 octrees. Intended to be 1 scan line in multithreaded parallel thread group.
	 * @param imageL image result 
	 * @param imageLx2 RGB image source
	 * @param yStart The Y scan line to process, this should be Atomic Integer incrementing per thread assignment
	 * @param width width of images
	 * @param height height of images
	 * @param nodel octree root node, filled by this method partially
	 */
	public final void imageToOctree(/*int[] imageL,*/ BufferedImage imageLx2,
									int yStart, int width, int height,
									octree_t nodel, float[] depth) {
		long etime = System.currentTimeMillis();
   		int[] imgsrcLx = new int[width]; // image scan line 
 		//synchronized(imageL) {
 		//	System.arraycopy(Arrays.copyOfRange(imageL, yStart*width, (yStart+1)*width), 0, imgsrcL, 0, width);
 		//}
		synchronized(imageLx2) {
			imageLx2.getRGB(0, yStart, width, 1, imgsrcLx, 0, width);
		}

	  	for(int xsrc = 0; xsrc < width; xsrc++) {
	  		// If the left image pixel which is the target to receive the depth value is not edge, continue
			//if(imgsrcL[xsrc] == 0)
			//		continue;
			double ks = xsrc - (width/2);
			double ms = yStart - (height/2);
			double os = depth[(yStart*width)+xsrc]; 

			synchronized(nodel) {
				octree_t.build(nodel, (double)ks, (double)ms, os, 
					((imgsrcLx[xsrc] & 0x00FF0000) >> 16 ), 
					((imgsrcLx[xsrc] & 0x0000FF00) >> 8), 
					((imgsrcLx[xsrc] & 0x000000FF) ));
			}
		} //++xsrc  move to next x subject in left image at this scanline y
			//System.out.println("Y scan="+y);
		//} // next y
		if( SAMPLERATE  )
			System.out.println("imagesToOctrees IMAGE SCAN LINE Y="+yStart+" ***** "+Thread.currentThread().getName()+" *** "+(System.currentTimeMillis()-etime)+" ms.***");
	}
	
	private void processRegions(
			int yStart, List<int[]> range,
			int camwidth,
			int camheight,
			ArrayList<octree_t> lNodes, double[] coeffs) {
		long etime = System.currentTimeMillis();
		int[] irange = range.get(yStart);
		if(DEBUGTEST2)
			System.out.println("Region from "+irange[0]+","+irange[1]+" ***** "+Thread.currentThread().getName());
		for(int i = irange[0]; i < irange[1]; i++) {
			if(i >= lNodes.size()) {
				return;
			}
			octree_t inode = lNodes.get(i);
			coeffs[i] = genChromoSpatialKeys2(inode);
		}
		if( SAMPLERATE  )
			System.out.println("processRegion processing="+irange[0]+" to "+irange[1]+" ***** "+Thread.currentThread().getName()+" *** "+(System.currentTimeMillis()-etime)+" ms.***");
	}
	/**
	 * Write the pixels from source image at depth * 2
	 * @param d List of envInterface post-processed nodes data
	 * @param filename file to write to
	 * @param compVal if >= 0 the value to compare to depth to write conditionally
	 */
	protected void writeFile(byte[] data, float[] depth, String filename) {
		DataOutputStream dos = null;
		File f = new File(outDir+filename+".asc");
		try {
			dos = new DataOutputStream(new FileOutputStream(f));
			for(int i = 0; i < imageWidth; i++) {
				for(int j = 0; j < imageHeight; j++) {
					dos.writeBytes(String.valueOf(i)); // X
						dos.writeByte(' ');
						dos.writeBytes(String.valueOf(j));// Y
						dos.writeByte(' ');
						dos.writeBytes(String.valueOf(depth[(j*imageWidth)+i]*2)); // Z
						dos.writeByte(' ');
						dos.writeBytes(String.valueOf(data[(((j*imageWidth)+i)*3)+2])); // R
						dos.writeByte(' ');
						dos.writeBytes(String.valueOf(data[(((j*imageWidth)+i)*3)+1])); // G
						dos.writeByte(' ');
						dos.writeBytes(String.valueOf(data[((j*imageWidth)+i)*3])); // B
						dos.writeByte('\r');
						dos.writeByte('\n');					
				//writer_file.line3D(dos, (int)id.getEnv()[0]/*xmin*/, (int)id.getEnv()[1]/*ymin*/, (int)(id.getDepth()*10.0d), 
				//		(int)id.getEnv()[2]/*xmax*/, (int)id.getEnv()[1]/*ymin*/, (int)(id.getDepth()*10.0d), 0, 255, 255);
				//writer_file.line3D(dos, (int)id.getEnv()[2]/*xmax*/, (int)id.getEnv()[1]/*ymin*/, (int)(id.getDepth()*10.0d), 
				//		(int)id.getEnv()[2]/*xmax*/, (int)id.getEnv()[3]/*ymax*/, (int)(id.getDepth()*10.0d), 0, 255, 255);
				//writer_file.line3D(dos, (int)id.getEnv()[2]/*xmax*/, (int)id.getEnv()[3]/*ymax*/, (int)(id.getDepth()*10.0d), 
				//		(int)id.getEnv()[0]/*xmin*/, (int)id.getEnv()[3]/*ymax*/, (int)(id.getDepth()*10.0d), 0, 255, 255);
				//writer_file.line3D(dos, (int)id.getEnv()[0]/*xmin*/, (int)id.getEnv()[3]/*ymax*/, (int)(id.getDepth()*10.0d), 
				//		(int)id.getEnv()[0]/*xmin*/, (int)id.getEnv()[1]/*ymin*/, (int)(id.getDepth()*10.0d), 0, 255, 255);
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
	}
	
	public static void writePerp(List<octree_t> nodes, int r, int g, int b, String fileName) {
		DataOutputStream dos = null;
		File f = new File(hough_settings.file+fileName+hough_settings.extension);
	  try {
		dos = new DataOutputStream(new FileOutputStream(f));
		Vector4d cen1 = new Vector4d();
		Vector4d cen2 = new Vector4d();
		Vector4d cen3 = new Vector4d();
		Vector4d cen4 = new Vector4d();
		for(octree_t cnode : nodes )  {
			// spherical of centroid
			writer_file.genPerp(cnode, cen1, cen2, cen3, cen4);
			writer_file.line3D(dos, (int)cnode.getCentroid().x, (int)cnode.getCentroid().y, (int)cnode.getCentroid().z, (int)cen1.x, (int)cen1.y, (int)cen1.z, 255, 255, 255);
			// axis of middle variance, perp to normal
			writer_file.line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, 255, 0, 0);
			// long axis
			// generate point at end of long segment
			writer_file.line3D(dos, (int)cen3.x, (int)cen3.y, (int)cen3.z, (int)cen4.x, (int)cen4.y, (int)cen4.z, 0, 0, 255);
			// outside
			writer_file.line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen4.x, (int)cen4.y, (int)cen4.z, r, g, b);
			writer_file.line3D(dos, (int)cen4.x, (int)cen4.y, (int)cen4.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, r, g, b);
			writer_file.line3D(dos, (int)cen2.x, (int)cen2.y, (int)cen2.z, (int)cen3.x, (int)cen3.y, (int)cen3.z, r, g, b);
			writer_file.line3D(dos, (int)cen3.x, (int)cen3.y, (int)cen3.z, (int)cen1.x, (int)cen1.y, (int)cen1.z, r, g, b);
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
	 * Generate the occupancy grid with minimum depth indicators into file seq
	 * from list of IndexMax elements
	 * @param nodes
	 */
	private static void genNav2(List<octree_t> nodes) {
		// descending sort mirrors forward direction of robot movement
		   //final Comparator<envInterface> yComp = new Comparator<envInterface>() {         
			//	  @Override         
			//	  public int compare(envInterface jc1, envInterface jc2) {             
			//		      return ((int)((IndexMax)jc2).node.getMiddle().y < (int)((IndexMax)jc1).node.getMiddle().y ? -1 :                     
			//		              ((int)((IndexMax)jc2).node.getMiddle().y == (int)((IndexMax)jc1).node.getMiddle().y ? 0 : 1));           
			//	  }     
		   //};
		   final Comparator<double[]> yComp = new Comparator<double[]>() {         
				  @Override         
				  public int compare(double[] jc1, double[] jc2) {             
					      return ((int)jc2[3] < (int)jc1[3] ? -1 :                     
					              ((int)jc2[3] == (int)jc1[3] ? 0 : 1));           
				  }     
		   };
		   final Comparator<octree_t> xComp = new Comparator<octree_t>() {         
				  @Override         
				  public int compare(octree_t jc1, octree_t jc2) {             
					      return ((int)jc2.getMiddle().x < (int)jc1.getMiddle().x ? -1 :                     
					              ((int)jc2.getMiddle().x == (int)jc1.getMiddle().x ? 0 : 1));           
				  }     
		   };
		   if(nodes.size() == 0)
			   return;
		   DataOutputStream dos = null;
		   File f = new File(hough_settings.file+"NavLine"+files+hough_settings.extension);
		   try {
			   dos = new DataOutputStream(new FileOutputStream(f));
			   // col and min depth marker
			   ArrayList<double[]> splinePts = new ArrayList<double[]>();
			   Collections.sort(nodes, xComp);
			   int x = (int)nodes.get(0).getMiddle().x;
			   int iPosStart = 0;
			   int iPosEnd = 0;
			   for(int i = 0 ; i < nodes.size(); i++) {
				   if( x != (int)nodes.get(i).getMiddle().x) {
					   iPosEnd = i;
					   genColHist2(dos, iPosStart, iPosEnd, nodes, splinePts);
					   iPosStart = i;
					   x = (int)nodes.get(i).getMiddle().x;
				   }
			   }
			   iPosEnd = nodes.size();
			   genColHist2(dos, iPosStart, iPosEnd, nodes, splinePts);
			   //if(splinePts.size() == 0)
				//   return;
			   // rows
			   //Collections.sort(nodes, yComp);
			   //int y = (int)((AbstractDepth)nodes.get(0)).node.getMiddle().y;
			   //iPosStart = 0;
			   //iPosEnd = 0;
			   //for(int i = 0 ; i < nodes.size(); i++) {
				//   if( y != (int)((AbstractDepth)nodes.get(i)).node.getMiddle().y) {
				//	   iPosEnd = i;
				//	   genRow2(dos, iPosStart, iPosEnd, nodes);
				//	   iPosStart = i;
				//	   y = (int)((AbstractDepth)nodes.get(i)).node.getMiddle().y;
				//   }
			   //}
			   //iPosEnd = nodes.size();
			   /*
			   Collections.sort(splinePts, yComp);
			   int y = (int)splinePts.get(0)[3]; //y
			   iPosStart = 0;
			   iPosEnd = 0;
			   for(int i = 0 ; i < splinePts.size(); i++) {
				   if( y != (int)splinePts.get(i)[3]) {
					   iPosEnd = i;
					   genRow2(dos, iPosStart, iPosEnd, splinePts);
					   iPosStart = i;
					   y = (int)splinePts.get(i)[3];
				   }
			   }
			   iPosEnd = splinePts.size();
			   genRow2(dos, iPosStart, iPosEnd, splinePts);
			   */
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

	public static void genRow2(DataOutputStream dos, int yStart, int yEnd, List<double[]> splinePts) throws IOException{
		//if(DEBUG)
			//System.out.println("genRow from="+yStart+" to="+yEnd);
			List<double[]> subNodes = splinePts.subList(yStart, yEnd);
			//final Comparator<envInterface> xComp = new Comparator<envInterface>() {         
			//		  @Override         
			//		  public int compare(envInterface jc1, envInterface jc2) {             
			//			      return ((int)((AbstractDepth)jc2).node.getMiddle().x < (int)((AbstractDepth)jc1).node.getMiddle().x ? -1 :                     
			//			              ((int)((AbstractDepth)jc2).node.getMiddle().x == (int)((AbstractDepth)jc1).node.getMiddle().x ? 0 : 1));           
			//		  }     
			//};
			final Comparator<double[]> xComp = new Comparator<double[]>() {         
					  @Override         
					  public int compare(double[] jc1, double[] jc2) {             
						      return ((int)jc2[0] < (int)jc1[0] ? -1 :                     
						              ((int)jc2[0] == (int)jc1[0] ? 0 : 1));           
					  }     
			};
			Collections.sort(subNodes, xComp);
			// sorted now by x and y
			double zMin = Double.MAX_VALUE;
			double[] cnode = null;
			for(int i = 0; i < subNodes.size(); i++) {
				//System.out.println("Row "+(int)subNodes.get(i)[0]+" "+(int)subNodes.get(i)[1]+" "+(int)subNodes.get(i)[2]);
				if( i < subNodes.size()-1) {
					//writer_file.line3D(dos, (int)((AbstractDepth)subNodes.get(i)).node.getCentroid().x, (int)((AbstractDepth)subNodes.get(i)).node.getCentroid().y, (int)((AbstractDepth)subNodes.get(i)).depth*10,
					//		(int)((AbstractDepth)subNodes.get(i+1)).node.getCentroid().x, (int)((AbstractDepth)subNodes.get(i+1)).node.getCentroid().y, (int)((AbstractDepth)subNodes.get(i+1)).depth*10,
					//		0, 255, 255);
					writer_file.line3D(dos, (int)subNodes.get(i)[0] , (int)subNodes.get(i)[1], (int)subNodes.get(i)[2], 
							(int)subNodes.get(i+1)[0], (int)subNodes.get(i+1)[1], (int)subNodes.get(i+1)[2], 0, 255, 255);
				}
				if((int)subNodes.get(i)[1] >= 0 && (int)subNodes.get(i)[0] >= leftBound && (int)subNodes.get(i)[0] <= rightBound &&
						subNodes.get(i)[2] <= zMin) {
					zMin = subNodes.get(i)[2];
					cnode = subNodes.get(i);
				}
			}
			//System.out.println("-----------");
			//  min depth
			if( cnode != null ) {
			Vector4d cen1 = new Vector4d();
			Vector4d cen2 = new Vector4d();
			cen1.x = cnode[0] - (cnode[4]/2);
			cen1.y = cnode[1] - (cnode[4]/2);
			cen1.z = zMin;
			cen2.x = cnode[0] + (cnode[4]/2);
			cen2.y = cnode[1] + (cnode[4]/2);
			cen2.z = zMin;
			//if( DEBUG )
			//	System.out.println("genColHist env minx,y,z="+cen1+" maxx,y,z="+cen2+" centroid node="+cnode);
			// xmin, ymin, xmax, ymin
			writer_file.line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen1.y, (int)cen1.z, 255, 127, 0);
			// xmax, ymin, xmax, ymax
			writer_file.line3D(dos, (int)cen2.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, 255, 127, 0);
			// xmax, ymax, xmin, ymax
			writer_file.line3D(dos, (int)cen2.x, (int)cen2.y, (int)cen2.z, (int)cen1.x, (int)cen2.y, (int)cen2.z, 255, 127, 0);
			// xmin, ymax, xmin, ymin
			writer_file.line3D(dos, (int)cen1.x, (int)cen2.y, (int)cen2.z, (int)cen1.x, (int)cen1.y, (int)cen1.z, 255, 127, 0);
			}
	}
	public static void genColHist2(DataOutputStream dos, int xStart, int xEnd, List<octree_t> nodelA, List<double[]> splinePts) throws IOException{
		//if(DEBUG)
			//System.out.println("genColHist from="+xStart+" to="+xEnd);
			List<octree_t> subNodes = nodelA.subList(xStart, xEnd);
			final Comparator<octree_t> yComp = new Comparator<octree_t>() {         
					  @Override         
					  public int compare(octree_t jc1, octree_t jc2) {             
						      return ((int)jc2.getMiddle().y < (int)jc1.getMiddle().y ? -1 :                     
						              ((int)jc2.getMiddle().y == (int)jc1.getMiddle().y ? 0 : 1));           
					  }     
			}; 
			Collections.sort(subNodes, yComp);
			/*
			ArrayList<double[]> dpit = new ArrayList<double[]>();
			if(SMOOTHEGRID) {
				PolyBezierPathUtil pbpu = new  PolyBezierPathUtil();
				ArrayList<PolyBezierPathUtil.EPointF> epfa = new ArrayList<PolyBezierPathUtil.EPointF>();
				for(int i = 0; i < subNodes.size(); i++) {
					PolyBezierPathUtil.EPointF epf = pbpu.new EPointF(((AbstractDepth)subNodes.get(i)).depth*10.0d, ((AbstractDepth)subNodes.get(i)).node.getMiddle().y);
					epfa.add(epf);
				}
				if(epfa.size() < 2)
					return;
				Path2D.Double path = pbpu.computePathThroughKnots(epfa);
				PathIterator pit = path.getPathIterator(null);
				while(!pit.isDone()) {
					double[] coords = new double[6];
					pit.currentSegment(coords);
					dpit.add(coords);
					pit.next();
				}
			} else {
				//---if we dont want to smooth
				for(int i = 0; i < subNodes.size(); i++) {
					double[] coords = new double[6];
					coords[0] = ((AbstractDepth)subNodes.get(i)).depth*10.0d;
					coords[1] = ((AbstractDepth)subNodes.get(i)).node.getMiddle().y;
					dpit.add(coords);
				}
			}
			*/
			double zMin = Double.MAX_VALUE;
			octree_t cnode = null;
			int cpos = -1;
			// sorted now by x and y
			for(int i = 0; i < subNodes.size(); i++) {
				//double[] splinep = dpit.get(i);
				//splinep[2] = splinep[0]; // rotate X back to Z
				//splinep[0] = ((AbstractDepth)subNodes.get(i)).node.getMiddle().x;// the x, then y is index 1 as originally
				//splinep[3] = ((AbstractDepth)subNodes.get(i)).node.getMiddle().y;// we need to sort the rows on this value
				//splinep[4] = ((AbstractDepth)subNodes.get(i)).node.getSize(); // to generate box
				//splinePts.add(splinep); // collect it in our splined array to make rows later
				//if( i < subNodes.size()-1) {
					//writer_file.line3D(dos, (int)((AbstractDepth)subNodes.get(i)).node.getCentroid().x, (int)((AbstractDepth)subNodes.get(i)).node.getCentroid().y, (int)((AbstractDepth)subNodes.get(i)).depth*10,
					//		(int)((AbstractDepth)subNodes.get(i+1)).node.getCentroid().x, (int)((AbstractDepth)subNodes.get(i+1)).node.getCentroid().y, (int)((AbstractDepth)subNodes.get(i+1)).depth*10,
					//		0, 255, 255);
					//writer_file.line3D(dos, (int)splinep[0] , (int)splinep[1], (int)splinep[2], 
					//		(int)((AbstractDepth)subNodes.get(i+1)).node.getMiddle().x, (int)dpit.get(i+1)[1], (int)dpit.get(i+1)[0], 0, 255, 255);
					//if( DEBUG ) 
					//	System.out.println("genColHist line3D x="+(int)((AbstractDepth)subNodes.get(i)).node.getCentroid().x+" y="+(int)((AbstractDepth)subNodes.get(i)).node.getCentroid().y+" z="+(int)((AbstractDepth)subNodes.get(i)).depth*10+
					//			" to x="+(int)((AbstractDepth)subNodes.get(i+1)).node.getCentroid().x+" y="+(int)((AbstractDepth)subNodes.get(i+1)).node.getCentroid().y+" z="+(int)((AbstractDepth)subNodes.get(i+1)).depth*10);
					// y remains the same and we rotate our depth 90 degrees to become the x axis
					// so we get an angular measurement thats perpendicular to frontal x,y plane
					// this corresponds to the theta angle we get from cartestian_to_spherical, for some reason in this
					// orientation, vs z axis vertical, the theta value is aligned along our depth, or z axis horizontal. 
					// So if we use spherical the phi and theta are swapped in this orientation.
					//double thet1 = Math.atan2( (((AbstractDepth)subNodes.get(i)).node.getCentroid().y-((AbstractDepth)subNodes.get(i+1)).node.getCentroid().y),
					//			((((AbstractDepth)subNodes.get(i)).depth-((AbstractDepth)subNodes.get(i+1)).depth)*10) );
					//double thet1 = Math.atan2( splinep[1]-dpit.get(i+1)[1], ((splinep[2]-dpit.get(i+1)[0])*10) );
					//double degThet = Math.toDegrees(thet1);
					//if( degThet < 0.0) {
					//	    degThet += 360.0;
					//}
					//Vector4d point = new Vector4d( (((AbstractDepth)subNodes.get(i)).node.getCentroid().x-((AbstractDepth)subNodes.get(i+1)).node.getCentroid().x),
					//		(((AbstractDepth)subNodes.get(i)).node.getCentroid().y-((AbstractDepth)subNodes.get(i+1)).node.getCentroid().y),
					//			((((AbstractDepth)subNodes.get(i)).depth-((AbstractDepth)subNodes.get(i+1)).depth)*10), 1);
					//double[] deg1 = octree_t.cartesian_to_spherical(point);
					//
					//System.out.println("Column "+(int)((AbstractDepth)subNodes.get(i)).node.getMiddle().x+" depth diff "+i+" degrees ="+degThet+" node="+((AbstractDepth)subNodes.get(i)).node);
				//}
				// since we sort descending to compute in the direction of robot motion, we want the highest numbered
				// row as minimum so we know the top of an obstacle right in front of us. To this end we use <= zMin
				// so identical depths will percolate to the top
				if((int)subNodes.get(i).getMiddle().y >= 0 && 
					//(int)splinep[0] >= leftBound && (int)splinep[0] <= rightBound &&
					(int)subNodes.get(i).getMiddle().x >= leftBound &&
					(int)subNodes.get(i).getMiddle().x <= rightBound &&
					subNodes.get(i).getMiddle().z > 0 && subNodes.get(i).getMiddle().z <= zMin) {
						zMin = subNodes.get(i).getMiddle().z;
						//cnode = ((AbstractDepth)subNodes.get(i)).node;
						cpos = i;
				}
			}
			//  min depth
			//if( cnode != null ) {
			if( cpos != -1 ) { // there may have been 0 elements
				cnode = subNodes.get(cpos);
				System.out.println("Zmin="+zMin+" cpos="+cpos+" node="+cnode); //delinate column depth display
				int isize = (int) cnode.getSize();
				Vector4d cen1 = new Vector4d();
				Vector4d cen2 = new Vector4d();
				cen1.x = cnode.getMiddle().x - (isize/2);
				cen1.y = cnode.getMiddle().y - (isize/2);
				cen1.z = 0;//zMin*10.0d;//cnode.getCentroid().z - (cnode.getSize()/2);
				cen2.x = cnode.getMiddle().x + (isize/2);
				cen2.y = cnode.getMiddle().y + (isize/2);
				cen2.z = 0;//zMin*10.0d;//cnode.getCentroid().z + (cnode.getSize()/2);
				while(cen1.y < (imageHeight/2)) {
					//if( DEBUG )
					//	System.out.println("genColHist env minx,y,z="+cen1+" maxx,y,z="+cen2+" centroid node="+cnode);
					// xmin, ymin, xmax, ymin
					writer_file.line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen1.y, (int)cen1.z, 0, 127, 255);
					// xmax, ymin, xmax, ymax
					writer_file.line3D(dos, (int)cen2.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, 0, 127, 255);
					// xmax, ymax, xmin, ymax
					writer_file.line3D(dos, (int)cen2.x, (int)cen2.y, (int)cen2.z, (int)cen1.x, (int)cen2.y, (int)cen2.z, 0, 127, 255);
					// xmin, ymax, xmin, ymin
					writer_file.line3D(dos, (int)cen1.x, (int)cen2.y, (int)cen2.z, (int)cen1.x, (int)cen1.y, (int)cen1.z, 0, 127, 255);
					// xmin, ymin, xmax, ymax
					writer_file.line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, 0, 127, 255);
					// xmax, ymin, xmin, ymax
					writer_file.line3D(dos, (int)cen2.x, (int)cen1.y, (int)cen1.z, (int)cen1.x, (int)cen2.y, (int)cen2.z, 0, 127, 255);
					cen1.y += isize;
					cen2.y += isize;
				}
			} else
				System.out.println("no elements");
			System.out.println("-----");
	}
	

	/**
	 * Generate the chromospatial keys for the node at degreesx10 and magnitudex10
	 * @param node
	 * @return 7 shorts of node value normal1: theta1, phi1, theta2, phi2, variance1, variance2, variance3
	 */
	private static double genChromoSpatialKeys2(octree_t node) {
		   double[] sph1 = octree_t.cartesian_to_spherical(node.getNormal1()); //1=theta,2=phi,3=rho
		   double degTheta = Math.toDegrees(sph1[0]);
		   if( degTheta < 0.0) {
			    degTheta += 360.0;
		   }
		   double degPhi = Math.toDegrees(sph1[1]);
		   if( degPhi < 0.0) {
			    degPhi += 360.0;
		   }
		   // deg*100 = 0-36099 = 1000110100000011b
		   short val1 = (short)(((int)(degTheta*10.0d)) & 0x7FFF);
		   short val2 = (short)(((int)(degPhi*10.0d)) & 0x7FFF);
		   //
		   double[] sph2 = octree_t.cartesian_to_spherical(node.getNormal2()); //1=theta,2=phi,3=rho
		   degTheta = Math.toDegrees(sph2[0]);
		   if( degTheta < 0.0) {
			    degTheta += 360.0;
		   }
		   degPhi = Math.toDegrees(sph2[1]);
		   if( degPhi < 0.0) {
			    degPhi += 360.0;
		   }
		   // deg*100 = 0-36099 = 1000110100000011b
		   short val1b = (short)(((int)(degTheta*10.0d)) & 0x7FFF);
		   short val2b = (short)(((int)(degPhi*10.0d)) & 0x7FFF);
		   //
		   double[] sph3 = octree_t.cartesian_to_spherical(node.getNormal3()); //1=theta,2=phi,3=rho
		   degTheta = Math.toDegrees(sph3[0]);
		   if( degTheta < 0.0) {
			    degTheta += 360.0;
		   }
		   degPhi = Math.toDegrees(sph3[1]);
		   if( degPhi < 0.0) {
			    degPhi += 360.0;
		   }
		   // deg*100 = 0-36099 = 1000110100000011b
		   short val1c = (short)(((int)(degTheta*10.0d)) & 0x7FFF);
		   short val2c = (short)(((int)(degPhi*10.0d)) & 0x7FFF);
		   if( val1 < 0 || val2 < 0 || val1b < 0 || val2b < 0 || val1c < 0 || val2c < 0)
			   System.out.println("OVERFLOW CHROMOSPATIAL KEY VALUE!! "+val1+" "+val2+" "+val1b+" "+val2b+" "+val1c+" "+val2c);
		   //short val3 = (short)(((int)(node.getVariance1()*10.0d)) & 0x3FF);
		   //short val4 = (short)(((int)(node.getVariance2()*10.0d)) & 0x3FF);
		   //short val5 = (short)(((int)(node.getVariance3()*10.0d)) & 0x3FF);
		   // 0x3FF 10 bits 1023 or 102.3 max variance
		   //long val = (((long)(degTheta*100.0d) & 0xFFFF) << 48) | (((long)(degPhi*100.0d) & 0xFFFF) << 32) | 
		   //	   (((long)(node.getVariance1()*10.0d) & 0x3FF) << 20) | (((long)(node.getVariance2()*10.0d) & 0x3FF) << 10) | (((long)(node.getVariance3()*10.0d) & 0x3FF)); 
		   //return new short[]{val1, val2, val1b, val2b, val1c, val2c, val3, val4, val5};
           BigInteger kxy = BigInteger.ZERO; 
           for(int i = 15; i >= 0; i--) {
           	kxy = kxy.shiftLeft(1);
           	kxy = kxy.add(BigInteger.valueOf((val1 >> i) & 1));
           	kxy = kxy.shiftLeft(1);
           	kxy = kxy.add(BigInteger.valueOf((val2 >> i) & 1));
           	kxy = kxy.shiftLeft(1);
            kxy = kxy.add(BigInteger.valueOf((val1b >> i) & 1));
            kxy = kxy.shiftLeft(1);
           	kxy = kxy.add(BigInteger.valueOf((val2b >> i) & 1));
           	kxy = kxy.shiftLeft(1);
            kxy = kxy.add(BigInteger.valueOf((val1c >> i) & 1));
            kxy = kxy.shiftLeft(1);
           	kxy = kxy.add(BigInteger.valueOf((val2c >> i) & 1));
           }
           return kxy.doubleValue();
	}
	protected void writeFile(octree_t[] nodel2, String filename) {
		DataOutputStream dos = null;
		File f = new File(outDir+filename+".asc");
		try {
			dos = new DataOutputStream(new FileOutputStream(f));
			for(int i = 0; i < nodel2[0].m_points.size(); i++) {
				Vector4d pnode = nodel2[0].m_points.get(i);
				Vector4d pcolor = nodel2[0].m_colors.get(i);
				dos.writeBytes(String.valueOf(pnode.x)); // X
					dos.writeByte(' ');
					dos.writeBytes(String.valueOf(pnode.y));// Y
					dos.writeByte(' ');
					dos.writeBytes(String.valueOf(pnode.z)); // Z
					dos.writeByte(' ');
					dos.writeBytes(String.valueOf((int)pcolor.x)); // R
					dos.writeByte(' ');
					dos.writeBytes(String.valueOf((int)pcolor.y)); // G
					dos.writeByte(' ');
					dos.writeBytes(String.valueOf((int)pcolor.z)); // B
					dos.writeByte('\r');
					dos.writeByte('\n');
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
}
