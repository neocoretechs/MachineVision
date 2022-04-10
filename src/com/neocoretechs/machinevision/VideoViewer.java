package com.neocoretechs.machinevision;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.BrokenBarrierException;
import java.util.concurrent.CyclicBarrier;
import java.util.concurrent.atomic.AtomicInteger;

import javax.imageio.ImageIO;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.SwingConstants;
import javax.swing.SwingUtilities;
import javax.swing.WindowConstants;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import org.ros.concurrent.CircularBlockingDeque;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import com.neocoretechs.machinevision.CannyEdgeDetector;
import com.neocoretechs.machinevision.hough2d.HoughElem;
import com.neocoretechs.machinevision.hough2d.HoughTransform;
import com.neocoretechs.machinevision.hough3d.Vector4d;
import com.neocoretechs.machinevision.hough3d.hough_settings;
import com.neocoretechs.machinevision.hough3d.octree_t;
import com.neocoretechs.machinevision.hough3d.writer_file;
import com.neocoretechs.robocore.SynchronizedFixedThreadPoolManager;

/**
 * Create a disparity map for the left and right images taken from stereo cameras published to bus.
 * We break the image into  bands and assign each band to a processing thread. 
 * We have an option to write files that can be read by CloudCompare for testing.
 *  __mode:= option on commandline determines how we process, such as 'edge','display', 'display3D',etc.
 *  
 * The new algorithm for stereo coplanar area matching principal component analysis (SCAMPCA). This fast algorithm for
 * assigning depth to stereo point clouds is independent of the presence of color images, works well on low
 * resolution images, requires no calibration of cameras, and is immune to misalignment of the cameras. 
 * 
 * New algorithm for stereo matching:
 * (The terms region, node, and octree node and octree cell are synonymous)
 * The steps are:
 * 1.) Edge detect both images using Canny, gradient level set high for max detail
 *
 * 2.) Generate 2 octrees of edge detected images at the minimal node level, now 7. this step in parallel by assigning
 * the octree build to one Y image scan line per thread.
 *
 * 3.) Use PCA on the 2 images to find minimal coplanar regions in both octrees and generate eigenvectors 
 * and eigenvalues of the principal axis of the minimal coplanar regions. this step in a single thread after barrier 
 * synchronization of the build step. Multiple threads not necessary as this step is fast.
 *
 * 4.) Process the left image coplanar regions against the right coplanar regions by comparing 
 * the vector cross products of the eigenvectors second and third axis for all minimal right regions in the
 * yTolerance of the left Y scan line assigned to the thread calling the method. 
 * Experiments have found that this yields a single candidate in most real world cases so far but the code is such 
 * that in case of multiple matches the one with the smallest difference in eigenvector values to the left region is chosen.
 * The minimum points per octree cell can be changed in hough_settings but typically minimum 5 points. 
 * The coplanar region found by PCA may contain more however.
 * This is analogous to the standard stereo block matching (SBM) done by brute force sum of absolute differences (SAD) of the RGB
 * values of left and right windows of pixels, but instead using a more evolved approach of comparing the essence 
 * of the orientation of small regions of marker points in each image, more akin to the way organisms seem to do it.
 *
 * 5.) Assign a disparity to the minimal octree region based on differences in epipolar distance values of the 2 octree 
 * centroids of the points in the coplanar minimal regions from step 4. The regions are small enough that 
 * centroid value is accurate enough to give reasonable disparity, identical to standard black matching
 *
 * 6.) Reset left image octree and regenerate coplanar regions via PCA at a higher octree level, using level 5. The concept
 * is that regions that were coplanar on a smaller level and are coplanar at a higher level still, form one region of interest.
 *
 * 7.) Find the minimal regions (that were assigned a depth) enclosed by the newly generated larger coplanar regions.
 *
 * 8.) For all points in the larger regions, determine which enclosed minimal region they fall into. If within the 
 * smaller region, assign the point the depth we found and assigned to that minimal region earlier in step 5.
 *
 * 9.) For points in the larger region that do not fall into a smaller enclosed region, find the closest edge of 
 * a smaller enclosed region they are near and assign the depth of that region to the point.
 *
 * 10.) After processing a larger region, remove the smaller regions it encloses from consideration.
 *
 * 11.) Regenerate coplanar regions via PCA  at a higher octree level, using level 4, if there are unprocessed smaller
 *  regions (regions that were not enclosed by a larger one) .
 *
 * 12.) For all minimal regions not previously processed, perform steps 7,8,9,10 although it seems that in 
 * practice all regions are processed without having to use the larger octree nodes at level 4.
 * 
 * Once completed, a set of coplanar octree regions that can be used to navigate or for further processing into larger coplanar
 * areas is available.
 *
 * @author jg (C) NeoCoreTechs 2018,2019
 *
 */
public class VideoViewer extends AbstractNodeMain 
{
	private static boolean DEBUG = false;
	private static boolean DEBUGTEST3 = true;
	private static boolean DEBUGTEST2 = true;
	private static final boolean SAMPLERATE = false; // display pubs per second
	private static final boolean TIMER = true;
	private static final boolean WRITEFILES = true;

    private BufferedImage imageL = null;
    private BufferedImage imageR = null;
    private BufferedImage imageLx = null;
    private BufferedImage imageRx = null;
    private BufferedImage imageT = null;
    private PlayerFrame displayPanel;
    private boolean viewChanged = false;
    
    // slider to control horizontal rotation
    final JSlider headingSlider = new JSlider(-180, 180, 0);
    // slider to control vertical rotation
	final JSlider pitchSlider = new JSlider(SwingConstants.VERTICAL, -90, 90, 0);
	//int outWidth = 1280;
	//int outHeight = 1000;
	int outWidth = 640;
	int outHeight = 480;
    double[] zBuffer = new double[outWidth*outHeight];
    
    ByteBuffer cbL, cbR;
    byte[] bufferL = new byte[0];
    byte[] bufferR = new byte[0];
   
	String mode = "";
	String outDir = "/users/jg/workspace/robocore/motionclouds";
	int frames = 0;
	int files = 0;
	int[] ibuf = null;
	// optical parameters of Logitech C310 and others

    final static float f = 4.4f; // focal length mm
    final static float B = 205.0f; // baseline mm
    final static float FOVD = 60; // degrees field of view
    final static double FOV = 1.04719755; // radians FOV
    final static int camWidth = 640; // pixels
    final static int camHeight = 480;
    final static double fp = B*(camWidth*.5)/Math.tan(FOV *.5 * (Math.PI/2)); //focal length in pixels
    final static double Bf = B*f;// calc the Bf of Bf/d
    // block matching constants
    final static int maxHorizontalSep = (int) (Bf-1)/4; // max pixel disparity
    final static int corrWinSize = 15; // Size of eventual depth patch
    final static int corrWinLeft = 7; // number of left/up elements in corr window
    final static int corrWinRight = 8;// number of right/down elements in corr window
    final static int yTolerance = 25; // pixel diff in y of potential candidates

    CircularBlockingDeque<BufferedImage> queueL = new CircularBlockingDeque<BufferedImage>(10);
    CircularBlockingDeque<BufferedImage> queueR = new CircularBlockingDeque<BufferedImage>(10);
    
	CyclicBarrier latchTrans = new CyclicBarrier(camHeight/corrWinSize);
	
    //static int threads = 0;

    byte[] bqueue;
	private int sequenceNumber,lastSequenceNumber;
	long time1;

	Object mutex = new Object();
	//FloatDCT_2D fdct2dL = new FloatDCT_2D(corrWinSize, corrWinSize);
	//FloatDCT_2D fdct2dR = new FloatDCT_2D(corrWinSize, corrWinSize);

    CannyEdgeDetector ced = null;
    
	BufferedImage bimage = null;
	double[][][] simage = null; // [x][y]([r][g][b][d])
	octree_t node = null;
	octree_t nodel = null;
	octree_t noder = null;
	List<IndexDepth> indexDepth;
	List<IndexDepth> indexUnproc;
	HoughTransform htL = null;
	HoughTransform htR = null;
	ArrayList<? extends HoughElem> hlL;
	ArrayList<? extends HoughElem> hlR;
	int[] dataL; // left array return from canny with magnitudes
	int[] dataR; // right canny array with magnitudes
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
	Matrix3 transform;
	//int yStart;
	int threads = 0;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("subs_videoproc");
	}
	
	@Override
	public void onStart(final ConnectedNode connectedNode) {

		Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
		if( remaps.containsKey("__mode") )
			mode = remaps.get("__mode");
		if( mode.startsWith("display") || mode.equals("hough")) {
			if( DEBUG )
				System.out.println("Pumping frames to AWT Panel");
			SwingUtilities.invokeLater(new Runnable() {
			    public void run() {
			        displayPanel = new PlayerFrame();
			    }
			});
		}
		// construct gaussian kernel with peak at 127
		//final int[][] kernel = ImgProcessor.gaussianNormal(127,kernWinSize);

		/**
		 * Thread to precompute the correspondence windows for each pixel in a new image and store those
		 * to the array backed blocking queue for retrieval by the main processing thread.
		 * This thread works strictly on left source image, while the main thread processes the right image with
		 * the kernels generated here and stored on the queue.
		ThreadPoolManager.getInstance().spin(new Runnable() {
				@Override
				public void run() {
					int[][] imgsrc = new int[corrWinSize][corrWinSize]; // left image subject pixel and surrounding
			        while(true) {
			        	synchronized(mutex) {
			        		try {
								mutex.wait();
							} catch (InterruptedException e) {}
			        	}
			        	corrWins.clear();
		        		
		        		// xsrc is index for subject pixel
		        		// correlation window assumed symmetric left/right/up/down with odd number of elements
		        		for(int y = corrWinLeft; y < imageL.getHeight()-corrWinRight; y++) {
		        			// for each subject pixel X in left image, sweep the scanline for that Y, move to next subject X
		        			for(int xsrc = corrWinLeft; xsrc < imageL.getWidth()-corrWinRight; xsrc++) {
		        				// check if main thread signaled we found a perfect score
		        				if(sadZero.get() != 0) {
		        					corrWins.clear();
		        					if( sadZero.get() != y)
		        						System.out.println("**POSSIBLE MISMATCH WITH WINDOW GENERATOR**");
		        					sadZero.set(0);
		        					break; // go to next y
		        				}
		        				// sweep the epipolar scan line for each subject pixel in left image
		        				// first create the subject pixel kernel and weight it
		        				int kx = 0;
		        				for(int i = xsrc-corrWinLeft; i < xsrc+corrWinRight; i++) {
		        						int ky = 0;
		        						for(int j = y-corrWinLeft; j < y+corrWinRight; j++) {
		        							// weight the high 8 bits with our gaussian distro values which have 127 max so sign should be unaffected
		        							imgsrc[kx][ky] = (imageL.getRGB(i,j) & 0xFFFFFF) | (kernel[kx][ky] << 24);
		        							++ky;
		        						}
		        						++kx;
		        				}
		        				corrWins.add(imgsrc);
		        			}
		        		}
			        }
				}
		}, "SYSTEM");
		*/
		/*
		 * Main worker thread for image data. 
		 * Wait at synch barrier for completion of all processing threads, then display disparity 
		 */
		final AtomicInteger yStart = new AtomicInteger(0);
		SynchronizedFixedThreadPoolManager.getInstance().init(16, 16);
		SynchronizedFixedThreadPoolManager.spin(new Runnable() {
				@Override
				public void run() {
					System.out.println("Processing "+camWidth+" by "+camHeight);
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
					  yStart.set(0);
					  int numThreads, execLimit;
					  switch(mode) {
						// Display results as greyscale bands of depth overlayed on image in a java window
						case "display":
						// Display a 3Dish rendering of image with depth values and sliders for orientation
						case "display3D":
						// Perform canny edge detect then depth value those and overlay, writing CloudCompare file
						case "edge":
							numThreads = camHeight/corrWinSize;
							execLimit = camHeight-corrWinSize;
							break;
						case "test":
							numThreads = camHeight/10;
							execLimit = camHeight;
							break;
						default:
							numThreads = camHeight/corrWinSize;
							execLimit = camHeight-corrWinSize;
							break;
					  }
					  //
					  // spin all threads necessary for execution
					  //
					  SynchronizedFixedThreadPoolManager.resetLatch(execLimit);
					  for(int syStart = 0; syStart < execLimit; syStart++) {
						//for(; threads < camHeight/corrWinSize; threads++) {
						//System.out.println("Spinning thread "+yStart);
						//ThreadPoolManager.getInstance().spin(new Runnable() {
						SynchronizedFixedThreadPoolManager.spin(new Runnable() {
						  //int yStart = threads*corrWinSize;
						  //int yEnd = yStart+corrWinSize-1;
						  @Override
						  public void run() {
							//try {
									//while(true) {
										//try {
											//latch.await();
											// Since we run a continuous loop inside run we have to have a way
											// to synchronize everything at the beginning of a new image, then
											// await completion of all processing threads and signal a reset to
											// resume processing at the start of another new image, hence the two barrier
											// synchronization latches
											switch(mode) {
												// Display results as greyscale bands of depth overlayed on image in a java window
												case "display":
													processImageChunk(imageL, imageR, yStart.getAndIncrement(), camWidth, bimage);
													break;
												// Display a 3Dish rendering of image with depth values and sliders for orientation
												case "display3D":
													processImageChunk3D(imageL, imageR, yStart.getAndIncrement(), camWidth, bimage);
													break;
												// Perform canny edge detect then depth value those and overlay, writing CloudCompare file
												case "edge":
													processImageChunkEdge(dataL, dataR, imageLx, imageRx, yStart.getAndIncrement(), camWidth, node);
													break;
												case "test":
													imagesToOctrees(dataL, dataR, imageLx, imageRx, yStart.getAndIncrement(), camWidth, camHeight, nodel, noder/*hlL, hlR*/);
													break;
												// Perform depth then deliver array of values
												default:
													//processImageChunk(imageL, imageR, imageT, yStart, camWidth, transform, true, simage);
													processImageChunk(imageL, imageR, imageT, yStart.getAndIncrement(), camWidth, null, false, simage);
											}
											//latchOut.await();
										//} catch (BrokenBarrierException e) { System.out.println("<<BARRIER BREAK>> "+this);}
									//}
							//} catch (InterruptedException e) { e.printStackTrace(); }
						  } // run
					    }); // spin
					  } // for syStart
					  //BlockingQueue<Runnable> bq = FixedThreadPoolManager.getInstance(camHeight-corrWinSize).getQueue();
					  // wait for the y scan atomic counter to reach max
					  //while(yStart.get() < camHeight-corrWinSize) Thread.sleep(1);
					  SynchronizedFixedThreadPoolManager.waitForGroupToFinish();
					  if( TIMER )
						  System.out.println("Process time one="+(System.currentTimeMillis()-etime));
					  latchOut.await();
					  //
					  // next parallel processing step, if any
					  //
					  switch(mode) {
						case "test":
							final Comparator<octree_t> yComp = new Comparator<octree_t>() {         
							    @Override         
							    public int compare(octree_t jc1, octree_t jc2) {             
							      return (jc2.getCentroid().y < jc1.getCentroid().y ? -1 :                     
							              (jc2.getCentroid().y == jc1.getCentroid().y ? 0 : 1));           
							    }     
							  }; 
							latch2.await();
							etime = System.currentTimeMillis();
							yStart.set(0);
							ArrayList<octree_t> tnodel = nodel.get_nodes();
							ArrayList<octree_t> tnoder = noder.get_nodes();
							Collections.sort(tnodel, yComp);
							Collections.sort(tnoder, yComp);
							final List<octree_t> nodelA = Collections.synchronizedList(tnodel);
							final List<octree_t> noderA = Collections.synchronizedList(tnoder);
							indexDepth = Collections.synchronizedList(new ArrayList<IndexDepth>());
							indexUnproc = Collections.synchronizedList(new ArrayList<IndexDepth>());
							for(int syStart = 0; syStart < execLimit; syStart++) {
								SynchronizedFixedThreadPoolManager.spin(new Runnable() {
									@Override
									public void run() {
										// set the left nodes with depth
										matchRegionsAssignDepth(yStart.getAndIncrement(), camWidth, camHeight, nodelA, noderA, indexDepth, indexUnproc);
									} // run									
								}); // spin
							} // for syStart
							SynchronizedFixedThreadPoolManager.getInstance().waitForGroupToFinish();
							if( TIMER )
								System.out.println("Process time two="+(System.currentTimeMillis()-etime));
							latchOut2.await();
						break;
						//
						default:
					  }
					  //
					  // next parallel processing step, if any
					  //
					  switch(mode) {
						case "test":
							latch3.await();
							etime = System.currentTimeMillis();
							yStart.set(0);
							final List<octree_t> nodelA = Collections.synchronizedList(nodel.get_nodes());
							final int nSize = nodelA.size();
							final int nSizeT = Math.min(nodelA.size(), 32);
							// gen 1 thread for each array element up to limit
							SynchronizedFixedThreadPoolManager.init(nSizeT, nSize, new String[]{"SETPOINT"});
							for(int syStart = 0; syStart < nSize; syStart++) {
								SynchronizedFixedThreadPoolManager.spin(new Runnable() {
									@Override
									public void run() {
										// set the left nodes with depth
										findEnclosedRegionsSetPointDepth(yStart.getAndIncrement(), nodelA, indexDepth);
									} // run									
								},"SETPOINT"); // spin
							} // for syStart
							SynchronizedFixedThreadPoolManager.waitForGroupToFinish("SETPOINT");
							if( TIMER )
								System.out.println("Process time three="+(System.currentTimeMillis()-etime));
							latchOut3.await();
						break;
						//
						default:
					  }
					  //
					  // next parallel processing step, if any
					  //
					  /*
					  switch(mode) {
						case "test":
							latch4.await();
							etime = System.currentTimeMillis();
							if( !indexDepth.isEmpty() ) {
								//yStart.set(0);
								final ArrayList<octree_t> nodelA = nodel.get_nodes();
								final int nSize = nodelA.size();
								// gen 1 thread for each array element
								for(int syStart = 0; syStart < nSize; syStart++) {
								//SynchronizedFixedThreadPoolManager.getInstance(nSize/10, nSize).spin(new Runnable() {
									//@Override
									//public void run() {
										// set the left nodes with depth
									findEnclosedRegionsSetPointDepth(yStart.getAndIncrement(), nodelA, indexDepth);
									//} // run									
								//}); // spin
								} // for syStart
							}
							//SynchronizedFixedThreadPoolManager.getInstance(nSize/10, nSize).waitForGroupToFinish();
							if( TIMER )
								System.out.println("Process time four="+(System.currentTimeMillis()-etime));
							latchOut4.await();
						break;
					  }
					  */
					// global barrier break
					} catch (BrokenBarrierException | InterruptedException e) { System.out.println("<<BARRIER BREAK>> "+this);}
					} // while true
			} // run
			//} catch (BrokenBarrierException | InterruptedException e) { System.out.println("<<BARRIER BREAK>> "+this);}
		}); // spin
					
		/**
		 * Main processing thread for image data. Extract image queue elements from ROS bus and then
		 * notify waiting worker threads to process them.
		 * Wait at synch barrier for completion of all processing threads, then display disparity 
		 */
		SynchronizedFixedThreadPoolManager.init(1, Integer.MAX_VALUE, new String[] {"SYSTEMX"});
		SynchronizedFixedThreadPoolManager.spin(new Runnable() {			
				public void run() {
					System.out.println("Image queue..");
					/**
					 * Main processing loop, extract images from queue, notify worker threads, then display disparity
					 */
			        while(true) {
			        	if( queueL.isEmpty() || queueR.isEmpty() ) {
							try {
								Thread.sleep(1);
							} catch (InterruptedException e) {}
			        	} else {
			        		try {
			        			// If we are waiting at either cyclic barrier, the reset will cause it to
			        			// to return to top level barrier
			        			// the latches represent each 'step', or group of parallel processing tasks
			        			// use as many steps as needed, unused latches ignored
								imageLx = queueL.takeFirst();
				        		imageRx = queueR.takeFirst();
			        			//latchOut4.reset();
			        			//latch4.reset();
			        			latchOut3.reset();
			        			latch3.reset();
			        			latchOut2.reset();
			        			latch2.reset();
			        			latchOut.reset();
			        			latch.reset();
				        		//
				        		switch(mode) {
				        		case "display3D":
				        			imageL = imageLx;
				        			imageR = imageRx;
				        			//score = new int[imageL.getWidth()];
				        			if( bimage == null ) {
				        				bimage = new BufferedImage(outWidth, outHeight, BufferedImage.TYPE_INT_ARGB);
				        				//bimage = new BufferedImage(outWidth, outHeight, BufferedImage.TYPE_INT_RGB);
				        				synchronized(bimage) {
			        						Graphics g = bimage.getGraphics();
			        						g.setColor(Color.WHITE);
			        						g.fillRect(0, 0, outWidth, outHeight);
			        					}
				        			}
			        				//System.out.println("setting display "+bimage);
			        				latch.await();
			        	     		latchOut.await();
			        				displayPanel.setLastFrame(bimage);
			        				displayPanel.invalidate();
			        				displayPanel.updateUI();
			        				if( viewChanged) {
			        					viewChanged = false;
			        				//	synchronized(bimage) {
			        				//		Graphics g = bimage.getGraphics();
			        				//		g.setColor(Color.WHITE);
			        				//		g.fillRect(0, 0, outWidth, outHeight);
			        				//	}
			        				}
			        				break;
				        		case "display":
				        			imageL = imageLx;
				        			imageR = imageRx;
				        			if( bimage == null ) {
				        				bimage = new BufferedImage(outWidth, outHeight, BufferedImage.TYPE_INT_ARGB);
				        			}
			        				//System.out.println("setting display "+bimage);
			        				latch.await();
			        	     		latchOut.await();
			        				displayPanel.setLastFrame(bimage);
			        				displayPanel.invalidate();
			        				displayPanel.updateUI();
			        				break;
				        		case "edge":		
					        	     	synchronized(imageLx) {
					        	     		ced = new CannyEdgeDetector();
					        				ced.setSourceImage(imageLx);
					        				dataL = ced.semiProcess();
					            			//ced.process();
					            			//imageL = ced.getEdgesImage();
					        	     	}
					        	     	synchronized(imageRx) {
					        	     		ced = new CannyEdgeDetector();
					        				ced.setSourceImage(imageRx);
					        				dataR = ced.semiProcess();
					            			//ced.process();
					            			//imageR = ced.getEdgesImage();
					        	     	}
					        	     		
				        				imageL = imageLx;
				        				imageR = imageRx;	
				        				//if( simage == null)
				        				//	simage = new double[outWidth][outHeight][4];
				        				node = new octree_t();
				        				octree_t.buildStart(node);
				        		   		// write source images
				        	     		synchronized(imageL) {
				        	     			writeFile("sourceL", imageL, ++files);
				        	     		}
				        	     		synchronized(imageR) {
				        	       			writeFile("sourceR", imageR, files);
				        	     		}
				        				latch.await();
				        	     		latchOut.await();
				        	     		//synchronized(simage) {
				        	     		//	writeFile(simage);
				        	     		//	writeOctree(simage);
				        	     		//}
				        	     		//break;
				        	     		synchronized(node) {
				        	     			octree_t.buildEnd(node);
				        	     			node.subdivide();
				        	     			writeFile(node,"/roscoe"+(++frames));
				        	     			writer_file.writePerp(node, "planars"+frames);
				        	     		}
				        	     		break;
				        		case "test":
				        			/*
				        			if( htL == null)
				        				htL = new HoughTransform(camWidth, camHeight);
				        			else
				        				htL.clear();
				        			if( htR == null)
				        				htR = new HoughTransform(camWidth, camHeight);
				        			else
				        				htR.clear();
				        			*/
				        	     	synchronized(imageLx) {
				        	     		ced = new CannyEdgeDetector();
				        	     		ced.setLowThreshold(0.5f);
				        	     		ced.setHighThreshold(1f);
				        				ced.setSourceImage(imageLx);
				        				dataL = ced.semiProcess();
				        				/*
				        				for(int j = 0; j < camWidth; j++) {
				        					for(int i = 0; i < camHeight; i++) {
				        						if( dataL[j*camWidth+i] != 0) {
				        							htL.addPoint(j, i);
				        						}
				        					}
				        				}
				        				*/
				            			//ced.process();
				            			//imageL = ced.getEdgesImage();
				        	     	}
				        	     	synchronized(imageRx) {
				        	     		ced = new CannyEdgeDetector();
				        	     		ced.setLowThreshold(0.5f);
				        	     		ced.setHighThreshold(1f);
				        				ced.setSourceImage(imageRx);
				        				dataR = ced.semiProcess();
				        				/*
				        				for(int j = 0; j < camWidth; j++) {
				        					for(int i = 0; i < camHeight; i++) {
				        						if( dataR[j*camWidth+i] != 0) {
				        							htR.addPoint(j, i);
				        						}
				        					}
				        				}
				        				*/
				            			//ced.process();
				            			//imageR = ced.getEdgesImage();
				        	     	}
				        	     	/*
				        	     	hlL = htL.getLines(10);
				        	     	hlR = htR.getLines(10);
				        	     	*/
			        				imageL = imageLx;
			        				imageR = imageRx;	
			        				nodel = new octree_t();
			        				octree_t.buildStart(nodel);
			        				noder = new octree_t();
			        				octree_t.buildStart(noder);
			        		   		// write source images
			        				if( WRITEFILES ) {
			        					synchronized(imageL) {
			        						writeFile("sourceL", imageL, ++files);
			        					}
			        					synchronized(imageR) {
			        						writeFile("sourceR", imageR, files);
			        					}
			        				}
			        	     		// first step end multi thread barrier synch
			        				latch.await();
			        	     		latchOut.await();
			        	     		synchronized(nodel) {
			        	     			octree_t.buildEnd(nodel);
			        	     			hough_settings.s_level = 7;
			        	     			hough_settings.max_distance2plane = 5;
			        	     			nodel.subdivide();
			        	     			//writeFile(nodel,"/roscoeL"+(++frames));
			        	     			if( WRITEFILES)
			        	     				writer_file.writePerp(nodel, "planarsL"+files);
			        	     		}
			        	     		synchronized(noder) {
			        	     			octree_t.buildEnd(noder);
			        	     			noder.subdivide();
			        	     			if( WRITEFILES) {
			        	     				writeFile(noder,"/roscoeR"+files);
			        	     				writer_file.writePerp(noder, "planarsR"+files);
			        	     			}
			        	     		}
			        	     		// second step end multi thread barrier synch
			        	     		latch2.await();
			        	     		latchOut2.await();
			        	     		// write unmatched minimal envelopes
			        	     		if( WRITEFILES) {
			        	     			synchronized(indexUnproc) {
			        	     				writeFile(indexUnproc, "/lvl7uncorrL"+files);
			        	     			}
			        	     		}
			        	     		System.out.println("Uncorrelated regions="+indexUnproc.size()+" correlated="+indexDepth.size()+", "+(((float)indexUnproc.size()/(float)(indexUnproc.size()+indexDepth.size()))*100.0)+"%");
			        	     		// reset level then regenerate tree with maximal coplanar points
			        	     		synchronized(nodel) {
			        	     			// set our new maximal level
			        	     			hough_settings.s_level = 4;
			        	     			// set the distance to plane large to not throw out as outliers points we recently assigned a z
			        	     			// this is a divisor so we set to 1
			        	     			hough_settings.max_distance2plane = 1;
			        	     			nodel.clear();
			        	     			nodel.subdivide();
			        	     			// write the display cloud with maximal envelopes
			        	     			if(WRITEFILES)
			        	     				writer_file.writeEnv(nodel, "lvl4planenvL"+files);
			        	     		}
			        	     		// third step wait for completion
			        	     		latch3.await();
			        	     		latchOut3.await();
			        	     		// start step 4
			        	     		System.out.println("Remaining unenclosed minimal envelopes="+indexDepth.size());
			        	     		synchronized(nodel) {
			        	     			// set our new maximal level
			        	     			hough_settings.s_level = 5;
			        	     			// set the distance to plane large to not throw out as outliers points we recently assigned a z
			        	     			// this is a divisor so we set to 1
			        	     			hough_settings.max_distance2plane = 5;
			        	     			nodel.clear();
			        	     			nodel.subdivide();
			        	     			// write the display cloud with maximal planes detected
			        	     			if(WRITEFILES) {
			        	     				writer_file.writePerp(nodel, "planesL"+files);
			        	     			}
			        	     		}
			        	     		// wait for step 4 complete
			        	     		//latch4.await();
			        	     		//latchOut4.await();
			        	     		if(WRITEFILES) {
			        	     			// write the remaining unprocessed envelopes from minimal
			        	     			if(!indexDepth.isEmpty())
			        	     				synchronized(indexDepth) {
			        	     					writeFile(indexDepth,"/lvl7unencL"+files);
			        	     				}
			        	     			// at this point the entire point set should be loaded with z
			        	     			synchronized(nodel) {
			        	     				writeFile(nodel,"/roscoeL"+files);
			        	     			}
			        	     		}
			        	     		break;
				        	     	default:
				        				imageL = imageLx;
				        				imageR = imageRx;
				        				/*
				        				transform = new Matrix3(new double[] /*{0.99939083, 0.01020363, -0.03337455,
				        						0.00000000, 0.95630476, 0.29237170,
				        						0.03489950, -0.29219360,0.95572220});
				        							{0.99984770,    -0.00091339,    -0.01742849,
				        						    0.00000000,     0.99862953,    -0.05233596,
				        						    0.01745241,     0.05232799,     0.99847744});
				        				if( imageT == null )
					        				imageT = new BufferedImage(outWidth, outHeight, BufferedImage.TYPE_INT_RGB);
				        				*/
				        				if( simage == null)
				        					simage = new double[outWidth][outHeight][4];
				        		   		// write source images
				        	     		synchronized(imageL) {
				        	     			writeFile("sourceL", imageL, ++files);
				        	     		}
				        	     		synchronized(imageR) {
				        	       			writeFile("sourceR", imageR, files);
				        	     		}
				        	     		//latchTrans.reset();
				        				latch.await();
				        	     		latchOut.await();
				        	     		synchronized(simage) {
				        	     			writeFile(simage);
				        	     		}
				        	     		/*
				        	     		synchronized(imageT) {
				        	     			writeFile("sourceT", imageT, files);
				        	     		}
				        	     		*/
				        	     		break;
				        				//navigate(simage, 213, 426);
				        			
				        		}
				        		// tell the image processing threads that a new image is ready
				        		//latch.await();
				        		// now wait for the threads to finish processing
				        		//latchOut.await();
							} catch (InterruptedException | BrokenBarrierException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
							}
			        		//imageL = null;
			        		//imageR = null;
			        	}
			        } // while true
				} // run      
		}, "SYSTEMX"); // spin
		
		final Subscriber<stereo_msgs.StereoImage> imgsub =
				connectedNode.newSubscriber("/stereo_msgs/StereoImage", stereo_msgs.StereoImage._TYPE);
	
		/**
		 * Image extraction from bus, then image processing, then on to display section.
		 */
		imgsub.addMessageListener(new MessageListener<stereo_msgs.StereoImage>() {
		@Override
		public void onNewMessage(stereo_msgs.StereoImage img) {
			long slew = System.currentTimeMillis() - time1;
			if( SAMPLERATE && slew >= 1000) {
				time1 = System.currentTimeMillis();
				System.out.println("Samples per second:"+(sequenceNumber-lastSequenceNumber)+". Slew rate="+(slew-1000));
				lastSequenceNumber = sequenceNumber;
			}
			try {
				//synchronized(mutex) {
					cbL = img.getData();
					bufferL = cbL.array();
					InputStream in = new ByteArrayInputStream(bufferL);
					BufferedImage imageL1 = ImageIO.read(in);
					in.close();
					cbR = img.getData2();
					bufferR = cbR.array();
					in = new ByteArrayInputStream(bufferR);
					BufferedImage imageR1 = ImageIO.read(in);
					in.close();
					queueL.addLast(imageL1);
					queueR.addLast(imageR1);
				//}
				//IntBuffer ib = cb.toByteBuffer().asIntBuffer();
				if( DEBUG ) {
					System.out.println("New left/right images "+img.getWidth()+","+img.getHeight()+" size:"+bufferL.length/*ib.limit()*/);
				}
			//image = new BufferedImage(img.getWidth(), img.getHeight(), BufferedImage.TYPE_3BYTE_BGR);
			//image = new BufferedImage(img.getWidth(), img.getHeight(), BufferedImage.TYPE_4BYTE_ABGR);
			/*
			image = new BufferedImage(img.getWidth(), img.getHeight(), BufferedImage.TYPE_INT_ARGB);
			WritableRaster raster = (WritableRaster) image.getRaster();
			int ibsize = img.getHeight() * img.getWidth();
			if( ibuf == null )
				ibuf = new int[ibsize * 4];
			int iup = 0;
			int ip = 0;
			for(int i = 0; i < ibsize; i++) {
				int ialph = 255; //(ib.get(i) >> 24) & 0x0ff;
				//int ired = (ib.get(i) >> 16) & 0x0ff; 
				//int igreen = (ib.get(i) >> 8 ) & 0x0ff;
				//int iblue = (ib.get(i) & 0x0ff);
				int iblue = bufferL[ip++];
				int igreen = bufferL[ip++];
				int ired = bufferL[ip++];
				ibuf[iup++] = ired;
				ibuf[iup++] = igreen;
				ibuf[iup++] = iblue;
				ibuf[iup++] = ialph;
			}
			//System.out.println(ibuf.length+" "+raster.getWidth()+" "+raster.getHeight()+" "+raster.getMinX()+" "+raster.getMinY());
		    raster.setPixels(0, 0, img.getWidth(), img.getHeight(), ibuf);
			*/
	
			} catch (IOException e1) {
				System.out.println("Could not convert image payload due to:"+e1.getMessage());
				return;
			}
				//displayPanel.setLastFrame((java.awt.Image)image);
				//displayPanel.setLastFrame(displayPanel.createImage(new MemoryImageSource(newImage.imageWidth
				//		, newImage.imageHeight, bufferL, 0, newImage.imageWidth)));
				//displayPanel.invalidate();
				//displayPanel.updateUI();
				//queue.addLast(image);
			++sequenceNumber; // we want to inc seq regardless to see how many we drop	
		}
	  });
		
	} // onstart
	
	/**
	 * Write CloudCompare point cloud viewer compatible file
	 * for each point - X,Y,Z,R,G,B ascii delimited by space
	 * @param simage2 the processed array chunks of [x][y][0]=R, [x][y][1]=G, [x][y][2]=B, [x][y][3]=D
	 */
	protected void writeFile(double[][][] simage2) {
		DataOutputStream dos = null;
		File f = new File(outDir+"/roscoe"+(++frames)+".asc");
		int ks, ms;
		double os;
		try {
			dos = new DataOutputStream(new FileOutputStream(f));
			for(int y = 0; y < outHeight; y++) {
				for(int x = 0; x < outWidth; x++) {
					// output only valid data
					if( mode.equals("edge") && simage2[x][y][0] == 0 )
						continue;
					// transform to 3D plane offsets
					ks = x - (camWidth/2);
					ms = y - (camHeight/2);
					os = (Bf/2) - simage2[x][y][3] ;//simage2[x][y][3] - (Bf/2);
					dos.writeBytes(String.valueOf((double)ks/*x*/)); // X
					dos.writeByte(' ');
					dos.writeBytes(String.valueOf((double)ms/*y*/));// Y
					dos.writeByte(' ');
					dos.writeBytes(String.valueOf(os/*simage2[x][y][3]*/)); // Z
					dos.writeByte(' ');
					dos.writeBytes(String.valueOf((int)simage2[x][y][0])); // R
					dos.writeByte(' ');
					dos.writeBytes(String.valueOf((int)simage2[x][y][1])); // G
					dos.writeByte(' ');
					dos.writeBytes(String.valueOf((int)simage2[x][y][2])); // B
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
		
	}
	
	protected void writeFile(octree_t node, String filename) {
		DataOutputStream dos = null;
		File f = new File(outDir+filename+".asc");
		int ks, ms;
		double os;
		try {
			dos = new DataOutputStream(new FileOutputStream(f));
			for(int i = 0; i < node.m_points.size(); i++) {
				Vector4d pnode = node.m_points.get(i);
				Vector4d pcolor = node.m_colors.get(i);
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
	protected void writeFile(List<IndexDepth> d, String filename) {
		DataOutputStream dos = null;
		File f = new File(outDir+filename+".asc");
		try {
			dos = new DataOutputStream(new FileOutputStream(f));
			for(IndexDepth id : d) {
				writer_file.line3D(dos, (int)id.xmin, (int)id.ymin, 1, (int)id.xmax, (int)id.ymin, 1, 0, 255, 255);
				writer_file.line3D(dos, (int)id.xmax, (int)id.ymin, 1, (int)id.xmax, (int)id.ymax, 1, 0, 255, 255);
				writer_file.line3D(dos, (int)id.xmax, (int)id.ymax, 1, (int)id.xmin, (int)id.ymax, 1, 0, 255, 255);
				writer_file.line3D(dos, (int)id.xmin, (int)id.ymax, 1, (int)id.xmin, (int)id.ymin, 1, 0, 255, 255);
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
	protected void writeOctree(double[][][] simage2) {
		octree_t node = new octree_t();
		octree_t.buildStart(node);
		for(int y = 0; y < outHeight; y++) {
			for(int x = 0; x < outWidth; x++) {
				// output only valid data
				if( mode.equals("edge") && simage2[x][y][0] == 0 )
					continue;
				// transform to 3D plane offsets
				double ks = x - (camWidth/2);
				double ms = y - (camHeight/2);
				double os = (Bf/2) - simage2[x][y][3] ;//simage2[x][y][3] - (Bf/2);
				octree_t.build(node, (double)ks, (double)ms, os, simage2[x][y][0], simage2[x][y][1], simage2[x][y][2]);
			}
		}
		octree_t.buildEnd(node);
		node.subdivide();
		writer_file.writePerp(node, "planars"+frames);
	}
	

	/**
	 * Write JPEG file 
	 * @param fname File name
	 * @param image BufferedImage with data
	 * @param findex Sequence to append to file name for consecutive frames
	 */
	protected void writeFile(String fname, BufferedImage image, int findex) {
		File f = new File(outDir+"/"+fname+(findex)+".jpg");
		try {
			ImageIO.write(image, "jpg", f);
			if( DEBUG )
				System.out.println("Wrote: "+fname+findex+".jpg");
		} catch (IOException e) {
			System.out.println("Cant write image "+fname+findex+".jpg");
			return;
		}
	}
	/**
	 * Process the stereo images into a 3D rendered image, orientation is based on horizontal and vertical sliders
	 * in the panel. The horizontal slider is heading, vertical slider is pitch.
	 * In practice the processing threads will segment the image into several regions to be processed in parallel
	 * differentiated by yStart and yEnd.
	 * @param imageL Left image from bus
	 * @param imageR Right image from bus
	 * @param yStart Starting Y band to process
	 * @param width width of image band
	 * @param bimage buffered image to hold 3D rendering
	 */
	public void processImageChunk3D(BufferedImage imageL, BufferedImage imageR, int yStart, int width, BufferedImage bimage) {
			double heading = Math.toRadians(headingSlider.getValue());
   			Matrix3 headingTransform = new Matrix3(new double[] {
                    Math.cos(heading), 0, -Math.sin(heading),
                    0, 1, 0,
                    Math.sin(heading), 0, Math.cos(heading)
            });
   			double pitch = Math.toRadians(pitchSlider.getValue());
   			Matrix3 pitchTransform = new Matrix3(new double[] {
                    1, 0, 0,
                    0, Math.cos(pitch), Math.sin(pitch),
                    0, -Math.sin(pitch), Math.cos(pitch)
            });
   			Matrix3 transform = headingTransform.multiply(pitchTransform);
   			if( viewChanged) {
   				System.out.println(Thread.currentThread().getName()+" "+yStart+","+(yStart+corrWinSize )+"\r\n"+transform);
   			}
			
			// initialize array with extremely far away depths
			//for (int q = 0; q < zBuffer.length; q++) {
			//	zBuffer[q] = Double.NEGATIVE_INFINITY;
			//}
    		// SAD - sum of absolute differences. take the diff of RGB values of
    		// win 1 and 2 and sum them
    		//int[][] imgsrc = new int[corrWinSize][corrWinSize]; // left image subject pixel and surrounding
    		int[] imgsrcL = new int[width*corrWinSize]; // left image scan line and corr window
     		int[] imgsrcR = new int[width*corrWinSize]; // right image scan line and corr window
    		// xsrc is index for subject pixel
    		// correlation window assumed symmetric left/right/up/down with odd number of elements
    		// so we use corrWinLeft as top half of array size and corrwinRight as bottom half
     		//int close = 0;
     		//int far = 0;
    		//for(int y = yStart; y < yEnd; y+=corrWinSize/*y++*/) {
    			synchronized(imageL) {
    				imageL.getRGB(0, yStart, width, corrWinSize, imgsrcL, 0 , width);
    			}
    			synchronized(imageR) {
    				imageR.getRGB(0, yStart, width, corrWinSize, imgsrcR, 0 , width);
    			}
    			// for each subject pixel X in left image, sweep the scanline for that Y, move to next subject X
    			loop:
    			for(int xsrc = 0; xsrc < width; xsrc++) {
    				// sweep the epipolar scan line for each subject pixel in left image
    				// first create the subject pixel kernel and weight it
    				//int kx = 0;
    				/*
    				for(int i = xsrc-corrWinLeft; i < xsrc+corrWinRight; i++) {
    						int ky = 0;
    						for(int j = y-corrWinLeft; j < y+corrWinRight; j++) {
    							// weight the high 8 bits with our gaussian distro values which have 127 max so sign should be unaffected
    							//imgsrc[kx][ky] = (imageL.getRGB(i,j) & 0xFFFFFF) | (kernel[kx][ky] << 24);
    							imgsrcL[ky*width+i] = (imgsrcL[ky*width+i] & 0xFFFFFF) | (kernel[kx][ky] << 24);
    							++ky;
    						}
    					++kx;
    				}
    				*/
					//int rank = 0;
					//int pix = 0;
					//int winL = xsrc-corrWinLeft;
    				// now begin sweep of scanline at y beginning at 1 plus subject x
    				//for(int x = xsrc+1; x < width-corrWinRight; x++) {
    						// imgsrc at kx,ky is target in weighted left correlation window
    						//kx = 0;
    						//int sum = 0;
    						// outer loop sweeps x in right image starting at subject left image x+1 - left half of
    						// correlation window size, to x+1 + right half of correlation window size
    						//for(int i = x-corrWinLeft; i < x+corrWinRight; i++) {
    							//int ky = 0;
    							// recall; symmetric left/right up down
    							// search at current pixel offset left, to current pixel + offset right, applied to y
    							//for(int j = y-corrWinLeft; j < y+corrWinRight; j++) {
    								//int rrgb = imageR.getRGB(i, j);
    								//int rrgb = imgsrcR[ky*width+i];
    								//rrgb &= 0xFFFFFF;
    								// sum of absolute difference of left image weight+RGB and right image weight+RGB
    								//sum += Math.abs(imgsrc[kx][ky] - (rrgb | (kernel[kx][ky] << 24)));
    								//sum += Math.abs(imgsrcL[ky*width+(winL+kx)] - (rrgb /*|(kernel[kx][ky] << 24)*/));
    								//++ky;
    							//}
    							// increment x of left source weighted correlation window
    							//++kx;
    						//}
    						// score array is sized to image width and we ignore irrelevant edge elements
    						//score[x] = sum;
    						//if( sum <= 127) {
    							//++close;
    							// set sadZero to current y value to signal kernel generator to progress to next y
    							//sadZero.set(y);
    							//rank = x;
		        				//pix = (int)(Bf/Math.abs(xsrc-rank));
		        				//if( pix <=0 || pix >=maxHorizontalSep)
		        					//System.out.print(xsrc+","+y+" p="+pix+"|");
		        				//bimage.setRGB(xsrc, y, (byte)pix); //greyscale option, not so hot
		        				//for( int k = xsrc-corrWinLeft; k < xsrc+corrWinRight; k++) {
		        					//int m = y - corrWinLeft;
		        					for(int l = 0; l < corrWinSize; l++) {		
		        					//for(int l = y-corrWinLeft; l < y+corrWinRight; l++) {
		        						//bimage.setRGB(k, l, ((imageL.getRGB(k,l) & 0xFFFFFF) | pix<<24));
		        						// replace weighted gaussian distro value with computed disparity val and 3D xform
		        						// z depth negative since positive is toward viewer
		        						//int ks, ms;
		        						//int[] c = scale(xsrc,/*l+*/yStart, transform);
		        						//ks = c[0];
		        						//ms = c[1];
		        						//Vertex v0 = depthToVertex(k, m+l, pix, camWidth, camHeight, FOV);
		        						//Point t = new Point(v0, new Color(((imgsrcL[l*width+k] & 0xFFFFFF))));
		               					//Vertex v1 = transform.transform(t);
		               					//Vertex norm = new Vertex(v1.x,v1.y,v1.z);       
		               					//double normalLength = Math.sqrt(norm.x * norm.x + norm.y * norm.y + norm.z * norm.z);
		               					//norm.x /= normalLength;
		               					//norm.y /= normalLength;
		               					//norm.z /= normalLength;
		               					//double angleCos = Math.abs(norm.z);
		               					//int zIndex = (int)v1.y * outWidth + (int)v1.x;
		               					//System.out.println(v1.x+","+v1.y+","+v1.z+","+zIndex+" anglecos="+angleCos);
		               					
		               					
		        						// ks and ms should now be transformed to coord of right image that slides to left
		        						
		               					//if( zIndex < zBuffer.length && zIndex >= 0) {
		               					//	synchronized(zBuffer) {
		               					//		if (zBuffer[zIndex] < v1.z) {
		               					//			boolean oob = false;
		        									//int lgbr = 0xFFFF0000;
		        									//synchronized(imageL) {
		        										//try {
		        											//lgbr = imageL.getRGB(ks, ms);
		        										//} catch(ArrayIndexOutOfBoundsException aioob) {}
		        										
		        									//}
		        						if ((imgsrcL[l*width+xsrc] & 0x00FF0000) == 0x00FF0000 &&
		        							(((imgsrcL[l*width+xsrc] & 0x0000FF00) >>8) >= 135)  &&
		        							(((imgsrcL[l*width+xsrc] & 0x0000FF00) >>8) <= 220)  &&
  	               							(((imgsrcL[l*width+xsrc] & 0x000000FF)) >= 135) &&
  	               							(((imgsrcL[l*width+xsrc] & 0x000000FF)) <= 220)) {
		               								synchronized(bimage) {
		               									try {
		               										bimage.setRGB(xsrc, yStart+l, 0xFFFF0000);
		               										//bimage.setRGB((int)v1.x, (int)v1.y, getShade(t.color, angleCos).getRGB());
		               										// original right target pixel, translated up and over, xor with left subject pixel
		               										//bimage.setRGB(ks, ms, lgbr ^ (imgsrcR[/*l*width+*/xsrc] & 0xFFFFFF));
		               									} catch(ArrayIndexOutOfBoundsException aioob) {
		               										//System.out.println("Coord out of bounds:"+ks+","+ms+" "+Thread.currentThread().getName());
		               										//bimage.setRGB(xsrc, yStart/*+l*/, 0xFFFF0000); // make red
		               					//					oob = true;
		               									}
		               								}
		      										System.out.println("LEFT:"+xsrc+","+(yStart+l)+"|"+
		      												((imgsrcL[l*width+xsrc] & 0xFF000000) >>24)+"|"+
		      	               								((imgsrcL[l*width+xsrc] & 0x00FF0000) >>16)+"|"+
		      	               								((imgsrcL[l*width+xsrc] & 0x0000FF00) >>8)+"|"+
		      	               								(imgsrcL[l*width+xsrc] & 0x000000FF));
		        						}
		        						if ((imgsrcR[l*width+xsrc] & 0x00FF0000) == 0x00FF0000 &&
			        							(((imgsrcR[l*width+xsrc] & 0x0000FF00) >>8) >= 135)  &&
			        							(((imgsrcR[l*width+xsrc] & 0x0000FF00) >>8) <= 220)  &&
	  	               							(((imgsrcR[l*width+xsrc] & 0x000000FF)) >= 135) &&
	  	               							(((imgsrcR[l*width+xsrc] & 0x000000FF)) <= 220)) {
               								synchronized(bimage) {
               									try {
               										bimage.setRGB(xsrc, yStart+l, 0xFF0000FF); //BLUE for right
               										//bimage.setRGB((int)v1.x, (int)v1.y, getShade(t.color, angleCos).getRGB());
               										// original right target pixel, translated up and over, xor with left subject pixel
               										//bimage.setRGB(ks, ms, lgbr ^ (imgsrcR[/*l*width+*/xsrc] & 0xFFFFFF));
               									} catch(ArrayIndexOutOfBoundsException aioob) {
               										//System.out.println("Coord out of bounds:"+ks+","+ms+" "+Thread.currentThread().getName());
               										//bimage.setRGB(xsrc, yStart/*+l*/, 0xFFFF0000); // make red
               					//					oob = true;
               									}
               								}
               								System.out.println("RIGHT:"+xsrc+","+(yStart+l)+"|"+
               								((imgsrcR[l*width+xsrc] & 0xFF000000) >>24)+"|"+
               								((imgsrcR[l*width+xsrc] & 0x00FF0000) >>16)+"|"+
               								((imgsrcR[l*width+xsrc] & 0x0000FF00) >>8)+"|"+
               								(imgsrcR[l*width+xsrc] & 0x000000FF)+"|");
		        						}
		               					//			if( oob )
		               					//				System.out.println("Coord out of bounds:"+v1+" point:"+t+" orig:"+k+","+(m+l)+","+pix);
		               					//			zBuffer[zIndex] = v1.z;
		               					//		}
		               					//	}
		               					//} else {
		               					//	System.out.println("Zindex out of bounds="+zIndex+" "+v1+" point:"+t);
		               					//}
		               					
		        						//imgsrcL[l*width+k] = ((imgsrcL[l*width+k] & 0xFFFFFF) | pix<<24);
		               					//++m;
		        					}
		        				//}
		        				// move to next x at outer x scanline loop
    							//continue loop;
    						//}
    							
    				}
    				// now calculate the one closest to zero from sum of differences
    				// rank = 0
    				//++far;
    				//int drank = Integer.MAX_VALUE;
    				//for(int s = xsrc+1; s < width-corrWinRight; s++) {
    					//System.out.print("Y="+y+" score "+s+"="+score[s]+" ");
    					//if (score[s] < drank) {
    						//rank = s;
    						//drank = score[s];
    					//} 
						/*else {
    						if (score[s] == drank && score[s] > 0 && score[rank] < 0) {
    							//same distance to zero but positive 
    							rank = s;
    						}
    						
    					}*/
    				//}
    				//calc the disparity and insert into disparity map image
    				//pix = (int)(Bf/Math.abs(xsrc-rank));
    				//if( pix <=0 || pix >=maxHorizontalSep)
    				//	System.out.print(xsrc+","+y+" p="+pix+"|");
    				//for( int k = xsrc-corrWinLeft; k < xsrc+corrWinRight; k++) {
    					//int m = y - corrWinLeft;
    					//for(int l = y-corrWinLeft; l < y+corrWinRight; l++) {
    					//for(int l = 0; l < corrWinSize; l++) {
    						//bimage.setRGB(k, l, ((imageL.getRGB(k,l) & 0xFFFFFF) | pix<<24));
    						// replace weighted gaussian distro value with computed disparity val and 3D xform
    						// x is the same axis, the y axis is depth, and z is original Y
    						//Point t = new Point((double)k, (double)m, (double)pix, new Color(((imgsrcL[l*width+k] & 0xFFFFFF))));
    						//Vertex v0 = depthToVertex(k, m+l, pix, camWidth, camHeight, FOV);
    						//Point t = new Point(v0, new Color(((imgsrcL[l*width+k] & 0xFFFFFF))));
    						// z depth made negtive because positive is toward viewer
    						//int ks, ms;
    						//if( k <= camWidth/2 )
    						//	ks = (camWidth/2) - k;
    						//else
    						//	ks = k - (camWidth/2);
    						//if( (m+l) <= camHeight/2 )
    						//	ms = (camHeight/2) - (m+l);
    						//else
    						//	ms = (m+l) - (camHeight/2);
    						//Point t = new Point(ks,ms, -pix, new Color(((imgsrcL[l*width+k] & 0xFFFFFF))));
           					//Vertex v1 = transform.transform(t);
           					//Vertex norm = new Vertex(v1.x,v1.y,v1.z);       
           					//double normalLength = Math.sqrt(norm.x * norm.x + norm.y * norm.y + norm.z * norm.z);
           					//norm.x /= normalLength;
           					//norm.y /= normalLength;
           					//norm.z /= normalLength;
           					//double angleCos = Math.abs(norm.z);
           					//int zIndex = (int)v1.y * outWidth + (int)v1.x;
           					//System.out.println(v1.x+","+v1.y+","+v1.z+","+zIndex+" anglecos="+angleCos);
           					
           					/*if( zIndex < zBuffer.length && zIndex >= 0) {
           						synchronized(zBuffer) {
           							if (zBuffer[zIndex] < v1.z) {*/
           								//boolean oob = false;
           								//synchronized(bimage) {
           									//try {
           										//bimage.setRGB((int)v1.x, (int)v1.y, getShade(t.color, angleCos).getRGB());
           										//bimage.setRGB((int)v1.x, (int)v1.y, t.color.getRGB());
           									//} catch(ArrayIndexOutOfBoundsException aioob) {
           									//	oob = true;
           									//}
           								//}
           								//if( oob )
           								//	System.out.println("Coord out of bounds:"+v1+" point:"+t+" orig:"+k+","+(m+l)+","+pix);
           								/*zBuffer[zIndex] = v1.z;
           							} 
           						}
           						
       						} else {
       							System.out.println("Zindex out of bounds="+zIndex+" "+v1+" point:"+t);
       						}
       						*/
       						
           					//++m;
    						// replace weighted gaussian distro value with computed disparity val in alpha channel
    						//imgsrcL[l*width+k] = ((imgsrcL[l*width+k] & 0xFFFFFF) | pix<<24);
    					//}
    				//}
    				//bimage.setRGB(xsrc, y, (byte)pix);
    				//bimage.setRGB(xsrc, y, ((imageL.getRGB(xsrc,y) & 0xFFFFFF) | pix<<24));
    			//} //++xsrc  move to next x subject in left image at this scanline y
    			//System.out.println("Y scan="+y);
    		//} // next y
    		//if( SAMPLERATE )
    		//	System.out.println("**********END OF 3D IMAGE "+yStart+","+yEnd/*+" close="+close+" far="+far+" ***********"*/);
	}
	/**
	 * Process the stereo images into a new BufferedImage with simulated greyscale representing depth
	 * and uncorrelated patches colored red 
	 * In practice the processing threads will segment the image into several regions to be processed in parallel
	 * differentiated by yStart.
	 * THIS METHOD DESIGNED TO BE USED IN A SINGLE THREAD PER IMAGE BAND, PROCESSING corrWinSize chunks and assigning to yStart scan line.
	 * @param imageL Left image from bus
	 * @param imageR Right image from bus
	 * @param yStart Starting Y band to process
	 * @param width width of image band
	 * @param bimage buffered image to hold result
	 */
	public final void processImageChunk(BufferedImage imageL, BufferedImage imageR, int yStart, int width, BufferedImage bimage) {
			long etime = System.currentTimeMillis();
    		// SAD - sum of absolute differences. take the diff of RGB values of
    		// win 1 and 2 and sum them	
			int[] score = new int[width];
    		int[] imgsrcL = new int[width*corrWinSize]; // left image scan line and kernel window
     		int[] imgsrcR = new int[width*corrWinSize]; // right image scan line and kernel window
     		float[] coeffsL = new float[corrWinSize*corrWinSize];
     		// precompute right side DCT values since we will be sweeping over them repeatedly as we march forward in left X
     		float[][] coeffsR = new float[width][corrWinSize*corrWinSize];
    		// xsrc is index for subject pixel
    		// correlation window assumed symmetric left/right/up/down with odd number of elements
    		// so we use corrWinLeft as top half of window array size and corrwinRight as bottom half
     		int close = 0;
     		int far = 0;
     		int xminL = 0;
     		int xmaxL = 0;
     		int xminR = 0;
     		int xmaxR = 0;
     		int trunc = 0;
     		
 			synchronized(imageL) {
				imageL.getRGB(0, yStart, width, corrWinSize, imgsrcL, 0 , width);
			}
			synchronized(imageR) {
				imageR.getRGB(0, yStart, width, corrWinSize, imgsrcR, 0 , width);
			}
			// Convert to greyscale
			for(int j = 0; j < width*corrWinSize ; j++) {
					int rr = (imgsrcR[j] & 0xFF0000) >> 24;
					int rg = (imgsrcR[j] & 0x00FF00) >> 16;
					int rb = (imgsrcR[j] & 0x0000FF);
					imgsrcR[j] = rr + rg + rb;
					rr = (imgsrcL[j] & 0xFF0000) >> 24;
					rg = (imgsrcL[j] & 0x00FF00) >> 16;
					rb = (imgsrcL[j] & 0x0000FF);
					imgsrcL[j] = rr + rg + rb;
					//int pixR = (int) (((float)rr *.299) + ((float)rg *.587) + ((float)rb *.114));
			}
			//
			// Precompute DCTs in right image.
			// 
			for(int x = 0; x < width; x++) {
	  			xminR = x;
	  			xmaxR = x+corrWinSize;
   				if(  x > width-corrWinSize) {
   					xmaxR = width;
   					xminR = width-corrWinSize;
				}
				// loop sweeps x in right image, we increment by one instead of corrWinSize as we do in outer loop
				// because we need to find every potential match. 
				//System.out.println("X scan="+x+" winsize="+winSize+" xCorrWinL="+xCorrWinL+" xCoorWinR="+xCorrWinR+"@"+Thread.currentThread().getName());
					//
				int coeffsRi = 0;
				//
				// j increments in y for right image
				for(int j = 0; j < corrWinSize; j++) {
					// search at current pixel offset left
					// i increments in x, row major order
					for(int i = xminR; i < xmaxR; i++) {
						int rrgb = imgsrcR[j*width+i];
						//
						//rrgb &= 0xFFFFFF;
						coeffsR[x][coeffsRi++] = rrgb;
						//int rr = (rrgb & 0xFF0000) >> 24;
						//int rg = (rrgb & 0x00FF00) >> 16;
						//int rb = (rrgb & 0x0000FF);
						//int pixR = rr + rg + rb;
						//int pixR = (int) (((float)rr *.299) + ((float)rg *.587) + ((float)rb *.114));
						//coeffsR[x][coeffsRi++] = pixR;
						   // Normalize and gamma correct:
					        //double rfrr = Math.pow(rr / 255.0, 2.2);
					        //double rfgg = Math.pow(rg / 255.0, 2.2);
					        //double rfbb = Math.pow(rb / 255.0, 2.2);
					        // Calculate luminance:
					        //double rflum = 0.2126 * rfrr + 0.7152 * rfgg + 0.0722 * rfbb;
					        // Gamma compand and rescale to byte range:
					        //int pixR = (int) (255.0 * Math.pow(rflum, 1.0 / 2.2));
							// sum of absolute difference of left image weight+RGB and right image weight+RGB
							//sum += Math.abs((pixL /*| (kernel[kx][ky] << 16)*/) - (pixR /*| (kernel[kx][ky] << 16)*/));
					}
				}
				//synchronized(fdct2dR) {
				//	fdct2dR.forward(coeffsR[x], false);
				//}
			}
  			// for each subject pixel X in left image, sweep the scanline for that Y, move to next subject X
			// We always process in corrWinSize chunks
			loop:
			//for(int xsrc = corrWinLeft; xsrc < width-corrWinRight; /*xsrc+=corrWinSize*/xsrc++) {
	  		for(int xsrc = 0; xsrc < width; xsrc++) {	
				// sweep the epipolar scan line for each subject pixel in left image
  	  			xminL = xsrc;
	  			xmaxL = xsrc+corrWinSize;
   				if(  xsrc > width-corrWinSize) {
   					xmaxL = width;
   					xminL = width-corrWinSize;
				}
				int rank = 0;
				double pix = 0;
			    // process left image data, fill IDCT array row major order.
				// Remember, we have extracted the corrWinSize*width band from image, and now deal with that
				int coeffsLi = 0; // DCT float array counter left
				for(int ly = 0; ly < corrWinSize; ly++) {
					for(int lx = xminL; lx < xmaxL; lx++) {
						int lrgb = imgsrcL[ly*width+lx];
						//lrgb &= 0xFFFFFF;
						coeffsL[coeffsLi++] = lrgb;
						//int lr = (lrgb & 0xFF0000) >> 24;
						//int lg = (lrgb & 0x00FF00) >> 16;
						//int lb = (lrgb & 0x0000FF);
						//System.out.println(lr+" "+lg+" "+lb);
						//int pixL = lr + lg + lb; // greyscale
						//int pixL = (int) (((float)lr *.299) + ((float)lg *.587) + ((float)lb *.114)); // human greyscale
						//coeffsL[coeffsLi++] = pixL;
					}
				}
				// try DCT
				//fdct2dL.inverse(coeffsL, false);
				//synchronized(fdct2dL) {
				//	fdct2dL.forward(coeffsL, false);
				//}
				//
				// Left image window set up, now begin sweep of scanline at y in right image.
				// variable x tracks the right image x position
				// 
				int sum = 0;
    			for(int x = 0; x < width; x++) {
    				// skip the subject
    				//if( x == xsrc ) {
    				// skip the subject and if the right chunk starting at x is devoid of edge, skip as well
    				if( x == xsrc ) {
    					score[x] = Integer.MAX_VALUE;
    					continue;
    				}
    	  			xminR = x;
    	  			xmaxR = x+corrWinSize;
       				if(  x > width-corrWinSize) {
	   					xmaxR = width;
	   					xminR = width-corrWinSize;
					}
					// loop sweeps x in right image, we increment by one
					// because we need to find every potential match. 
					//System.out.println("X scan="+x+" winsize="+winSize+" xCorrWinL="+xCorrWinL+" xCoorWinR="+xCorrWinR+"@"+Thread.currentThread().getName());
					// Compute the sum of absolute difference SAD between the 2 matrixes representing
					// the left subject subset and right target subset
					sum = 0;
					for(int isum = 0; isum < corrWinSize*corrWinSize; isum++) {
						//sum += Math.abs(coeffsL[isum]-coeffsR[isum]);
						sum += Math.abs(coeffsL[isum]-coeffsR[x][isum]);
						// sum of squared diffs
						//sum += Math.pow((coeffsL[isum]-coeffsR[isum]),2);
					}
					// score array is sized to image width and we ignore irrelevant edge elements
					score[x] = sum;
 					// If we have a value thats as small as possible, we can assume the differences are zero and this is our target
					// This is just an optimization to save time computing smallest element
					if( sum == 0) {
						++close;
						// set sadZero to current y value to signal kernel generator to progress to next y
						//sadZero.set(y);
						rank = x;
	        			pix = Bf/(double)Math.abs(xsrc-rank);
		        		//System.out.print(xsrc+","+y+" p="+pix+"|");
		        		if( pix > 255) {
		        			//	System.out.println("CLOSE PIX TRUNCATED FROM:"+xsrc+","+y+" p="+pix);
		        			++trunc;
		        			pix = 255;
		        			// make the band red, indicating its been truncated
		            		//for(int l = 0; l < corrWinSize; l++) {
		            			imgsrcL[/*l*width+*/xsrc] = ((imgsrcL[/*l*width+*/xsrc] & 0x00FFFFFF) | 255<<24);
		            			imgsrcL[/*l*width+*/xsrc] = ((imgsrcL[/*l*width+*/xsrc] & 0xFF000000) | 255<<16);
		            		//}
		        		} else {
		        			//bimage.setRGB(xsrc, y, (byte)pix); //pixel by pixel ersatz greyscale option, not so hot
		        			// bulk pixel scanline setup
		        			//for(int l = 0; l < corrWinSize; l++) {
		        			//for(int l = y-corrWinLeft; l < y+corrWinRight; l++) {
		        				//bimage.setRGB(k, l, ((imageL.getRGB(k,l) & 0xFFFFFF) | pix<<24));
		        				// replace weighted gaussian distro value with computed disparity val in alpha channel
		        				//imgsrcL[l*width+k] = ((imgsrcL[l*width+k] & 0xFFFFFF) | pix<<24);
		        				// try a simulated greyscale
		        				int pixG = (int) (((float)pix *.299) + ((float)pix *.587) + ((float)pix *.114));
		        				imgsrcL[/*l*width+*/xsrc] = ((imgsrcL[/*l*width+*/xsrc] & 0x00FFFFFF) | 127<<24);
		        				imgsrcL[/*l*width+*/xsrc] = ((imgsrcL[/*l*width+*/xsrc] & 0xFF00FFFF) | pixG<<16);
		        				imgsrcL[/*l*width+*/xsrc] = ((imgsrcL[/*l*width+*/xsrc] & 0xFFFF00FF) | pixG<<8);
		        				imgsrcL[/*l*width+*/xsrc] = ((imgsrcL[/*l*width+*/xsrc] & 0xFFFFFF00) | pixG);
		        			//}
		        		}
		        		// move to next x at outer x scanline loop
    					continue loop;
    				} // sum = 0
    			} //++x  move to next x target in right image at this scanline y
    			// move through array of differences we built in above scan loops, determine smallest
       			// now calculate the one closest to zero from sum of differences
    			// rank = 0
    			++far;
    			int drank = Integer.MAX_VALUE;
    			// move through array of differences we built in above scan loops
    			//for(int s = xsrc+1; s < winR; s++) {
     			//for(int s = corrWinLeft; s < winR; s++) {
    			for(int s = 0; s < width; s++) {
    				//System.out.print("Y="+y+" score "+s+"="+score[s]+" ");
     				if( s == xsrc )
     					continue;
    				if (score[s] < drank) {
    					rank = s;
    					drank = score[s];
    				}
    			}
    			//System.out.println();
    			//System.out.println("------------------------");
    			//System.out.println("For x="+rank+" subject="+xsrc+" scanline y="+y+" score="+drank);
    			//System.out.println("------------------------");
    			//calc the disparity and insert into disparity map image
    			pix = Bf/(double)Math.abs(xsrc-rank);
    			//if( pix <=0 || pix >=maxHorizontalSep)
    			//	System.out.print(xsrc+","+y+" p="+pix+"|");
    			if( pix > 255) {
    					//System.out.println("FAR PIX TRUNCATED FROM:"+xsrc+","+y+" p="+pix);
    					++trunc;
    					pix = 255;
    					// make the tile red, indicating its been truncated
        				//for( int k = xsrc-corrWinLeft; k < xsrc+corrWinRight; k++) {
        				//for(int l = 0; l < corrWinSize; l++) {
        					imgsrcL[/*l*width+*/xsrc] = ((imgsrcL[/*l*width+*/xsrc] & 0x00FFFFFF) | 255<<24);
        					imgsrcL[/*l*width+*/xsrc] = ((imgsrcL[/*l*width+*/xsrc] & 0xFF000000) | 255<<16);
        				//}
        				//}
    			} else {
    				// bulk pixel scanline setup
    				//for( int k = xsrc-corrWinLeft; k < xsrc+corrWinRight; k++) {
    					//for(int l = y-corrWinLeft; l < y+corrWinRight; l++) {
    					//for(int l = 0; l < corrWinSize; l++) {
    						//bimage.setRGB(k, l, ((imageL.getRGB(k,l) & 0xFFFFFF) | pix<<24));	
    						// replace weighted gaussian distro value with computed disparity val in alpha channel
    						//imgsrcL[l*width+k] = ((imgsrcL[l*width+k] & 0xFFFFFF) | pix<<24);
    						// try a simulated greyscale
    						int pixG = (int) ((pix *.299) + (pix *.587) + (pix *.114));
    						imgsrcL[/*l*width+*/xsrc] = ((imgsrcL[/*l*width+*/xsrc] & 0x00FFFFFF) | 127<<24);
    						imgsrcL[/*l*width+*/xsrc] = ((imgsrcL[/*l*width+*/xsrc] & 0xFF00FFFF) | pixG<<16);
    						imgsrcL[/*l*width+*/xsrc] = ((imgsrcL[/*l*width+*/xsrc] & 0xFFFF00FF) | pixG<<8);
    						imgsrcL[/*l*width+*/xsrc] = ((imgsrcL[/*l*width+*/xsrc] & 0xFFFFFF00) | pixG);
    					//}
    				//}
    			}
    			// pixel by pixel options
    			//bimage.setRGB(xsrc, y, (byte)pix);
    			//bimage.setRGB(xsrc, y, ((imageL.getRGB(xsrc,y) & 0xFFFFFF) | pix<<24));
    			//System.out.println("Y scan="+y);
    		} // next xsrc increment left subject
   			// set the bimage chunk with processed left image band array
			synchronized(bimage) {
				//bimage.setRGB(0, yStart, width, corrWinSize, imgsrcL, 0 , width);
				bimage.setRGB(0, yStart, width, 1, imgsrcL, 0 , width);
			}
    		if( SAMPLERATE )
    			System.out.println("********IMAGE "+yStart+","+(yStart+corrWinSize)+" close="+close+" far="+far+" trunc="+trunc+" "+(System.currentTimeMillis()-etime)+" ms.*******");
	}
	/**
	 * Process the stereo images into a 3D array of RGBD indexed by [x][y][R,G,B,D]
	 * Optionally convert to greyscale and preprocess with transformation matrix to rotate image.
	 * In practice the processing threads will segment the image into several regions to be processed in parallel
	 * differentiated by yStart. corrWinSize window always guaranteed to fit evenly.
	 * A discrete cosine transform is created for both subwindows and the sum of absolute difference is used
	 * to correlate each chunk along the epipolar plane from left to right row.
	 * @param imageL Left image from bus
	 * @param imageR Right image from bus
	 * @param imageT The transformed image we use as right image
	 * @param yStart Starting Y band to process
	 * @param width width of image band
	 * @param transform The 3D rotation matrix
	 * @param convToGrey flag to perform greyscale conversion first.
	 * @param simage2 3D short array to hold result. [x][y][0]=R, [x][y][1]=G, [x][y][2]=B, [x][y][3]=D
	 */
	public final void processImageChunk(BufferedImage imageL, BufferedImage imageR, BufferedImage imageT, int yStart, int width, Matrix3 transform, boolean convToGrey, double[][][] simage2) {	
		long etime = System.currentTimeMillis();
		// SAD - sum of absolute differences. take the diff of RGB values of
		// win 1 and 2 and sum them, result in score array
		long[] score = new long[width];
		int[] imgsrcL = new int[width*corrWinSize]; // left image scan line and kernel window
 		int[] imgsrcR = new int[width*corrWinSize]; // right image scan line and kernel window
 		// precompute right side DCT values since we will be sweeping over them repeatedly as we march forward in left X

		// correlation window assumed symmetric left/right/up/down with odd number of elements
		// so we use corrWinLeft as top half of window array size and corrwinRight as bottom half
 		int close = 0;
 		int far = 0;
 		int xminL = 0;
 		int xmaxL = 0;
 		int xminR = 0;
 		int xmaxR = 0;
 		
 		//System.out.println("images:"+imageL+" "+imageR+" "+Thread.currentThread().getName());
		synchronized(imageL) {
			imageL.getRGB(0, yStart, width, corrWinSize, imgsrcL, 0 , width);
		}
		synchronized(imageR) {
			imageR.getRGB(0, yStart, width, corrWinSize, imgsrcR, 0 , width);
		}
		//System.out.println("Image bands "+Thread.currentThread().getName());
		// A little experiment, possibly use transform to warp image to epipolar lines
		// should ideally be done on both images. Turn off by passing null instead of 3D transform matrix
		if( transform != null ) {
			synchronized(imageT) {
				//System.out.println(imageT.getColorModel()+" orig="+imageR.getColorModel());
				for(int x = 0; x < width; x++) {
					for(int y = 0; y < corrWinSize; y++) {
						int[] tcoords = scale(x,y+yStart,transform);
						imageT.setRGB(tcoords[0], tcoords[1], imgsrcR[y*width+x]);
					}
				}
			}
			// wait for all processing bands to complete the newly transformed image
			try {
				latchTrans.await();
			} catch (InterruptedException | BrokenBarrierException e) {
				System.out.println("Barrer break, return "+Thread.currentThread().getName());
				return;
			}
			// Replace the right image buffer with the transformed one
			synchronized(imageT) {
				imageT.getRGB(0, yStart, width, corrWinSize, imgsrcR, 0 , width);
			}
		} /*else {	synchronized(imageR) {
			imageR.getRGB(0, yStart, width, corrWinSize, imgsrcR, 0 , width);
			}
		}*/

		//System.out.println("right image precomp "+Thread.currentThread().getName());
		// for each subject pixel X in left image, sweep the scanline for that Y, move to next subject X
		// We always process in corrWinSize chunks
		loop:
		//for(int xsrc = corrWinLeft; xsrc < width-corrWinRight; /*xsrc+=corrWinSize*/xsrc++) {
	  	for(int xsrc = 0; xsrc < width; xsrc++) {	
				// sweep the epipolar scan line for each subject pixel in left image
  	  			xminL = xsrc;
	  			xmaxL = xsrc+corrWinSize;
   				if(  xsrc > width-corrWinSize) {
   					xmaxL = width;
   					xminL = width-corrWinSize;
				}
				int rank = 0;
				double pix = 0;
			    // process left image data, process array row major order.
				// Remember, we have extracted the corrWinSize*width band from image, and now deal with that
				//
				// Left image window set up, now begin sweep of scanline at y in right image.
				// variable x tracks the right image x position
				// 
				long sum = 0;
    			for(int x = 0; x < width; x++) {
    				// skip the subject
    				if( x == xsrc ) {
    					score[x] = Long.MAX_VALUE;
    					continue;
    				}
    	  			xminR = x;
    	  			xmaxR = x+corrWinSize;
       				if(  x > width-corrWinSize) {
	   					xmaxR = width;
	   					xminR = width-corrWinSize;
					}
    				// Compute the sum of absolute difference SAD between the 2 matrixes representing
					// the left subject subset and right target subset
    				sum = 0;
       				for(int ly = 0; ly < corrWinSize; ly++) {
       					int rx = xminR;
    					for(int lx = xminL; lx < xmaxL; lx++) {
    						sum += Math.abs((imgsrcL[ly*width+lx] & 0x00FFFFFF) - (imgsrcR[ly*width+rx] & 0x00FFFFFF));
    						/*
    						int lr = (imgsrcL[ly*width+lx] & 0x00FF0000) >> 16;
    						int lg = (imgsrcL[ly*width+lx] & 0x0000FF00) >> 8;
							int lb = (imgsrcL[ly*width+lx] & 0x000000FF);
							int rr = (imgsrcR[ly*width+rx] & 0x00FF0000) >> 16;
    						int rg = (imgsrcR[ly*width+rx] & 0x0000FF00) >> 8;
							int rb = (imgsrcR[ly*width+rx] & 0x000000FF);
    						sum += (Math.abs(lr - rr) + Math.abs(lg - rg) + Math.abs(lb - rb));
    						*/
    						++rx;
    					}
       				}
					// score array is sized to image width and we ignore irrelevant edge elements
					score[x] = sum;
				} // x
				// now calculate the one closest to zero from sum of differences
				// rank = 0
				++far;
				long tscore = Long.MAX_VALUE;
				// move through array of differences we built in above scan loops
				//for(int s = xsrc+1; s < winR; s++) {
 				//for(int s = corrWinLeft; s < winR; s++) {
				for(int s = 0; s < width; s++) {
					//System.out.print("Y="+y+" score "+s+"="+score[s]+" ");
 					//if( s == xsrc )
 					//	continue;
					if (score[s] < tscore) {
						rank = s;
						tscore = score[s];
					}
				}
				// failsafe
				if( rank == xsrc || tscore < 0 ) {
					System.out.println("***FAIL to rank pixel x="+xsrc+" y="+yStart+" score="+tscore);
					synchronized(simage2) {
						//for(int l = 0; l < corrWinSize; l++) {
							simage2[xsrc][/*l+*/yStart][0] = 255;
							simage2[xsrc][/*l+*/yStart][1] = 0;
							simage2[xsrc][/*l+*/yStart][2] = 0;
							simage2[xsrc][/*l*/+yStart][3] = 0;
					}
					continue;
				}
					
				//System.out.println();
				//System.out.println("------------------------");
				//System.out.println("For x="+rank+" subject="+xsrc+" scanline y="+y+" score="+drank);
				//System.out.println("------------------------");
				//calc the disparity and insert into disparity map
				pix = Bf/(double)Math.abs(xsrc-rank);
				//if( pix >=maxHorizontalSep) {
					//System.out.println("PIX TRUNCATED FROM:"+xsrc+","+y+" p="+pix);
					//pix = maxHorizontalSep;
				//} /*else
					//System.out.println("x="+xsrc+", y="+yStart+" score="+drank);*/
				synchronized(simage2) {
						//for(int l = 0; l < corrWinSize; l++) {
							simage2[xsrc][/*l+*/yStart][0] = ((imgsrcL[/*l*width+*/xsrc] & 0x00FF0000) >> 16 );
							simage2[xsrc][/*l+*/yStart][1] = ((imgsrcL[/*l*width+*/xsrc] & 0x0000FF00) >> 8);
							simage2[xsrc][/*l+*/yStart][2] = ((imgsrcL[/*l*width+*/xsrc] & 0x000000FF) );
							simage2[xsrc][/*l*/+yStart][3] = pix;
						//}
					//}
    					//for(int l = y; l < y+corrWinSize; l++) {
   						//for(int l = yStart; l < yStart+corrWinSize; l++) {
    					//	simage[xsrc][l][0] = (short) ((imageLx2.getRGB(xsrc,l) & 0x00FF0000) >> 16 );
    					//	simage[xsrc][l][1] = (short) ((imageLx2.getRGB(xsrc,l) & 0x0000FF00) >> 8);
    					//	simage[xsrc][l][2] = (short) ((imageLx2.getRGB(xsrc,l) & 0x000000FF) );
    						//if( simage[xsrc][l][3] == 0 )
    					//		simage[xsrc][l][3] = (short)pix;
    						//else 
    							// average depth from last images
    							//simage[xsrc][l][3] = (short) ((simage[xsrc][1][3]+pix)/2);
    					//}
				}
			} // ++xsrc Let us  move to next x subject in left image at this scanline y
			//System.out.println("Y scan="+y);
		//} // next y
		if( SAMPLERATE )
			System.out.println("*****IMAGE FROM "+yStart+" to "+(yStart+corrWinSize)+" close="+close+" far="+far+" ***** "+Thread.currentThread().getName()+" *** "+(System.currentTimeMillis()-etime)+" ms.***");
	}
	
	
	/**
	 * Process the stereo images into a 3D array of RGBD indexed by [x][y][R,G,B,D] using an edge detected set of images
	 * In practice the processing threads will segment the image into several regions to be processed in parallel
	 * differentiated by yStart and yEnd.
	 * corrWinSize, which is smallest window always guaranteed to fit vertically and horizontally.
	 * A discrete cosine transform is created for both subwindows and the sum of absolute difference is used
	 * to correlate each chunk along the epipolar plane from left to right row. yStart to yEnd should be evenly divisible by corrWinSize.
	 * THIS METHOD IS TO BE USED IN A SINGLE THREAD, PROCESSING A SINGLE MAGE BAND OF corrWinSize
	 * @param imageL Left image from canny
	 * @param imageR Right image from canny
	 * @param imageRx2 
	 * @param imageLx2 
	 * @param yStart Starting Y band to process
	 * @param width width of image band
	 * @param simage2 3D short array to hold result. [x][y][0]=R, [x][y][1]=G, [x][y][2]=B, [x][y][3]=D
	 */
	public final void processImageChunkEdge(int[] imageL, int[] imageR, BufferedImage imageLx2, BufferedImage imageRx2, int yStart, int width, octree_t node) {
			long etime = System.currentTimeMillis();
			boolean convToGrey = false;
    		// SAD - sum of absolute differences. take the diff of RGB values of
    		// win 1 and 2 and sum them, result in score array
			int[] score = new int[width];
    		int[] imgsrcL = new int[width*corrWinSize]; // left image scan line and kernel window
     		int[] imgsrcR = new int[width*corrWinSize]; // right image scan line and kernel window
       		int[] imgsrcLx = new int[width*corrWinSize]; // left image scan line and kernel window
     		int[] imgsrcRx = new int[width*corrWinSize]; // right image scan line and kernel window
     		//float[] coeffsL = new float[corrWinSize*corrWinSize];
     		// precompute right side DCT values since we will be sweeping over them repeatedly as we march forward in left X
     		//float[][] coeffsR = new float[width][corrWinSize*corrWinSize];
     		boolean emptyRight[] = new boolean[width]; // whether chunk was found to be devoid of edges during precompute
    		// xsrc is index for subject pixel
    		// correlation window assumed symmetric left/right/up/down with odd number of elements
    		// so we use corrWinLeft as top half of window array size and corrwinRight as bottom half
     		// this is the target area but the search area is larger if we wont hit edges of image
     		int close = 0;
     		int far = 0;
     		int xminL = 0;
     		int xmaxL = 0;
     		int xminR = 0;
     		int xmaxR = 0;
     		
     		synchronized(imageL) {
     			// gradient magnitude
     			System.arraycopy(Arrays.copyOfRange(imageL, yStart*width, (yStart+corrWinSize)*width), 0, imgsrcL, 0, width*corrWinSize);
     		}
 			synchronized(imageLx2) {
				imageLx2.getRGB(0, yStart, width, corrWinSize, imgsrcLx, 0 , width);
			}
    		synchronized(imageR) {
     			// gradient magnitude
     			System.arraycopy(Arrays.copyOfRange(imageR, yStart*width, (yStart+corrWinSize)*width), 0, imgsrcR, 0, width*corrWinSize);
     		}
			synchronized(imageRx2) {
				imageRx2.getRGB(0, yStart, width, corrWinSize, imgsrcRx, 0 , width);
			}
			// Convert to greyscale if indicated
			if( convToGrey ) {
				for(int j = 0; j < width*corrWinSize ; j++) {
					int rr = (imgsrcR[j] & 0xFF0000) >> 24;
					int rg = (imgsrcR[j] & 0x00FF00) >> 16;
					int rb = (imgsrcR[j] & 0x0000FF);
					imgsrcR[j] = rr + rg + rb;
					rr = (imgsrcL[j] & 0xFF0000) >> 24;
					rg = (imgsrcL[j] & 0x00FF00) >> 16;
					rb = (imgsrcL[j] & 0x0000FF);
					imgsrcL[j] = rr + rg + rb;
					//int pixR = (int) (((float)rr *.299) + ((float)rg *.587) + ((float)rb *.114));
				}
			}
			//
			// Precompute right image.
			// 
			for(int x = 0; x < width; x++) {
	  			xminR = x;
	  			xmaxR = x+corrWinSize;
   				if(  x > width-corrWinSize) {
   					xmaxR = width;
   					xminR = width-corrWinSize;
				}
				// loop sweeps x in right image, we increment by one instead of corrWinSize as we do in outer loop
				// because we need to find every potential match. 
				//System.out.println("X scan="+x+" winsize="+winSize+" xCorrWinL="+xCorrWinL+" xCoorWinR="+xCorrWinR+"@"+Thread.currentThread().getName());
					//
				//int coeffsRi = 0;
				//
				// j increments in y for right image
				emptyRight[x] = true;
				loop0:
				for(int j = 0; j < corrWinSize; j++) {
					// i increments in x, row major order
					for(int i = xminR; i < xmaxR; i++) {
						//int rrgb = imgsrcR[j*width+i];
						//int rb = (rrgb & 0x0000FF);
						//if( rb != 0 )
						if( imgsrcR[j*width+i] != 0) {
							emptyRight[x] = false;
							break loop0;
						}
						//coeffsR[x][coeffsRi++] = rrgb;
						   // Normalize and gamma correct:
					        //double rfrr = Math.pow(rr / 255.0, 2.2);
					        //double rfgg = Math.pow(rg / 255.0, 2.2);
					        //double rfbb = Math.pow(rb / 255.0, 2.2);
					        // Calculate luminance:
					        //double rflum = 0.2126 * rfrr + 0.7152 * rfgg + 0.0722 * rfbb;
					        // Gamma compand and rescale to byte range:
					        //int pixR = (int) (255.0 * Math.pow(rflum, 1.0 / 2.2));
							// sum of absolute difference of left image weight+RGB and right image weight+RGB
							//sum += Math.abs((pixL /*| (kernel[kx][ky] << 16)*/) - (pixR /*| (kernel[kx][ky] << 16)*/));
					}
				}
				// if no edge anywhere in window, move along
				//if( !emptyRight[x] ) {
				//	synchronized(fdct2dR) {
				//		fdct2dR.forward(coeffsR[x], false);
				//	}
				//}
				/*
				for(int i = 0; i < coeffsR[x].length; i++) {
					System.out.print(x+","+i+"="+coeffsR[x][i]+" ");
				}
				System.out.println();
				*/
			}
				
				
    		//for(int y = yStart; y < yEnd-corrWinSize; /*y+=corrWinSize*/y++) {
     		// y controls the starting scanline of the corrWinSize band we are retrieving from images
    	 	//for(int y = yStart; y < yEnd; y+=corrWinSize) {
				//System.out.println("y="+y+" yStart="+yStart+" yEnd="+yEnd+" width="+width+"@"+Thread.currentThread().getName());
    			//synchronized(imageL) {
    			//	imageL.getRGB(0, y, width, corrWinSize, imgsrcL, 0 , width);
    			//}
    			//synchronized(imageR) {
    			//	imageR.getRGB(0, y, width, corrWinSize, imgsrcR, 0 , width);
    			//}
 
				//
    			// for each subject pixel X in left image, sweep the scanline for that Y, move to next subject X
    			// We always process in corrWinSize chunks
    			loop:
    			//for(int xsrc = corrWinLeft; xsrc < width-corrWinRight; /*xsrc+=corrWinSize*/xsrc++) {
    	  		for(int xsrc = 0; xsrc < width; xsrc++) {
    				// If the left image pixel which is the target to receive the depth value is not edge, continue
					if(imgsrcL[xsrc] == 0)
						continue loop;
    				// sweep the epipolar scan line for each subject pixel in left image
      	  			xminL = xsrc;
    	  			xmaxL = xsrc+corrWinSize;
       				if(xsrc > width-corrWinSize) {
	   					xmaxL = width;
	   					xminL = width-corrWinSize;
					}/* else {
						if( xsrc < corrWinSize) {
							xminL = 0; // xminL = xsrc?, xmaxL = xsrc+corrWinSize?
							xmaxL = corrWinSize;
						}
					}*/
					int rank = 0;
					double pix = 0;
					// Remember, we have extracted the corrWinSize*width band from image, and now deal with that
	
				    // process left image data, fill IDCT array row major order.
					//int coeffsLi = 0; // DCT float array counter left
					//loop2:
					//for(int ly = 0; ly < corrWinSize; ly++) {
						//for(int lx = xminL; lx < xmaxL; lx++) {
							//int lrgb = imgsrcL[ly*width+lx];
							//lrgb &= 0xFFFFFF;
							//coeffsL[coeffsLi++] = lrgb;
							//int lb = (lrgb & 0x0000FF);
							//if( lb != 0 )
							//if( imgsrcL[ly*width+lx] != 0 ) {
							//	foundEdge = true;
								//break loop2;
							//}
							//System.out.println(lr+" "+lg+" "+lb);
							//int pixL = (int) (((float)lr *.299) + ((float)lg *.587) + ((float)lb *.114)); // human greyscale
							//coeffsL[coeffsLi++] = lrgb;
						//}
					//}
					// source contains no edges
					//if(!foundEdge)
					//	continue loop;
					// try DCT
					//fdct2dL.inverse(coeffsL, false);
					//synchronized(fdct2dL) {
					//	fdct2dL.forward(coeffsL, false);
					//}
					//
    				// Left image window set up, now begin sweep of scanline at y in right image.
					// variable x tracks the right image x position
					// 
 					int sum = 0;
        			for(int x = 0; x < width; x++) {
        				// skip the subject
        				//if( x == xsrc ) {
        				// skip the subject and if the right chunk starting at x is devoid of edge, skip as well
        				if( x == xsrc || emptyRight[x] ) {
        					score[x] = Integer.MAX_VALUE;
        					continue;
        				}
        	  			xminR = x;
        	  			xmaxR = x+corrWinSize;
           				if(  x > width-corrWinSize) {
    	   					xmaxR = width;
    	   					xminR = width-corrWinSize;
    					}/* else {
    						if( x < corrWinSize) {
    							xminR = 0; // xminL = xsrc?, xmaxL = xsrc+corrWinSize?
    							xmaxR = corrWinSize;
    						}
    					}*/
    					// loop sweeps x in right image, we increment by one instead of corrWinSize as we do in outer loop
    					// because we need to find every potential match. 
    					//System.out.println("X scan="+x+" winsize="+winSize+" xCorrWinL="+xCorrWinL+" xCoorWinR="+xCorrWinR+"@"+Thread.currentThread().getName());
  						//
						//int coeffsRi = 0;
						//
						// j increments in y for right image
						//foundEdge = false;
						//for(int j = 0; j < corrWinSize; j++) {
							// i increments in x, row major order
    						//for(int i = xminR; i < xmaxR; i++) {
    							//int rrgb = imgsrcR[j*width+i];
    							//
    							//rrgb &= 0xFFFFFF;
    							//coeffsR[coeffsRi++] = rrgb;
    							//int rr = (rrgb & 0xFF0000) >> 24;
    							//int rg = (rrgb & 0x00FF00) >> 16;
    							//int rb = (rrgb & 0x0000FF);
    							//if( rb != 0 )
    							//	foundEdge = true;
      							//int pixR = rr + rg + rb;
    							//int pixR = (int) (((float)rr *.299) + ((float)rg *.587) + ((float)rb *.114));
    							//coeffsR[coeffsRi++] = pixR;
    							   // Normalize and gamma correct:
    						        //double rfrr = Math.pow(rr / 255.0, 2.2);
    						        //double rfgg = Math.pow(rg / 255.0, 2.2);
    						        //double rfbb = Math.pow(rb / 255.0, 2.2);
    						        // Calculate luminance:
    						        //double rflum = 0.2126 * rfrr + 0.7152 * rfgg + 0.0722 * rfbb;
    						        // Gamma compand and rescale to byte range:
    						        //int pixR = (int) (255.0 * Math.pow(rflum, 1.0 / 2.2));
    								// sum of absolute difference of left image weight+RGB and right image weight+RGB
    								//sum += Math.abs((pixL /*| (kernel[kx][ky] << 16)*/) - (pixR /*| (kernel[kx][ky] << 16)*/));
    						//}
    					//}
						// if no edge anywhere in window, move along
						//if( !foundEdge ) {
							//score[x] = Integer.MAX_VALUE;
							//continue;
						//}
    					//fdct2R.inverse(coeffsR, false);
						//fdct2dR.forward(coeffsR, false);

    					// Compute the sum of absolute difference SAD between the 2 matrixes representing
    					// the left subject subset and right target subset
    					sum = 0;
						for(int j = 0; j < corrWinSize; j++) {
							// i increments in x, row major order
    						for(int i = 0; i < corrWinSize; i++) {
    							int rdata = imgsrcRx[j*width+(i+xminR)] & 0x00FFFFFF;//imgsrcR[j*width+(i+xminR)];
    							int ldata = imgsrcLx[j*width+(i+xminL)] & 0x00FFFFFF;//imgsrcL[j*width+(i+xminL)];
    							sum += Math.abs(rdata-ldata);
    						}
						}
    					//for(int isum = 0; isum < corrWinSize*corrWinSize; isum++) {
    						//sum += Math.abs(coeffsL[isum]-coeffsR[isum]);
    						//sum += Math.abs(coeffsL[isum]-coeffsR[x][isum]);
    						// sum of squared diffs
    						//sum += Math.pow((coeffsL[isum]-coeffsR[isum]),2);
    					//}
    					/*
    						synchronized(mutex) {
    							System.out.println("*** xsrc="+xsrc+" x="+x+" y="+y);
    						for(int cil = 0; cil < kernWinSize*kernWinSize; cil++) {
    								System.out.printf(cil+"=%.5f ", coeffsL[cil]);
    						}
    						System.out.println("\r\n---");
    						for(int cir = 0; cir < kernWinSize*kernWinSize; cir++) {
    								System.out.printf(cir+"=%.5f ", coeffsR[cir]);
    						}
    						System.out.println("\r\n=====");
    						}
    					*/
    					// score array is sized to image width and we ignore irrelevant edge elements
    					score[x] = sum;
    							
    				} // x
    				// now calculate the one closest to zero from sum of differences
    				// rank = 0
    				++far;
    				int drank = Integer.MAX_VALUE;
    				// move through array of differences we built in above scan loops
    				//for(int s = xsrc+1; s < winR; s++) {
     				//for(int s = corrWinLeft; s < winR; s++) {
    				for(int s = 0; s < width; s++) {
    					//System.out.print("Y="+y+" score "+s+"="+score[s]+" ");
     					if( s == xsrc )
     						continue;
    					if (score[s] < drank) {
    						rank = s;
    						drank = score[s];
    					}
    				}
    				// possibly the entire right scan was blank?
    				if( drank == Integer.MAX_VALUE )
    					continue;
    				//System.out.println();
    				//System.out.println("------------------------");
    				//System.out.println("For x="+rank+" subject="+xsrc+" scanline y="+y+" score="+drank);
    				//System.out.println("------------------------");
    				//calc the disparity and insert into disparity map image
    				pix = Bf/(double)Math.abs(xsrc-rank);
    				if( pix >=maxHorizontalSep) {
    					//System.out.println("PIX TRUNCATED FROM:"+xsrc+","+y+" p="+pix);
    					pix = maxHorizontalSep;
    				} /*else {
    					System.out.println("Score:"+xsrc+","+yStart+" score="+drank+" p="+pix);
    				}*/
    				
    				//synchronized(simage2) {
        			//			simage2[xsrc][yStart/*l*/][0] = ((imageLx2.getRGB(xsrc,yStart/*l*/) & 0x00FF0000) >> 16 );
        			//			simage2[xsrc][yStart/*l*/][1] = ((imageLx2.getRGB(xsrc,yStart/*l*/) & 0x0000FF00) >> 8);
        			//			simage2[xsrc][yStart/*l*/][2] = ((imageLx2.getRGB(xsrc,yStart/*l*/) & 0x000000FF) );
        			//			simage2[xsrc][yStart/*l*/][3] = pix;
    				//}
    				double ks = xsrc - (camWidth/2);
    				double ms = yStart - (camHeight/2);
    				double os = (Bf/2) - pix ;

    				synchronized(node) {
    					octree_t.build(node, (double)ks, (double)ms, os, 
    						((imgsrcLx[xsrc] & 0x00FF0000) >> 16 ), 
    						((imgsrcLx[xsrc] & 0x0000FF00) >> 8), 
    						((imgsrcLx[xsrc] & 0x000000FF) ));
    				}
    			} //++xsrc  move to next x subject in left image at this scanline y
    			//System.out.println("Y scan="+y);
    		//} // next y
    		if( SAMPLERATE )
    			System.out.println("*****IMAGE FROM "+yStart+" to "+(yStart+corrWinSize)+" close="+close+" far="+far+" ***** "+Thread.currentThread().getName()+" *** "+(System.currentTimeMillis()-etime)+" ms.***");
	}
	
	/**
	 * Translate 2 edge detected image integer linear arrays of edge data and their corresponding RGB images
	 * into 2 octrees where the Z is set to 1. Intended to be 1 scan line in multithreaded parallel thread group.
	 * @param imageL Left image result of Canny edge detector
	 * @param imageR Right image Canny array
	 * @param imageLx2 RGB image source left
	 * @param imageRx2 RGB image source right
	 * @param yStart The Y scan line to process, this should be Atomic Integer incrementing per thread assignment
	 * @param width width of images
	 * @param height height of images
	 * @param nodel Left octree root node, filled by this method partially
	 * @param noder Right octree root node, also built and written to by this method
	 */
	public final void imagesToOctrees(int[] imageL, int[] imageR, 
											BufferedImage imageLx2, BufferedImage imageRx2, 
											int yStart, 
											int width, int height,
											octree_t nodel, octree_t noder) {
		long etime = System.currentTimeMillis();
		int[] imgsrcL = new int[width]; // left image scan line 
 		int[] imgsrcR = new int[width]; // right image scan line
   		int[] imgsrcLx = new int[width]; // left image scan line 
 		int[] imgsrcRx = new int[width]; // right image scan line
 		synchronized(imageL) {
 			// gradient magnitude
 			System.arraycopy(Arrays.copyOfRange(imageL, yStart*width, (yStart+1)*width), 0, imgsrcL, 0, width);
 		}
		synchronized(imageLx2) {
			imageLx2.getRGB(0, yStart, width, 1, imgsrcLx, 0, width);
		}
		synchronized(imageR) {
 			// gradient magnitude
 			System.arraycopy(Arrays.copyOfRange(imageR, yStart*width, (yStart+1)*width), 0, imgsrcR, 0, width);
 		}
		synchronized(imageRx2) {
			imageRx2.getRGB(0, yStart, width, 1, imgsrcRx, 0, width);
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
  		for(int xsrc = 0; xsrc < width; xsrc++) {
			// If the left image pixel which is the target to receive the depth value is not edge, continue
			if(imgsrcR[xsrc] == 0)
				continue;
			double ks = xsrc - (width/2);
			double ms = yStart - (height/2);
			double os = 1;//(Bf/2) - imgsrcR[xsrc]; // gradient intensity

			synchronized(noder) {
				octree_t.build(noder, (double)ks, (double)ms, os, 
					((imgsrcRx[xsrc] & 0x00FF0000) >> 16 ), 
					((imgsrcRx[xsrc] & 0x0000FF00) >> 8), 
					((imgsrcRx[xsrc] & 0x000000FF) ));
			}
		} //++xsrc  move to next x subject in left image at this scanline y
			//System.out.println("Y scan="+y);
		//} // next y
		if( SAMPLERATE )
			System.out.println("imagesToOctrees IMAGE SCAN LINE Y="+yStart+" ***** "+Thread.currentThread().getName()+" *** "+(System.currentTimeMillis()-etime)+" ms.***");
	}
	/**
	 * Match the regions at minimal level from left to right image. Weed out the ones beyond yTolerance
	 * and match the vector cross product of the 2 secondary eigenvectors if the difference between the left
	 * node eigenvectors and right is below tolerance. From the candidates below tolerance, select the one
	 * that has the mnimum difference in left an right eigenvectors. Take the distance between centroids as disparity.
	 * @param yStart Starting Y scan line for this thread.
	 * @param camwidth width of image
	 * @param camheight height of image
	 * @param nodelA list of left octree nodes at smallest level, sorted by centroid y
	 * @param noderA list of right octree nodes at smallest level, sorted by centroid y
	 * @param indexDepth2 resulting array filled with IndexDepth instances of matched regions
	 * @param indexunproc2 resulting array of IndexDepth filled with unmatched minimal regions
	 */
	private void matchRegionsAssignDepth(int yStart, 
										int camwidth, int camheight, 
										List<octree_t> nodelA, List<octree_t> noderA, 
										List<IndexDepth> indexDepth2, List<IndexDepth> indexUnproc2) {
		long etime = System.currentTimeMillis();
		octree_t tnodel = new octree_t();
		tnodel.getCentroid().y = (double)(yStart-(camheight/2));
		//ArrayList<octree_t> leftNodes = new ArrayList<octree_t>();
		List<octree_t> leftNodes;
		// get all nodes along this Y axis, if any
		boolean found = false;
		synchronized(nodelA) {
			int i1 = 0;
			int i0 = 0;
			for( octree_t inode : nodelA) {
				if( ((int)inode.getCentroid().y) == (yStart-(camheight/2)) ) {
					if( !found ) { 
						found = true;
						i0 = i1; // found first =, set lower range
					} 
					++i1; // found or not, bump 'to' range	
				} else {
					if( !found ) { // not found, not = , keep searching
						++i1;
					} else {
						break; // was found, now not =
					}
				}
			}
			if(!found) {
				if(DEBUGTEST2)
					System.out.println("matchRegionsAssignDepth no left image nodes at centroid Y="+(yStart-(camheight/2))+" ***** "+Thread.currentThread().getName()+" ***" );
				return;
			}
			leftNodes = nodelA.subList(i0, i1);
		}
		
		// get the right nodes, where the octree cell has same basic number points and plane nornal
		//if( leftNodes.size() == 0) {
		//	if(DEBUGTEST2)
		//		System.out.println("matchRegionsAssignDepth no left image nodes at centroid Y="+(yStart-(camheight/2))+" ***** "+Thread.currentThread().getName()+" ***" );
		//	return;
		//}
		if(DEBUGTEST2)
			System.out.println("matchRegionsAssignDepth "+leftNodes.size()+" left nodes with centroid Y="+(yStart-(camheight/2))+" ***** "+Thread.currentThread().getName()+" ***");
		// cross product of normals zero and same number points for each left node compared to all right nodes
		//synchronized(noderA) {
		//	for( int i = 0; i < leftNodes.size(); i++) {
		//		for(int j = 0; j < noderA.size(); j++) {
		//			if( leftNodes.get(i).getIndexes().size() == noderA.get(j).getIndexes().size() &&
		//				leftNodes.get(i).getNormal1().multiplyVectorial(noderA.get(j).getNormal1()).getLength() < .001) {
		//					rightNodes.add(noderA.get(j));
		//			}
		//		}
		//	}		
		//}
		//if( rightNodes.size() == 0 ) {
		//	System.out.println("processImageChunkTest2 no right image nodes found for left image scan line at "+yStart);
		//	return;
		//}
		//int[] score = null;
		octree_t oscore = null;
		//double minscore = Double.MAX_VALUE;
		for( octree_t inode : leftNodes) {
			int iscore = 0;
			int nscore = 0;
			int yscore = 0;
			int isize = 0;
			double cScore = Double.MAX_VALUE;
			double dScore = Double.MAX_VALUE;
			synchronized(inode) {
			isize = inode.getIndexes().size();
			// check all right nodes against the ones on this scan line
			found = false;
			synchronized(noderA) {
				for(int j = 0; j < noderA.size(); j++) {
					double yDiff =  Math.abs(inode.getCentroid().y-noderA.get(j).getCentroid().y);
					if( yDiff <= yTolerance ) {
						if( !found ) { 
							found = true;
						}
						// found in range
						double cscore = inode.getNormal2().multiplyVectorial(noderA.get(j).getNormal2()).getLength();
						double dscore = inode.getNormal3().multiplyVectorial(noderA.get(j).getNormal3()).getLength();
						if(cscore < .001 && dscore < .001) {
							if(cscore < cScore && dscore < dScore) {
								cScore = cscore;
								dScore = dscore;
								++yscore;
								// Option 0 go with first one, as .001 is a rather small magnitude that might as well be 0, AKA epsilon
								oscore = noderA.get(j);
							}
							++iscore;
						} else
							++nscore;
						//++i1; // found or not, bump 'to' range	
					} else {
						if( !found ) { // not found, not <= , keep searching
							//++i1;
							continue;
						} else {
							break; // was found, now not <=, y will only increase
						}
					}
				} // right node loop
				if(!found) {
					if(DEBUGTEST2)
							System.out.println("matchRegionsAssignDepth no right image nodes in tolerance at centroid Y="+(yStart-(camheight/2))+" ***** "+Thread.currentThread().getName()+" ***" );
					indexUnproc2.add(new IndexDepth(inode.getMiddle(), inode.getSize(), 0));
					continue; // next left node
				}
							// option 2
							// Compute the sum of absolute difference SAD between the 2 matrixes representing
							// the left subject subset and right target subset
							// each point in left and right octree cells. In practice we never get here but as a fallback
							// we can resort to this old standby
							//int sum = 0;
							//score = new int[noderA.get(j).getIndexes().size()];
							//oscore = new octree_t[score.length];
							// all right nodes that qualified
							//iscore = 0;
							//for(int c = 0; c < inode.getIndexes().size(); c++) {
							//	Vector4d lcolor = inode.getRoot().m_colors.get(inode.getIndexes().get(c));
							//	Vector4d rcolor = inode.getRoot().m_colors.get(noderA.get(j).getIndexes().get(c));				
							//	sum += Math.abs( (lcolor.x-rcolor.x) + (lcolor.y-rcolor.y) + (lcolor.z+rcolor.z) );
							//}
							//oscore[iscore] = noderA.get(j);
							//score[iscore++] = sum
	
			} // right node array synch	
			} // inode synch
	
			if( iscore == 0 ) {
				if(DEBUGTEST2)
					System.out.println("matchRegionsAssignDepth no matching right image nodes found for left at Y="+(yStart-(camheight/2)) +" out of "+nscore+" possible ***** "+Thread.currentThread().getName()+" ***");
				indexUnproc2.add(new IndexDepth(inode.getMiddle(), inode.getSize(), 0));
				continue;
			}
			// Option 2 continues..
			// now calculate the one closest to zero from sum of differences
			//int drank = Integer.MAX_VALUE;
			//int rank = 0;
			// move through array of differences we built in above scan loops
			//for(int s = 0; s < iscore; s++) {
			//	if (score[s] < drank) {
			//		rank = s;
			//		drank = score[s];
			//	}
			//}
			double xDiff = Math.abs(inode.getCentroid().x-oscore/*[rank]*/.getCentroid().x);
			double yDiff =  Math.abs(inode.getCentroid().y-oscore/*[rank]*/.getCentroid().y);
			if(DEBUGTEST2)
				if(iscore > 1) {
					System.out.println("matchRegionsAssignDepth WARNING left node Y="+(yStart-(camheight/2))+" got total of "+iscore+" on plane scores,"+nscore+" off plane, "+yscore+" on-plane weedouts, resulting in node "+oscore+" ***** "+Thread.currentThread().getName()+" *** ");
				} else {
						System.out.println("matchRegionsAssignDepth left node Y="+(yStart-(camheight/2))+" got one good score of "+oscore+" disparity="+xDiff+", "+nscore+" off plane ***** "+Thread.currentThread().getName()+" *** ");
				}
			//calc the disparity and insert into collection
			//we will call disparity the distance to centroid of right
			double pix = Bf/Math.hypot(xDiff, yDiff);	
			if( pix >=maxHorizontalSep) {
				if(DEBUGTEST2)
					System.out.println("matchRegionsAssignDepth left node at Y="+(yStart-(camheight/2))+" PIX TRUNCATED FROM:"+pix+" ***** "+Thread.currentThread().getName()+" *** ");
				pix = maxHorizontalSep;
				//if( pix < maxHorizontalSep) {
				// set points in octree cell to this disparity
				//for(int c = 0; c < inode.getIndexes().size(); c++) {
					//Vector4d tpoint = inode.getRoot().m_points.get(inode.getIndexes().get(c));
					//tpoint.z = pix;//(Bf/2) - pix ;
					//tpoint.w = -1; // mark it as having been set
					//synchronized(indexDepth2) {
					//		indexDepth2.add(new IndexDepth(inode.getMiddle(), inode.getSize(), pix));
					//}
				//}
				//if(DEBUGTEST2)
				//	System.out.println("processImageChunkTest2 node left="+inode+" set "+inode.getIndexes().size()+" points to "+pix+" for line "+yStart+" ***** "+Thread.currentThread().getName()+" *** ");
			} //else
			synchronized(indexDepth2) {
					indexDepth2.add(new IndexDepth(inode.getMiddle(), inode.getSize(), pix));
			}
			//	if(DEBUGTEST2)
			//		System.out.println("processImageChunkTest2 node left="+inode+" of "+inode.getIndexes().size()+" points NOT SET "+pix+" out of tolerance for line "+yStart+" ***** "+Thread.currentThread().getName()+" *** ");
			if(DEBUGTEST2)
				System.out.println("matchRegionsAssignDepth left node Y="+(yStart-(camheight/2))+" should set set "+isize+" points to "+pix+" ***** "+Thread.currentThread().getName()+" *** ");
		} // left octree nodes

		if( SAMPLERATE )
			System.out.println("matchRegionAssignDepth IMAGE SCAN LINE Y="+(yStart-(camheight/2)) +" ***** "+Thread.currentThread().getName()+" *** "+(System.currentTimeMillis()-etime)+" ms.***");
	}
	/**
	 * Process the minimal nodes we collected in the last step against the maximal nodes we generated now.
	 * The indexDepth collection has verified minimal octree node middle point and edge size that we use to determine
	 * whether they fall in the maximal envelope. If so find the points in the maximal envelope that fall within the minimal one and
	 * assign the depth of the minimal region to those points. If the point in question falls within no
	 * minimal region, find the edge of the closes minimal region to the point and assign the depth of the minimal region
	 * to that point. This process can be repeated until all the enclosed minimal regions are
	 * all assigned, by clearing the octree, setting the minimum subdivide level one step higher (s_level), and then
	 * calling the subdivide method and getting the new node arraylist via get_nodes.
	 * @param yStart Index into left maximal node array
	 * @param nodelA contains maximal octree nodes
	 * @param indexDepth2 contains envelopes of minimal octree nodes previously computed
	 */
	private void findEnclosedRegionsSetPointDepth(int yStart, List<octree_t> nodelA, List<IndexDepth> indexDepth2) {
		octree_t inode = null;
		double tdepth = -1;
		Vector4d iMiddle = null;
		double iSize;
		// get first available octree node and process it for this thread
		inode = nodelA.get(yStart);
		synchronized(inode) {
			iMiddle = inode.getMiddle();
			iSize = inode.getSize();
		}
		
		ArrayList<IndexDepth> enclosed = new ArrayList<IndexDepth>();
		
		// find all minimal nodes with an envelope inside this node from previous step
		synchronized(indexDepth2) {
			for(int j = 0; j < indexDepth2.size(); j++) {
				IndexDepth d = indexDepth2.get(j);
				if(d.enclosedBy(iMiddle, iSize)) {
					//indexDepth.remove(j);
					enclosed.add(d); // save for later point assignment
						//if( tdepth == -1 )
						//	tdepth = d.depth;
						//else
						//	tdepth = (tdepth+d.depth)/2; // maintain average
				}
			}
			for(IndexDepth r : enclosed)
				indexDepth2.remove(r);
		}
		if(enclosed.isEmpty()) {
			if(DEBUGTEST3)
				System.out.println("findEnclosedRegionsSetPointDepth node="+yStart+" middle="+iMiddle+" size="+iSize+" encloses 0 nodes of "+indexDepth2.size()+" ***** "+Thread.currentThread().getName()+" *** ");
			return;
		}
		// done with indexDepth, process enclosed array for each point, assign depth
		synchronized(inode) {
			synchronized(inode.getRoot().m_points) {
				// for each point in this maximal node
				for(int c = 0; c < inode.getIndexes().size(); c++) {
					double cdist = Double.MAX_VALUE;
					Vector4d tpoint = inode.getRoot().m_points.get(inode.getIndexes().get(c));
					boolean notAssigned = true;
					// for each minimal node that was enclosed by maximal, does target point lie within?
					for( IndexDepth d : enclosed ) {
						//System.out.println("processImageChunkTest3 node="+inode+" encloses "+d+" ***** "+Thread.currentThread().getName()+" *** ");
						if(d.encloses(tpoint)) {
							notAssigned = false;
							tpoint.z = d.depth;
							break;
						} else {
							// see if this point is closer to a new envelope, then use that depth
							// we are not looking for a real point to measure to, just the sides of the envelope
							double txmin = Math.abs(tpoint.x-d.xmin);
							double txmax = Math.abs(tpoint.x-d.xmax);
							double tymin = Math.abs(tpoint.y-d.ymin);
							double tymax = Math.abs(tpoint.y-d.ymax);
							txmin = Math.min(txmin,  txmax);
							tymin = Math.min(tymin, tymax);
							txmin = Math.min(txmin,  tymin);
							if( txmin < cdist ) {
								cdist = txmin; // distance to edge
								tdepth = d.depth; // depth of cell we assign to target
							}
						}
					}
					// did we assign the target point in maximal node to one of the minimal nodes inside?
					if( notAssigned ) {
						if(DEBUGTEST3)
							System.out.println("findEnclosedRegionsSetPointDepth node "+yStart+"="+inode+" point="+tpoint+" not enclosed by any "+enclosed.size()+" minimal areas, assigning default depth "+tdepth+" ***** "+Thread.currentThread().getName()+" *** ");
						tpoint.z = tdepth;
					} else 
						if(DEBUGTEST3)
							System.out.println("findEnclosedRegionsSetPointDepth node "+yStart+"="+inode+" point="+tpoint+" assigned depth "+tpoint.z+" ***** "+Thread.currentThread().getName()+" *** ");		
				}
				
			}
		}

		if(DEBUGTEST3)
			System.out.println("findEnclosedRegionsSetPointDepth Maximal Envelope "+yStart+" of "+nodelA.size()+" node="+inode+" exits with "+indexDepth2.size()+" minimal envelopes ***** "+Thread.currentThread().getName()+" *** ");
	}
	/**
	 * Transform the coord at x,y by the Matrix3 transform provided, translate and rotate.
	 * @param x
	 * @param y
	 * @param transform
	 * @return the transformed coords
	 */
	private int[] scale(int x, int y, Matrix3 transform) {
		int ks, ms;
		// transform to 3D plane offsets
		if( x <= camWidth/2 )
			ks = (camWidth/2) - x;
		else
			ks = x - (camWidth/2);
		if( y <= camHeight/2 )
				ms = (camHeight/2) - y;
			else
				ms = y - (camHeight/2);
		Point t = new Point(ks, ms, 0, Color.red );
		Vertex v1 = transform.transform(t);
		// transform back to 2D plane
		if( x <= camWidth/2 )
			ks = (camWidth/2) - (int)v1.x;
		else
			ks = (int)v1.x + (camWidth/2);
		if( y <= camHeight/2 )
			ms = (camHeight/2) - (int)v1.y;
		else
			ms = (int)v1.y + (camHeight/2);
		if( ks < 0 || ks >= camWidth || ms < 0 || ms >=camHeight)
			System.out.println("Coords transformed out of range:"+ks+","+ms+" from:"+x+","+y);
		// ks and ms should now be transformed to coord of right image that slides to left
		return new int[]{ks,ms};
		
	}

	
	/**
	 * Plot a course based on visual data
	 * @param simage
	 * @param rangeL
	 * @param rangeR
	 */
	public void navigate(short[][][] simage, int rangeL, int rangeR) {
		int aveL, aveC, aveR, aveLN, aveCN, aveRN;
		//for all the pixels p of the central window {
		//	if D(p) > T {
		//		counter++ 
		//	}
		//	numC++ 
		//}
		// center from rangeL to rangeR
		aveC = 0;
		aveCN = 0;
		for(int i = rangeL; i< rangeR; i+=corrWinSize)	{
			for(int j = corrWinLeft; j < simage[0].length-corrWinRight; j+=corrWinSize) {
				aveC += simage[i][j][3];
				++aveCN;
			}
		}
		aveC /= aveCN;
		// left part from 0 to rangeL
		aveL = 0;
		aveLN = 0;
		for(int i = corrWinLeft; i < rangeL; i+=corrWinSize)	{
			for(int j = corrWinLeft; j < simage[0].length-corrWinRight; j+=corrWinSize) {
				aveL += simage[i][j][3];
				++aveLN;
			}
		}
		//	for all the pixels p of the left window {
		//		sumL =+ D(p)
		//		numL++ 
		//	}
		aveL /= aveLN;
		//	for all the pixels p of the right window {
		//		sumR =+ D(p)
		//		numR++ 
		//	} 
		//}
		// right from rangeR to end
		aveR = 0;
		aveRN = 0;
		for(int i = rangeR; i < simage.length-corrWinRight; i+=corrWinSize)	{
			for(int j = corrWinLeft; j < simage[0].length-corrWinRight; j+=corrWinSize) {
				aveR += simage[i][j][3];
				++aveRN;
			}
		}
		aveR /= aveRN;
	//if counter < r% of numC {
	//	GO STRAIGHT 
	//} else {	
	//avL = sumL / numL
	//avR = sumR / numR
	//if avL <avR {
	//	GO LEFT 
	//} else {
	//GO RIGHT 
	//} 
	//}
		System.out.println("AVE:="+aveL+","+aveC+","+aveR);
		if( aveL < aveR && aveL < aveC )
			System.out.println("<<GO LEFT<<");
		else
			if( aveR < aveC && aveR < aveL)
				System.out.println(">>GO RIGHT>>");
			else
				System.out.println("||GO STRAIGHT||");
	}
	

	class PlayerFrame extends JPanel {
		JFrame frame;
		public PlayerFrame() {
			frame = new JFrame("Player");
			//frame.add(this, BorderLayout.CENTER);
			// Remove window title and borders
	        frame.setUndecorated(true);
	        // Disable Alt+F4 on Windows
	        frame.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
	        // Make frame full-screen
	        //frame.setExtendedState(Frame.MAXIMIZED_BOTH);
		    Container pane = frame.getContentPane();
		    pane.setLayout(new BorderLayout());
		    pane.add(headingSlider, BorderLayout.SOUTH);
		    pane.add(pitchSlider, BorderLayout.EAST);
		    pane.add(this,BorderLayout.CENTER);
	        headingSlider.addChangeListener(new ChangeListener() {
					@Override
					public void stateChanged(ChangeEvent arg0) {
						repaint();
						viewChanged = true;
					}
		        	
		    });
		    //pitchSlider.addChangeListener(e -> renderPanel.repaint());
		    pitchSlider.addChangeListener(new ChangeListener() {
		    		@Override
					public void stateChanged(ChangeEvent arg0) {
						repaint();
						viewChanged = true;
					}
		    });	        
			// remove setsize when setting frame extended state to MAXIMIZED_BOTH
	        // Display frame
			frame.pack();
			frame.setSize(new Dimension(outWidth, outHeight));
			frame.setVisible(true);
		}
		private java.awt.Image lastFrame;
		public void setLastFrame(java.awt.Image lf) {
			if( lf == null ) {
				System.out.println("-----------LAST FRAME NULL ARG SET-------------");
				return;
			}
			if( lastFrame == null ) {
				lastFrame = lf;
				return;
			}
			synchronized(lastFrame) {
				lastFrame = lf; 
			} 
		}
		//public void paint(Graphics g) {
		public void paintComponent(Graphics g) {
			super.paintComponent(g);
			if( lastFrame == null ) {
				System.out.println("-----------LAST FRAME NULL PAINT-------------");
				return;
			}
			synchronized(lastFrame) {
				//g.drawImage(lastFrame, 0, 0, lastFrame.getWidth(this), lastFrame.getHeight(this), this);
				// 3d
				Graphics2D g2 = (Graphics2D)g;
				// move 0.0 origin to center
				g2.translate(lastFrame.getWidth(this)/2,lastFrame.getHeight(this)/2);
				// end 3d
				g2.drawImage(lastFrame, -(lastFrame.getWidth(this)/2),-(lastFrame.getHeight(this)/2), lastFrame.getWidth(this), lastFrame.getHeight(this), this);
				lastFrame.flush();
			}
		}
	}
		
	   public static Color getShade(Color color, double shade) {
	        double redLinear = Math.pow(color.getRed(), 2.4) * shade;
	        double greenLinear = Math.pow(color.getGreen(), 2.4) * shade;
	        double blueLinear = Math.pow(color.getBlue(), 2.4) * shade;

	        int red = (int) Math.pow(redLinear, 1/2.4);
	        int green = (int) Math.pow(greenLinear, 1/2.4);
	        int blue = (int) Math.pow(blueLinear, 1/2.4);
	        if( red < 0 ) {
	        	System.out.println("Red out of range="+red);
	        	red = 0;
	        }
	        if( red > 255) {
	        	System.out.println("Red out of range="+red);
	        	red = 255;
	        }
	        if( green < 0 ) {
	        	System.out.println("Green out of range="+green);
	        	green = 0;
	        }
	        if( green > 255) {
	        	System.out.println("Green out of range="+green);
	        	green = 255;
	        }
	        if( blue < 0 ) {
	        	System.out.println("Blue out of range="+blue);
	        	blue = 0;
	        }
	        if( blue > 255) {
	        	System.out.println("Blue out of range="+blue);
	        	blue = 255;
	        }
	        return new Color(red, green, blue);
	    }
	    /*
	    public static List<Triangle> inflate(List<Triangle> tris) {
	        List<Triangle> result = new ArrayList<>();
	        for (Triangle t : tris) {
	            Vertex m1 = new Vertex((t.v1.x + t.v2.x)/2, (t.v1.y + t.v2.y)/2, (t.v1.z + t.v2.z)/2);
	            Vertex m2 = new Vertex((t.v2.x + t.v3.x)/2, (t.v2.y + t.v3.y)/2, (t.v2.z + t.v3.z)/2);
	            Vertex m3 = new Vertex((t.v1.x + t.v3.x)/2, (t.v1.y + t.v3.y)/2, (t.v1.z + t.v3.z)/2);
	            result.add(new Triangle(t.v1, m1, m3, t.color));
	            result.add(new Triangle(t.v2, m1, m2, t.color));
	            result.add(new Triangle(t.v3, m2, m3, t.color));
	            result.add(new Triangle(m1, m2, m3, t.color));
	        }
	        for (Triangle t : result) {
	            for (Vertex v : new Vertex[] { t.v1, t.v2, t.v3 }) {
	                double l = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z) / Math.sqrt(30000);
	                v.x /= l;
	                v.y /= l;
	                v.z /= l;
	            }
	        }
	        return result;
	    }
	    */
	
	class Vertex {
	    double x;
	    double y;
	    double z;
	    Vertex(double x, double y, double z) {
	        this.x = x;
	        this.y = y;
	        this.z = z;
	    }
		public String toString() {
			return "x="+x+" y="+y+" z="+z;
		}
	}

	class Point extends Vertex {
		Color color;
		Point(double x, double y, double z, Color color) {
			super(x, y, z);
			this.color = color;
		}
		Point(Vertex v, Color color) {
			super(v.x, v.y, v.z);
			this.color = color;
		}
		public String toString() {
			return super.toString()+" color:"+color;
		}
	}

	class Triangle {
	    Vertex v1;
	    Vertex v2;
	    Vertex v3;
	    Color color;
	    Triangle(Vertex v1, Vertex v2, Vertex v3, Color color) {
	        this.v1 = v1;
	        this.v2 = v2;
	        this.v3 = v3;
	        this.color = color;
	    }
	}

	class Matrix3 {
	    double[] values;
	    Matrix3(double[] values) {
	        this.values = values;
	    }
	    Matrix3 multiply(Matrix3 other) {
	        double[] result = new double[9];
	        for (int row = 0; row < 3; row++) {
	            for (int col = 0; col < 3; col++) {
	                for (int i = 0; i < 3; i++) {
	                    result[row * 3 + col] +=
	                        this.values[row * 3 + i] * other.values[i * 3 + col];
	                }
	            }
	        }
	        return new Matrix3(result);
	    }
	    Vertex transform(Vertex in) {
	        return new Vertex(
	            in.x * values[0] + in.y * values[3] + in.z * values[6],
	            in.x * values[1] + in.y * values[4] + in.z * values[7],
	            in.x * values[2] + in.y * values[5] + in.z * values[8]
	        );
	    }
	    public String toString() {
	    	return String.format("[%15.8f,%15.8f,%15.8f]\r\n[%15.8f,%15.8f,%15.8f]\r\n[%15.8f,%15.8f,%15.8f]\r\n",values[0],values[1],values[2],values[3],values[4],values[5],values[6],values[7],values[8]);
	    	
	    }
	}
	   /**
	    * Horizontal and vertial FOV considered the same, in radians.
	    * @param x
	    * @param y
	    * @param depth
	    * @param width
	    * @param height
	    * @param FOV Camera field of view in radians
	    * @return
	    */
		public Vertex depthToVertex(int x, int y, int depth, int width, int height, double FOV) {
			double vx,vy,vz;
		    //calculate x-coordinate
		    double alpha_h = (Math.PI - FOV) / 2;
		    double gamma_i_h = alpha_h + x*(FOV / width);
		    vx = depth / Math.tan(gamma_i_h);
		    //calculate y-coordinate
		    double alpha_v = 2 * Math.PI - FOV / 2;
		    double gamma_i_v = alpha_v + y*(FOV / width);
		    vy = depth * Math.tan(gamma_i_v)*-1;
		    //z-coordinate
		    vz = depth;
		    return new Vertex(vx,vy,vz);
		}
		
		final class IndexDepth {
			public Vector4d middle;
			public double xmin, ymin, xmax, ymax;
			public double depth;
			public IndexDepth(Vector4d middle, double m_size, double depth) {
				this.middle = middle;
				xmin = middle.x-(m_size/2);
				xmax = middle.x+(m_size/2);
				ymin = middle.y-(m_size/2);
				ymax = middle.y+(m_size/2);
				this.depth = depth;
			}
			/**
			 * does the passed square lie within this one edges included?
			 * @param tmiddle
			 * @param tm_size
			 * @return
			 */
			public boolean encloses(Vector4d tmiddle, double tm_size) {
				double txmin = tmiddle.x-(tm_size/2);
				double tymin = tmiddle.y-(tm_size/2);
				double txmax = tmiddle.x+(tm_size/2);
				double tymax = tmiddle.y+(tm_size/2);
				return (xmin <= txmin && xmax >= txmax && ymin <= tymin && ymax >= tymax);
			}
			/**
			 * does the passed point lie within this envelope edges included?
			 * @param tmiddle
			 * @param tm_size
			 * @return
			 */
			public boolean encloses(Vector4d point) {
				return (xmin <= point.x && xmax >= point.x && ymin <= point.y && ymax >= point.y);
			}
			/**
			 * does 'this' lie within the passed square?
			 * @param tmiddle
			 * @param tm_size
			 * @return
			 */
			public boolean enclosedBy(Vector4d tmiddle, double tm_size) {
				double txmin = tmiddle.x-(tm_size/2);
				double tymin = tmiddle.y-(tm_size/2);
				double txmax = tmiddle.x+(tm_size/2);
				double tymax = tmiddle.y+(tm_size/2);
				return (txmin <= middle.x && txmax >= middle.x && tymin <= middle.y && tymax >= middle.y);
			}
			@Override
			public boolean equals(Object tind) {
				return ( xmin == ((IndexDepth)tind).xmin && xmax == ((IndexDepth)tind).xmax &&
						 ymin == ((IndexDepth)tind).ymin && ymax == ((IndexDepth)tind).ymax);
			}
			@Override
			public String toString() {
				return "IndexDepth xmin="+xmin+" ymin="+ymin+" xmax="+xmax+"ymax="+ymax+" depth="+depth;
			}
			
		}

}


