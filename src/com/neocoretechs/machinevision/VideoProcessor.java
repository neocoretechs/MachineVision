package com.neocoretechs.machinevision;

import java.awt.Color;
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
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.SortedMap;
import java.util.TreeMap;
import java.util.concurrent.BrokenBarrierException;
import java.util.concurrent.CyclicBarrier;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicInteger;

import javax.imageio.ImageIO;

import org.jtransforms.dct.DoubleDCT_1D;
import org.jtransforms.dct.FloatDCT_1D;
import org.jtransforms.utils.IOUtils;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;


//import com.neocoretechs.machinevision.CannyEdgeDetector;
import com.neocoretechs.machinevision.MeanColorGenerator;
import com.neocoretechs.machinevision.MedianCutQuantizer;
//import com.neocoretechs.machinevision.ParallelCannyEdgeDetector;
import com.neocoretechs.machinevision.hough3d.Matrix3;
import com.neocoretechs.machinevision.hough3d.Vector4d;
import com.neocoretechs.machinevision.hough3d.hough_settings;
import com.neocoretechs.machinevision.hough3d.octree_t;
import com.neocoretechs.machinevision.hough3d.writer_file;
import com.neocoretechs.robocore.machine.bridge.RadixTree;
import com.neocoretechs.robocore.SynchronizedThreadManager;
import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;
/**
 * Create a disparity map for the left and right images taken from stereo cameras published to bus.
 * We break the image into bands and assign each band to a processing thread. 
 * We have an option to write files that can be read by CloudCompare for testing.
 *  __mode:= option on commandline determines how we process, such as 'edge','display', 'display3D',etc.
 *  
 * The new algorithm for Fast Average Sum of Absolute Differences (FASAD) Matching. This fast algorithm for
 * assigning depth to stereo point clouds works well on low
 * resolution images, requires no calibration of cameras, and is immune to misalignment of the cameras. Instead of brute
 * force region matching, a chromospatial model using averaging and dimensionality reduction allows us to work with
 * groups of points instead of individual pixels. No boundary condition problems.
 * 
 * New algorithm for stereo matching:
 * (The terms region, node, and octree node and octree cell are synonymous)
 * The steps are:
 * 1.) Create the ChromoSpatial model from a pseudo 3D representation of image X,Y,Z coords with the Z axis as 
 * the color value, sans alpha, converted to grayscale. 
 *
 * 2.) Generate 2 octrees of model converted images at the minimal node level, now 6. Execute this step in parallel by assigning
 * the octree build to one entry of Y image scan line range per thread.
 * This subdivide step is in a single thread after barrier synchronization of the build step.
 *
 * 3.) Each node centroid represents the average color in Z of the points in the octree node cell, and 
 * spatial position of that cell in Y, and we are solving for X.
 * Create a radix tree of collections of related X centroids per Z,Y node coordinates. The key is bit interchanged Z,Y and
 * the value in the treemap is the ArrayList of X nodes at that Z,Y.
 *
 * 4.) Process the left image octree regions against the right octree regions by comparing the expanded Z,Y window 
 * of the left node centroid to the retrieved nodes from the radixtree of right nodes. If the retrieved set is empty (no
 * right nodes in the expanded left node window),or all elements of the set have already been processed, put the left node
 * in an 'unprocessed list'.
 * 
 * This is analogous to the standard stereo block matching (SBM) done by brute force sum of absolute differences (SAD) of the RGB
 * values of left and right windows of pixels, but instead using a more evolved approach of comparing the essence 
 * of the contrast of small regions of points in each image.
 *
 * 5.) Assign a disparity to the minimal octree region based on differences in epipolar distance values of the 2 octree 
 * centroids of the points in the regions from step 4. The regions are small enough that 
 * centroid value is accurate enough to give reasonable disparity, comparable to standard block matching.
 * 
 * 6.) Postprocess disparity nodes by throwing out points more than 2 sigma from mean depth. 
 * Multiple 'channels' can be processed simultaneously. A channel is basically another instance of the complete model.
 * For instance if you have the 3 color bands broken out into separate octrees, 3 channels would be used, one for each color
 * band, R, G, B.
 * 
 * Note: A lot of the old commented code revolves around the use of PCA on the octree nodes to find minimal coplanar regions 
 * in both left and right image octrees and generate eigenvectors  and eigenvalues of the principal axis of 
 * the minimal coplanar regions.
 * The octree subdivision scheme relies on hough_settings.min_isotropy to set how much variance is allowed during 
 * PCA of the octree node fields variance2/variance3 when deciding whether to subdivide the octree at each level. 
 * The eigenvalue magnitude of the eigenvectors can be used to form values on which to perform a DCT on the bit interchanged
 * key of the theta, phi of the 3 PCA principal axis (as 16 bit degrees*10 values) as reflected in some of the other old code.
 * The new code tosses that out in favor of a simplified more tolerant matching closer to established SBM stereo block match
 *
 *
 * @author jg (C) NeoCoreTechs 2018,2019
 *
 */
public class VideoProcessor extends AbstractNodeMain 
{
	private static final boolean DEBUG = false;
	private static final boolean DEBUGTEST3 = false;
	private static final boolean DEBUGTEST2 = false;
	private static final boolean DEBUGTEST4 = false;
	private static final boolean SAMPLERATE = false; // display thread timing performance data
	private static final boolean TIMER = false; // display global performance data
	private static final boolean MODELTIMER = false; // display model generation performance data
	private static final boolean QUEUETIMER = false; // display frames queued per second
	private static final boolean WRITEFILES = false; // write full set of display files
	private static final boolean WRITEGRID = false; // write occupancy grid derived from depth points
	private static final boolean WRITEPLANARS = false; // write left and right minimal planars, the processed ones in outplanars# with correlated one color, uncorr another, then left and right channel planarsC#L# and planarsC#R# where C# is channel
	private static final boolean WRITEPLANES = false; // write final planes approximations
	private static final boolean WRITEUNPROCESSED = false; // write unprocessed nodes
	private static final boolean WRITEMODEL = false; // write left and right model, if ASSIGNPOINTS true left file has 3D
	private static final boolean WRITEJPEGS = false; // write left right raw images from stereo bus
	private static final boolean WRITERMS = false; // write an edge file with name RMSxxx (where xxx is rms value) when RMS value changes
	private static final int WRITEMATCHEDPAIRS = 0; // if > 0, the number of individual matched pair files to write from the first number matched to check matching
	private static final boolean ASSIGNPOINTS = false; // assign depth to points in file vs simply compute planars
	private static final boolean SMOOTHEGRID = false; // Bezier smoothing of occupancy nav grid

    private BufferedImage imageLx = null;
    private BufferedImage imageRx = null;
	private int[] dataL = null; // left array return from model generation with Z magnitudes
	private int[] dataR = null; // right model array with magnitudes
	private int[] dataLp = null; // prev left array return from model gen with magnitudes
	private int[] dataRp = null; // prev right canny array with magnitudes

	int outWidth = 640;
	int outHeight = 480;
     
	String mode = "";
	String outDir = "/users/jg/workspace/robocore/motionclouds";
	int frames = 0;
	int channelsx = 0;
	static int files = 0;
	// Number of data channels and octrees we are processing
	final static int CHANNELS = 1;
	// optical parameters
    final static float f = 4.4f; // focal length mm
    final static float B = 100.0f; // baseline mm
    final static float FOVD = 60; // degrees field of view
    final static double FOV = 1.04719755; // radians FOV
    final static int camWidth = 640; // pixels
    final static int camHeight = 480;
    final static int leftBound = -207;
    final static int rightBound = 207;
    final static double fp = B*(camWidth*.5)/Math.tan(FOV *.5 * (Math.PI/2)); //focal length in pixels
    final static double Bf = B*f;// calc the Bf of Bf/d
    final static int maxHorizontalSep = (int) (Bf-1)/4; // max pixel disparity
    final static int yTolerance = 100; // pixel diff in y of potential candidates
    // .0174533 rad = 1 deg
    final static double aTolerance = .007;// minimum angle between normals eigenvector axis 1
    final static double bTolerance = .5;// max difference between eigenvalues, eigenvector axis 1
    final static int cTolerance = 10; // eigenvector eigenvalue axis 2
    final static int dTolerance = 10; // eigenvector eigenvalue axis 3
    public final static int imageIntegrate = 0; // millis to accumulate images and integrate edge detects
    final static int corrSize = 50;
    final static int corrHalfSize = 25;
    final static double DCTRMSSigma = 1.35; // number of standard deviations from DCT RMS mean before we decide its a new image

    final static int queueLength = 5;
    CircularBlockingDeque<Object[]> queueI = new CircularBlockingDeque<Object[]>(queueLength);
	CircularBlockingDeque<stereo_msgs.DisparityImage2> queueO = new CircularBlockingDeque<stereo_msgs.DisparityImage2>(queueLength);
	
	private int sequenceNumber,lastSequenceNumber;
	long time1;

	BufferedImage bimage = null;
	octree_t[] nodel = new octree_t[CHANNELS];
	octree_t[] noder = new octree_t[CHANNELS];
	//List<envInterface> indexDepth; // correlated and matched minimal regions
	float[] depth;// = new float[camWidth*camHeight];
	List<envInterface> indexUnproc; // uncorrelated minimal regions
	List<envInterface> maxEnv; // maximal regions that enclose one or more minimal regions
	List<envInterface> zeroEnc; // maximal regions that enclose zero minimal regions
	//ArrayList<List<int[]>> leftYRange;// = new ArrayList<List<int[]>>(); // from/to position in array for sorted Y centroid processing
	//ArrayList<RadixTree<Integer, AxisNodes>> radixTree;// = new ArrayList<RadixTree<Integer, AxisNodes>>();

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
	double mean, sigma, variance;
	int degen;
	int pairCount;
	//int metric1 = 0; int metric2 = 0; int metric2b; int metric3 = 0; int metric4 = 0; int metric5 = 0;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("subs_videoproc");
	}
	
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		//Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
		System.out.println("Processing "+camWidth+" by "+camHeight);
		setUp();
		// bring up all parallel  processing threads
		spinUp(connectedNode);
		// spin the model generator class and threads
		final PixelsToModel modelGen = new PixelsToModel();
		modelGen.spinGen();
		// subscribe to stereo image bus to start flow of images into queue
		final Subscriber<stereo_msgs.StereoImage> imgsub =
				connectedNode.newSubscriber("/stereo_msgs/StereoImage", stereo_msgs.StereoImage._TYPE);
		/**
		 * Image extraction from bus, then image processing.
		 */
		imgsub.addMessageListener(new MessageListener<stereo_msgs.StereoImage>() {
		@Override
		public void onNewMessage(stereo_msgs.StereoImage img) {
			BufferedImage imageL1 = null;
			BufferedImage imageR1 = null;
			ByteBuffer cbL, cbR;
			byte[] bufferL;
			byte[] bufferR;
			long slew = System.currentTimeMillis() - time1;
			// signal commencement processing by placing the last in queue
			try {
					cbL = img.getData();
					bufferL = cbL.array();
					InputStream in = new ByteArrayInputStream(bufferL);
					imageL1 = ImageIO.read(in);
					in.close();
					cbR = img.getData2();
					bufferR = cbR.array();
					in = new ByteArrayInputStream(bufferR);
					imageR1 = ImageIO.read(in);
					in.close();
					//
					modelGen.imageL1 = imageL1;
					modelGen.imageR1 = imageR1;
					modelGen.latchOutL.reset();
					modelGen.latchOutR.reset();
					modelGen.latchL.reset();
					modelGen.latchR.reset();
					long edgeTime = System.currentTimeMillis();
					try {
						modelGen.latchL.await();
						modelGen.latchR.await();
					} catch (InterruptedException | BrokenBarrierException e) {System.out.println("<<Unexpected InLatch barrier break>>");}
					try {
						modelGen.latchOutL.await();
						modelGen.latchOutR.await();
					} catch (InterruptedException | BrokenBarrierException e) {System.out.println("<<Unexpected OutLatch barrier break>>");}
					if(MODELTIMER)
						System.out.println("Model generator time="+(System.currentTimeMillis()-edgeTime)+" ms.");
	        	    // We have a set amount of time to resolve our view then send it down the line
					if( slew >= imageIntegrate ) {
						//if( queueI.length() == queueLength ) {
						//	Object[] owrite = queueI.peekLast();
						//	((BufferedImage)owrite[0]).flush();
						//	((BufferedImage)owrite[1]).flush();
						//}
						queueI.addLast(new Object[]{imageL1,imageR1,dataLp,dataRp,bufferL});
						dataLp = null;
						dataRp = null;
						time1 = System.currentTimeMillis();
						if(QUEUETIMER)
							System.out.println("Queued framesets:"+(sequenceNumber-lastSequenceNumber)+" INPUT queue size:"+queueI.length()+" OUTPUT queue size:"+queueO.length()+" file index:"+files);
						lastSequenceNumber = sequenceNumber;
					}
				//if( DEBUG ) {
				//	System.out.println("New left/right images "+img.getWidth()+","+img.getHeight()+" size:"+bufferL.length/*ib.limit()*/);
				//}
			} catch (IOException e1) {
				System.out.println("Could not convert image payload due to:"+e1.getMessage());
				return;
			}
			++sequenceNumber; // we want to inc seq regardless to see how many we drop	
		}
	  }); // StereoImage message listener
		
		/**
		 * Publishing loop. Wait for point cloud to appear in queueO, then publish it to the bus.
		 */
		final Publisher<stereo_msgs.DisparityImage2> pubcloud =
				connectedNode.newPublisher("/sensor_msgs/PointCloud", stereo_msgs.DisparityImage2._TYPE);

		connectedNode.executeCancellableLoop(new CancellableLoop() {
			private int sequenceNumber;
			@Override
			protected void setup() {
				sequenceNumber = 0;
			}
			@Override
			protected void loop() throws InterruptedException {
				if( !queueO.isEmpty() ) {
					stereo_msgs.DisparityImage2 pubc = queueO.takeFirst();
					++sequenceNumber;
					if( DEBUG) System.out.println("Published:"+sequenceNumber+" INPUT queue size:"+queueI.length()+" OUTPUT queue size:"+queueO.length()+" file index:"+files);
					pubcloud.publish(pubc);
				} else
					Thread.sleep(1);
			}
		});  
	} // onstart
	
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
		SynchronizedThreadManager.getInstance().init(new String[]{"BUILDOCTREEA","MATCHREGIONA"});
		final int execLimit = camHeight;
		SynchronizedThreadManager.getInstance().spin(new Runnable() {
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
					  Future<?>[] jobs = new Future<?>[execLimit];
					  for(int syStart = 0; syStart < execLimit; syStart++) {
					    	jobs[syStart] = SynchronizedThreadManager.getInstance().submit(new Runnable() {
					    		@Override
					    		public void run() {
					    			// Since we run a continuous loop inside run we have to have a way
					    			// to synchronize everything at the beginning of a new image, then
					    			// await completion of all processing threads and signal a reset to
					    			// resume processing at the start of another new image, hence the two barrier
					    			// synchronization latches
					    			imagesToOctrees(dataL, dataR, imageLx, imageRx, 
										yStart[0].getAndIncrement(), camWidth, camHeight, 
										nodel[0], noder[0]);
					    		} // run
					    	},"BUILDOCTREEA"); // spin
					  }
					  SynchronizedThreadManager.waitForCompletion(jobs);
					  if( TIMER )
						  System.out.println("Process time one="+(System.currentTimeMillis()-etime));
					  latchOut.await();
					  latch2.await();
					  etime = System.currentTimeMillis();

					  //leftYRange.clear();
					  //radixTree.clear();
					  final ArrayList<List<int[]>> leftYRange = new ArrayList<List<int[]>>(); // from/to position in array for sorted Y centroid processing
					  final ArrayList<RadixTree<Integer, AxisNodes>> radixTree = new ArrayList<RadixTree<Integer, AxisNodes>>();
					  //
					  final List<ArrayList<octree_t>> tnodel = Collections.synchronizedList(new ArrayList<ArrayList<octree_t>>());
					  final List<ArrayList<octree_t>> tnoder = Collections.synchronizedList(new ArrayList<ArrayList<octree_t>>());
					  //Arrays.fill(depth, 0);
					  depth = new float[camWidth*camHeight];
					  // build the work partition array
					  for(int channels = 0; channels < CHANNELS; channels++){
						  yStart[channels].set(0);
						  tnodel.add(nodel[channels].get_nodes());
						  tnoder.add(noder[channels].get_nodes());
						  List<int[]> yr = Collections.synchronizedList(new ArrayList<int[]>());
						  leftYRange.add(yr);
						  // Right nodes
						  radixTree.add(new RadixTree<Integer, AxisNodes>(((short)0), ((short)(camHeight/2))));
						  genRadixTree(tnoder.get(channels), radixTree.get(channels));
						  System.out.println("Node list length channel "+channels+" Left="+tnodel.get(channels).size()+", Right="+tnoder.get(channels).size()+" Radix tree right="+radixTree.get(channels).getTreeMap().size());
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
					  //indexDepth = Collections.synchronizedList(new ArrayList<envInterface>());
					  indexUnproc = Collections.synchronizedList(new ArrayList<envInterface>());
					  Future<?>[] jobs2 = new Future<?>[leftYRange.get(0).size()];
					  for(int syStart = 0; syStart < leftYRange.get(0).size(); syStart++) {
							jobs2[syStart] = SynchronizedThreadManager.getInstance().submit(new Runnable() {
								@Override
								public void run() {
									// set the left nodes with depth
									matchRegions(yStart[0].getAndIncrement(), leftYRange.get(0), 
											camWidth, camHeight, tnodel.get(0), radixTree.get(0), 
											depth, indexUnproc);
								} // run									
							},"MATCHREGIONA"); // spin
					    } // for syStart
					  SynchronizedThreadManager.waitForCompletion(jobs2);
					  if( TIMER )
							System.out.println("Process time two="+(System.currentTimeMillis()-etime));
					  latchOut2.await();
					// global barrier break
					} catch (BrokenBarrierException | InterruptedException e) { System.out.println("<<BARRIER BREAK>> "+files);}
					} // while true
			} // run
		}); // spin
					
		/**
		 * Main processing thread for image data. Extract image queue elements from ROS bus and then
		 * notify waiting worker threads to process them.
		 * Wait at synch barrier for completion of all processing threads, then display disparity 
		 */
		SynchronizedThreadManager.getInstance().spin(new Runnable() {			
				public void run() {
					//double meanRMS = 0;
					//double varianceRMS = 0;
					//double sigmaRMS = 0;
					System.out.println("Image queue..");
					/**
					 * Main processing loop, extract images from queue, notify worker threads, then display disparity
					 */
			        while(true) {
			        	if( queueI.isEmpty() ) {
							try {
								Thread.sleep(1);
							} catch (InterruptedException e) {}
			        	} else {
			        		try {
			        			// pull first (oldest) object from input queue
			        			// Its a generic object array if image bytes, BufferedImage left and right, 
			        			// and image byes of compressed JPEG from original raw bus input to 
			        			// pass along in our final payload
			        			Object[] o = queueI.takeFirst();
								imageLx = (BufferedImage) o[0];
				        		imageRx = (BufferedImage) o[1];
				        		dataL = (int[]) o[2];
				        		dataR = (int[]) o[3];
				        		final byte[] imageBytes = (byte[])o[4];
			        		   	// write source images
				        		++files;
				        		if( WRITEFILES || WRITEJPEGS) {
				        			synchronized(imageLx) {
				        				writeFile("sourceL", imageLx, files);
				        			}
				        			synchronized(imageRx) {
				        				writeFile("sourceR", imageRx, files);
				        			}
				        		}
			        			// If we are waiting at either cyclic barrier, the reset will cause it to
			        			// to return to top level barrier
			        			// the latches represent each 'step', or group of parallel processing tasks
			        			// use as many steps as needed, unused latches ignored
			        			//latchOut4.reset();
			        			//latch4.reset();
			        			//latchOut3.reset();
			        			//latch3.reset();
			        			latchOut2.reset();
			        			latch2.reset();
			        			latchOut.reset();
			        			latch.reset();
			        			//
				        	    for(int channels = 0; channels < CHANNELS; channels++) {
				        	    	nodel[channels] = new octree_t();
				        	    	octree_t.buildStart(nodel[channels]);
				        	    	noder[channels] = new octree_t();
				        	    	octree_t.buildStart(noder[channels]);
				        	    }
			        			// metrics
			        			//metric1 = metric2 = metric3 = metric4 = metric5 = 0;
			        	     	// first step end multi thread barrier synch
			        			latch.await();
			        	     	latchOut.await();   	     	
			        	     	for(int channels = 0; channels < CHANNELS; channels++) {
			        	     	  synchronized(nodel[channels]) {
			        	     		octree_t.buildEnd(nodel[channels]);
			        	     		//nodel[channels].subdivide();
			        	     		nodel[channels].subdivideFast(); // no PCA
			        	     		/* need PCA
			        	     		if( WRITEFILES || WRITEPLANARS) {
			        	     			System.out.println("Writing planarsC"+channels+"L"+files);
			        	     			writer_file.writePerp(nodel[channels], "planarsC"+channels+"L"+files);
			        	     		}
			        	     		*/
			        	     	  }
			        	     	}
			        	     	for(int channels = 0; channels < CHANNELS; channels++) {
			        	     	  synchronized(noder[channels]) {
			        	     		octree_t.buildEnd(noder[channels]);
			        	     		//noder[channels].subdivide();
			        	     		noder[channels].subdivideFast(); // no PCA
			        	     		/* need PCA
			        	     		if( WRITEFILES || WRITEPLANARS) {
			        	     			System.out.println("Writing planarsC"+channels+"R"+files);
			        	     			writer_file.writePerp(noder[channels], "planarsC"+channels+"R"+files);
			        	     		}
			        	     		*/
			        	     	  }
			        	     	}
		        	     		if(WRITEFILES || WRITEMODEL) {
		        	     			System.out.println("Writing roscoeR"+files);
		        	     			writeFile(noder,"/roscoeR"+files);
		        	     		}
			        	     	// second step end multi thread barrier synch
			        	     	latch2.await();
			        	     	latchOut2.await();
			        	     	//
			        	     	//imageLx.flush();
			        	     	//imageRx.flush();
			        	     	//
								mean = variance = 0;
								degen = 0;
								//if( indexDepth.size() > 0) {
									//synchronized(indexDepth) {
									  //for(envInterface ind : indexDepth) {
										//  mean += ind.getDepth();
									  //}
								int errCount = 0;
								for(int i = 0; i < camWidth*camHeight; i++) {
									if( depth[i] == 0 )
										++errCount;
									else
										mean += depth[i];
								}
									  //mean /= (double)indexDepth.size();
								int goodDepth = (camWidth*camHeight) - errCount;
								mean /= (double)goodDepth;
								for(int i = 0; i < camWidth*camHeight; i++)
									if( depth[i] > 0 )
										variance += Math.pow((mean - depth[i]),2);
									  //for(envInterface ind : indexDepth) {
									//	  variance += Math.pow((mean - ind.getDepth()),2);
									  //}
									  //variance /= (double)indexDepth.size();
								variance /= (double)goodDepth;
								sigma = Math.sqrt(variance);
								double maxDepth =  mean+(sigma*2);
								for(int i = 0; i < camWidth*camHeight; i++) {
									if( depth[i] > maxDepth) {
										depth[i] = 0;
										++degen;
									}
								}
									  //Iterator<envInterface> ind = indexDepth.iterator();
									 // while(ind.hasNext()) {
										//envInterface ei = ind.next();
										//if( ei.getDepth() > maxDepth ) {
											//ei.setDepth(.2); //degenerate
											//indexUnproc.add(ei);
											//ind.remove();
											//++degen;
										//}
									  //}
								mean = 0;
								errCount = 0;
								for(int i = 0; i < camWidth*camHeight; i++)
									if( depth[i] > 0)
										mean += depth[i];
									else
										++errCount;
								goodDepth = (camWidth*camHeight) - errCount;
								mean /= goodDepth;
									  //for(envInterface inda : indexDepth) {
										//  mean += inda.getDepth();
									  //}
									  //mean /= (double)indexDepth.size();
								//System.out.println("Mean depth = "+mean+" with "+degen+" beyond "+maxDepth+" removed as degenerate.");
									  // write unmatched minimal envelopes
									  /*
									  int empty = 0;
									  int wasvotes = 0;
									  synchronized(indexUnproc) {
				        	     		for(envInterface e: indexUnproc) {
				        	     			if( e.getDepth() == 0 )
				        	     					++empty;
				        	     			else
				        	     				if( e.getDepth() == .1)
				        	     					++wasvotes;
				        	     				else
				        	     					if( e.getDepth() == .2)
				        	     						++degen;
				        	     					else
				        	     						System.out.println("UNKNOWN CASE IN UNPROCESSED NODE LIST:"+e.getDepth());
				        	     		}
				        	     		if( WRITEFILES || WRITEUNPROCESSED) {
					        	     		 System.out.println("Writing emptyset"+files);
					        	     		 writeFile(indexUnproc, "/emptyset"+files, 0);
					        	     		 System.out.println("Writing hadvoted"+files);
					        	     		 writeFile(indexUnproc, "/hadvoted"+files, .1);
					        	     		 System.out.println("Writing degenerate"+files);
					        	     		 writeFile(indexUnproc, "/degenerate"+files, .2); 	     			
				        	     		}     	     		
									  }
									  System.out.println("Run had "+empty+" empty sets and "+wasvotes+
											  " instances where all elements voted and "+degen+" degenerates thrown out post.");
				        	     	  if( WRITEFILES || WRITEPLANES) {
					        	     		 System.out.println("Writing planes"+files);
					        	     		 writeFile(indexDepth, "/planes"+files, -1); // write all processed
				        	     	  }
				        	     	  // PCA only
				        	     	  if( WRITEFILES || WRITEPLANARS) {
				        	     		 System.out.println("Writing outplanars"+files);
				        	     		 writePerp(indexDepth, indexUnproc, 255,255,0, 0,255,255, "outplanars"+files);
				        	     	  }
									  int isize = indexDepth.size();
									  a = genDCT1(indexDepth, isize);
									  */
									  //sensor_msgs.PointCloud pointCloud  = 
										//		connectedNode.getTopicMessageFactory().newFromType(sensor_msgs.PointCloud._TYPE);
									  //genCloud(indexDepth, pointCloud);
									  //queueO.addLast(pointCloud);		        	     			 
									  std_msgs.Header imghead = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);
									  imghead.setSeq(sequenceNumber);
									  org.ros.message.Time tst = connectedNode.getCurrentTime();
									  imghead.setStamp(tst);
									  imghead.setFrameId(tst.toString());
									  stereo_msgs.DisparityImage2 pointCloud  = 
												connectedNode.getTopicMessageFactory().newFromType(stereo_msgs.DisparityImage2._TYPE);
									  sensor_msgs.Image dimage = 
											  connectedNode.getTopicMessageFactory().newFromType(sensor_msgs.Image._TYPE);
									  dimage.setEncoding("JPG"); // 8UC3
									  dimage.setWidth(camWidth);
									  dimage.setHeight(camHeight);
									  dimage.setStep(camWidth);
									  dimage.setIsBigendian((byte)0);
									  dimage.setHeader(imghead);
									  dimage.setData(ByteBuffer.wrap(imageBytes));
									  pointCloud.setHeader(imghead);
									  pointCloud.setImage(dimage);
									  pointCloud.setD(depth);
									  pointCloud.setMaxDisparity((float) maxDepth);
									  pointCloud.setMinDisparity((float) (Bf/camWidth));
									  pointCloud.setF(f); // focal length
									  pointCloud.setT(B); // baseline
									  //genCloud(indexDepth, pointCloud);
									  queueO.addLast(pointCloud);						  
									//} // locking indexDepth processed nodes
								//} // indexDepth > 0
			        	     	// reset level then regenerate tree with maximal coplanar points
			        	     	/*
			        	     	synchronized(nodel) {
			        	     		// set our new maximal level
			        	     		hough_settings.s_level = 4;
			        	     		// set the distance to plane large to not throw out as outliers points we recently assigned a z
			        	     		// this is a divisor so we set to 1
			        	     		hough_settings.max_distance2plane = 1;
			        	     		hough_settings.min_isotropy = .01; // prevent elongated or vertical funky planes
			        	     		nodel.clear();
			        	     		nodel.subdivide();
			        	     	}
			        	     	// third step wait for completion
			        	     	latch3.await();
			        	     	latchOut3.await();
			        	     	// start step 4
			        	     	System.out.println("Unenclosed minimal envelopes="+indexDepth.size()+" zero enclosing maximal envelopes="+zeroEnc.size());
		        	     		// if any unenclosing envelopes, try to assign points using another enclosed maximal area
			        	     	if( !zeroEnc.isEmpty()) {
			        	     		latch4.await();
			        	     		latchOut4.await();
			        	     		Iterator<envInterface> it = zeroEnc.iterator();
			        	     		//for(envInterface e : zeroEnc) {
			        	     		while(it.hasNext()) {
			        	     			envInterface e = (envInterface) it.next();
			        	     			if(e.getDepth() != 0) {
			        	     				maxEnv.add(e);
			        	     				it.remove();
			        	     			}
			        	     		}
			        	     		// write the display cloud with maximal envelopes
			        	     		if(WRITEFILES || WRITEZEROENC)
			        	     				writeFile(zeroEnc, "/lvl5zeroenvL"+files);
			        	     		System.out.println("Final zero enclosing maximal envelopes="+zeroEnc.size());
			        	     		System.out.println("Metrics="+metric1+", "+metric2+", "+metric3+", "+metric4+" ,"+metric5);
			        	     	}
			        	     	// end of parallel processing
			        	     	 */
	
			        	     	// at this point processing of image is essentially complete. If ASSIGNPOINTS is true
			        	     	// the z coords in the point cloud are populated, otherwise we have a list of maximal
			        	     	// planar regions with depth.
								/* only with PCA
			        	     	if(WRITEFILES || WRITEMODEL) {
			        	     		System.out.println("Writing roscoeL"+files);
					        	     synchronized(nodel) {
			        	     			writeFile(nodel,"/roscoeL"+files);
					        	     }
			        	     	}
			        	     	*/
			        	     	//
			        	     	//if(WRITEFILES || WRITEGRID)
			        	     	//	if(!indexDepth.isEmpty()) 
			        	     	//		synchronized(indexDepth) {
					        	//     			genNav2(indexDepth);
			        	     	//		}
			        	     		/*
			        	     		synchronized(maxEnv) {
			        	     			if(WRITEFILES || WRITEPLANES)
			        	     				writeFile(maxEnv, "/lvl5MaxEnv"+files);
			        	     			if(WRITEFILES || WRITEGRID)
			        	     				genNav2(maxEnv);
			        	     			//a = genDCT(maxEnv);
			        	     		}
			        	     		*/
			        	     	//
								//int totalNodes = camWidth*camHeight;;
								//for(int i = 0; i < CHANNELS; i++) totalNodes += nodel[i].get_nodes().size();
			        	     	//System.out.println("Uncorrelated regions="+(totalNodes-indexDepth.size())+" correlated="+indexDepth.size()+", "+(100.0-((float)(totalNodes-indexDepth.size())/(float)totalNodes)*100.0)+"% correlated");
								//System.out.println("Uncorrelated points="+(totalNodes-goodDepth)+" correlated="+goodDepth+", "+(100.0-((float)(totalNodes-goodDepth)/(float)totalNodes)*100.0)+"% correlated");
							} catch (InterruptedException | BrokenBarrierException e) {
								e.printStackTrace();
							}
			        	}
			        } // while true
				} // run      
		}); // spin
	}
	
	
	/**
	 * Translate 2 PixelsToModel integer linear arrays of edge data and their corresponding RGB images
	 * into 2 octrees. Intended to be 1 scan line in multithreaded parallel thread group.
	 * @param imageL Left image result of PixelsToModel
	 * @param imageR Right image from PixelsToModel
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
 			System.arraycopy(Arrays.copyOfRange(imageL, yStart*width, (yStart+1)*width), 0, imgsrcL, 0, width);
 		}
		synchronized(imageLx2) {
			imageLx2.getRGB(0, yStart, width, 1, imgsrcLx, 0, width);
		}
		synchronized(imageR) {
 			System.arraycopy(Arrays.copyOfRange(imageR, yStart*width, (yStart+1)*width), 0, imgsrcR, 0, width);
 		}
		synchronized(imageRx2) {
			imageRx2.getRGB(0, yStart, width, 1, imgsrcRx, 0, width);
		}
	  	for(int xsrc = 0; xsrc < width; xsrc++) {
	  		// If the left image pixel which is the target to receive the depth value is not edge, continue
			//if(imgsrcL[xsrc] == 0)
			//		continue;
			double ks = xsrc - (width/2);
			double ms = yStart - (height/2);
			double os = imgsrcL[xsrc]; // from PixelsToModel if no depth in model set to 1

			synchronized(nodel) {
				octree_t.build(nodel, (double)ks, (double)ms, os, 
					((imgsrcLx[xsrc] & 0x00FF0000) >> 16 ), 
					((imgsrcLx[xsrc] & 0x0000FF00) >> 8), 
					((imgsrcLx[xsrc] & 0x000000FF) ));
			}
		} //++xsrc  move to next x subject in left image at this scanline y
  		for(int xsrc = 0; xsrc < width; xsrc++) {
			// If the left image pixel which is the target to receive the depth value is not valid, continue
			//if(imgsrcR[xsrc] == 0)
			//	continue;
			double ks = xsrc - (width/2);
			double ms = yStart - (height/2);
			double os = imgsrcR[xsrc]; // mean color depth or 1

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
	 * Match the regions at minimal level from left to right image.
	 * Take the distance between centroids as disparity.
	 * Use the radix tree to retrieve a window in the color space model the corresponds to the theta, phi angles of the
	 * 3 eigenvector axis. The 6D window is flattened low and high to retrieve nodes within the 3 angle combined tolerance.
	 * In the Chromospatial model the color value is the Z, so since the radix tree is indexing the orientation of the
	 * coplanar area, the octree is used to select only overlapping nodes based on color model Z value.
	 * @param yStart index into leftYRange2 to indicate our processing range
	 * @param leftYRange2 contains from and to elelments in left nodex txl to process for this thread
	 * @param width
	 * @param height
	 * @param txl List of octree nodes in left image model
	 * @param leftNodes right image nodes arranged in 6D radix tree based on theta/phi of 3 eigencvectors
	 * @param indexDepth2 resultant processed left nodes with depth
	 * @param indexUnproc2 resultant unprocessed left nodes
	 */
	/*
	private void matchRegionsAssignDepth(int yStart, 
										List<int[]> leftYRange2, 
										int width, int height, 
										List<octree_t> txl, RadixTree<BigInteger, octree_t> radixTree2,
										List<envInterface> indexDepth2, List<envInterface> indexUnproc2) {
		long etime = System.currentTimeMillis();
		octree_t inode;
		octree_t oscore = null;
		//List<octree_t> leftNodes;
		//List<octree_t> rightNodes;
		// get all nodes along this Y axis, if any
		// we are just trying to establish a range to sublist
		//synchronized(nodelA) {
		int[] irange = leftYRange2.get(yStart);
			//leftNodes = nodelA.subList(irange[0], irange[1]);
		//}
		// get the right node that matches left as best as possible
		for(int i = irange[0]; i < irange[1]; i++) {
			if(i >= txl.size()) {
				return;
			}
			boolean found = false;
			inode = txl.get(i);
			if(inode == null){ System.out.println("inode null"); return;}
			if( inode.getVariance1() == 0 && inode.getVariance2() == 0 && inode.getVariance3() == 0) {
				indexUnproc2.add(new IndexDepth(inode, 0));
				continue;
			}
			double sum = Double.MAX_VALUE;
			int nscore = 0;
			// the color values as Z have to overlap, there have to be common colors. So the max Z axis have to overlap
			double izMin = Math.floor(inode.getCentroid().z - (inode.getSize()/2));
			double izMax = Math.ceil(inode.getCentroid().z + (inode.getSize()/2));
			//double izMin = Math.floor(inode.getCentroid().x - (inode.getSize()/2));
			//double izMax = Math.ceil(inode.getCentroid().x + (inode.getSize()/2));
			double jzMin = 0;
			double jzMax = 0;
			//int options = (vals[2] << 20) |(vals[3] << 10) | vals[4];
			//SortedMap<Integer, octree_t> noderD = radixTree.subMap(vals[0], vals[1], (short)0xFFFFFFC0, (short)0x3F); // last args is bit mask for search low and high, and and or
			// FFFFFFFFFFFFFFF000000000
			// 000000000000000FFFFFFFFF
			SortedMap<BigInteger, octree_t> noderD = null;//radixTree2.subMap(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], 8);
			// last args is bit mask for search low and high, 'and' and 'or' 6 bits at a time for both tolerances
			// so magnitude 6 is 6 * 6 bits or 36 bits, equivaluent to FFFFFFFFF
			if(noderD == null){ System.out.println("noderD null"); return;}
			if( noderD.isEmpty()) {
				indexUnproc2.add(new IndexDepth(inode, 0));
				continue;
			}
			if(noderD.size() == 1) {
				nscore = 1;
				oscore = noderD.values().iterator().next();
				if(oscore == null){ System.out.println("oscore null"); return;}
				jzMin = Math.floor(oscore.getCentroid().z - (oscore.getSize()/2));
				jzMax = Math.ceil(oscore.getCentroid().z + (oscore.getSize()/2));
				//jzMin = Math.floor(oscore.getCentroid().x - (oscore.getSize()/2));
				//jzMax = Math.ceil(oscore.getCentroid().x + (oscore.getSize()/2));
				//if( (jzMin >= izMin && jzMin <= izMax) || (jzMax >= izMin && jzMax <= izMax) ||
				//	(izMin >= jzMin && izMin <= jzMax) || (izMax >= jzMin && izMax <= jzMax) )
				if( (jzMin < izMin || jzMin > izMax) && (jzMax < izMin || jzMax > izMax) &&
					(izMin < jzMin || izMin > jzMax) && (izMax < jzMin || izMax > jzMax) )
					continue;
			} else {
				for(octree_t jnode : noderD.values()) {
					if(jnode == null){ System.out.println("jnode null"); return;}
					//
					synchronized(jnode) {
						if( jnode.getVotes() == 1 ) {
							continue;
						}
					}
					jzMin = Math.floor(jnode.getCentroid().z - (jnode.getSize()/2));
					jzMax = Math.ceil(jnode.getCentroid().z + (jnode.getSize()/2));
					//jzMin = Math.floor(jnode.getCentroid().x - (jnode.getSize()/2));
					//jzMax = Math.ceil(jnode.getCentroid().x + (jnode.getSize()/2));
					//if( (jzMin >= izMin && jzMin <= izMax) || (jzMax >= izMin && jzMax <= izMax) ||
					//	(izMin >= jzMin && izMin <= jzMax) || (izMax >= jzMin && izMax <= jzMax) )
					if( (jzMin < izMin || jzMin > izMax) && (jzMax < izMin || jzMax > izMax) &&
						(izMin < jzMin || izMin > jzMax) && (izMax < jzMin || izMax > jzMax) )
						continue;
					//Vector4d cross = inode.getNormal1().multiplyVectorial(jnode.getNormal1());
					//double dot = inode.getNormal1().and(jnode.getNormal1()); // dot
					//double crossMag = Math.sqrt(cross.x*cross.x + cross.y*cross.y + cross.z*cross.z);
					//double angle = Math.atan2(crossMag, dot);
					//double vscore1 = Math.abs(inode.getVariance1()-jnode.getVariance1());
					//double vscore2 = Math.abs(inode.getVariance2()-jnode.getVariance2());
					//double vscore3 = Math.abs(inode.getVariance3()-jnode.getVariance3());
					//if(angle > aTolerance ) { //|| vscore1 > bTolerance || vscore2 > cTolerance || vscore3 > dTolerance
					//	continue;
					//}
					++nscore;
					found = true;
					//System.out.println("ANGLE:="+angle+" "+vscore1+" "+vscore2+" "+vscore3+"\r\n"+inode+"\r\n"+jnode);
					// we may have to SAD the coplanar attributes
					//double sadf = genSAD(inode, jnode, Math.min(inode.getIndexes().size(), jnode.getIndexes().size()));
					double sadf = genSAD(inode, jnode);
					// possible for all target children 
					if( sadf > 0 || sadf < sum ) {
						sum = sadf;
						oscore = jnode;
					}
				}
				// either set was empty or only voted upon nodes were encountered
				if(!found) {
					//if(DEBUGTEST2)
					//	System.out.println("matchRegionsAssignDepth left node "+inode+" no right image nodes in tolerance ***** "+Thread.currentThread().getName()+" ***" );
					indexUnproc2.add(new IndexDepth(inode, .1));
					continue;
				}
				if(oscore == null){ System.out.println("oscore  2 null"); return;}
				synchronized(oscore) {
					if(DEBUGTEST2 && nscore == 1 && oscore.getVotes() == 1 ) {
						System.out.println("Only rerieved node has already been matched! "+oscore);
					}
					oscore.setVotes(1);
				}
			}
			octree_t jnode = oscore;
			double xDiff = Math.abs(inode.getCentroid().x-jnode.getCentroid().x);
			//double xDiff = Math.abs(inode.getCentroid().z-jnode.getCentroid().z);
			double yDiff =  Math.abs(inode.getCentroid().y-jnode.getCentroid().y);
			//calc the disparity and insert into collection
			//we will call disparity the distance to centroid of right
			double pix = Bf/Math.hypot(xDiff, yDiff);
			if(DEBUGTEST2) {
				Vector4d cross = inode.getNormal1().multiplyVectorial(jnode.getNormal1());
				double dot = inode.getNormal1().and(jnode.getNormal1()); // dot
				double crossMag = Math.sqrt(cross.x*cross.x + cross.y*cross.y + cross.z*cross.z);
				double angle1 = Math.atan2(crossMag, dot);
				cross = inode.getNormal2().multiplyVectorial(jnode.getNormal2());
				dot = inode.getNormal2().and(jnode.getNormal2()); // dot
				crossMag = Math.sqrt(cross.x*cross.x + cross.y*cross.y + cross.z*cross.z);
				double angle2 = Math.atan2(crossMag, dot);
				cross = inode.getNormal3().multiplyVectorial(jnode.getNormal3());
				dot = inode.getNormal3().and(jnode.getNormal3()); // dot
				crossMag = Math.sqrt(cross.x*cross.x + cross.y*cross.y + cross.z*cross.z);
				double angle3 = Math.atan2(crossMag, dot);
				System.out.println("Found "+nscore+" Angle1="+Math.toDegrees(angle1)+" Angle2="+Math.toDegrees(angle2)+" Angle3="+Math.toDegrees(angle3)+" disp="+pix+" ***** "+Thread.currentThread().getName()+" ***");
			}
			//if( pix >=maxHorizontalSep) {
			//	if(DEBUGTEST2)
			//		System.out.println("matchRegionsAssignDepth left node "+inode+" right node "+jnode+" PIX TRUNCATED FROM:"+pix+" ***** "+Thread.currentThread().getName()+" *** ");
			//	pix = maxHorizontalSep;
			//}
			// insert it into the synchronized list of maximal envelopes
			if( pairCount < WRITEMATCHEDPAIRS && oscore.getCentroid().y > 0) {
				IndexDepth d1 = new IndexDepth(inode, pix);
				IndexDepth d2 = new IndexDepth(jnode, pix);
				writeTestPerp(d1, d2,"match"+(pairCount++));
				indexDepth2.add(d1);
			} else
				indexDepth2.add(new IndexDepth(inode, pix));
		}
	
		// We have a set of left nodes with a common Y value, now extract all right nodes with a Y value in tolerance of
		// our left node set upon which this thread is operating. 
		// Other similar left node sets exist upon which other threads are operating.
		//------------deprecated
		int rpos1 = 0;
		int rpos2 = 0;
		synchronized(radixTree) {		
			octree_t lnode = leftNodes.get(0);
			loop0:
			for(int j = 0; j < radixTree.size(); j++) {
				double yDiff =  Math.abs(lnode.getCentroid().y-radixTree.get(j).getCentroid().y);
				if( yDiff <= yTolerance ) {
					rpos1 = j;
					found = true;
					for(int k = j; k < radixTree.size(); k++) {
						yDiff =  Math.abs(lnode.getCentroid().y-radixTree.get(k).getCentroid().y);
						if(yDiff > yTolerance) {
							rpos2 = k;
							break loop0;
						}
					}
					rpos2 = radixTree.size();
					break;
				}
			}
			if(!found) {
				for( octree_t inode : leftNodes) {
					if(DEBUGTEST2)
						System.out.println("matchRegionsAssignDepth left node "+inode+" no right image nodes in tolerance ***** "+Thread.currentThread().getName()+" ***" );
					indexUnproc2.add(new IndexDepth(inode, 0));
				}
				return;
			}
			rightNodes = radixTree.subList(rpos1, rpos2);
		}
		//--------deprecated

		//-----deprecated
		for( octree_t inode : leftNodes) {
			double sum = Double.MAX_VALUE;
			double csum = Double.MAX_VALUE;
			synchronized(inode) {
				// dont try to match a horizontal or vertical line, no confidence factor
				if( inode.getVariance1() == 0.0 
					//(inode.getNormal1().x == 0 && inode.getNormal1().y == 0) ||
					//(inode.getNormal1().x == 0 && inode.getNormal1().y == 1) ||
					//(inode.getNormal1().x == 1 && inode.getNormal1().y == 0)) {
					if(DEBUGTEST2)
						System.out.println("matchRegionsAssignDepth rejection left node "+inode+" horizontal or vertical ***** "+Thread.currentThread().getName()+" ***");
					indexUnproc2.add(new IndexDepth(inode, 0));
					continue; // next left node
				}
				int isize = inode.getIndexes().size();
				//int zsizeMax = (int)inode.getCentroid().z + (int)Math.ceil(inode.getSize()/2) + zTolerance;
				//int zsizeMin = (int)inode.getCentroid().z - (int)Math.ceil(inode.getSize()/2) - zTolerance;
				// check all right nodes against the ones on this scan line
				found = false;
				int right = 0;
				octree_t oscore = null;
				//
				for(octree_t jnode: rightNodes) {
					synchronized(jnode) {
						int jzsizeMax = (int)jnode.getCentroid().z + (int)Math.ceil(jnode.getSize()/2);
						int jzsizeMin = (int)jnode.getCentroid().z - (int)Math.ceil(jnode.getSize()/2);
						if( jnode.getVotes() != 0 || 
							(jzsizeMin < zsizeMin && jzsizeMax < zsizeMin) || (jzsizeMin > zsizeMin && jzsizeMax > zsizeMax))
							continue;
					}
					// found in range, acos of dot product
					double cscore = Math.acos(inode.getNormal1().and(jnode.getNormal1()));
					//double tcscore = Math.abs(inode.getVariance2()-jnode.getVariance2());
					//double ecscore = Math.abs(inode.getVariance3()-jnode.getVariance3());
					int iscore = (Math.abs(isize-jnode.getIndexes().size())*100)/isize;
					if(cscore <= aTolerance && iscore <= cTolerance) {
						found = true;
						double osum = 0;
						int nsize = jnode.getIndexes().size();
						if(isize < nsize)
							nsize = isize;
						osum = genSAD(inode, jnode, nsize);
						//osum = genDCT2(inode, jnode, nsize);
						if(osum < sum) {
							oscore = jnode;
							sum = osum;
							++right;
						}
					} else {
					    ++nscore; // off plane, beyond tolerance
						if(DEBUGTEST2) // this creates a massive amout of output quickly
							System.out.println("matchRegionsAssignDepth rejection-\r\nleft node :"+inode+"\r\nright node:"+jnode+"\r\nangle dot="+cscore+" point diff="+iscore+" right="+right+" wrong="+nscore+" ***** "+Thread.currentThread().getName()+" ***");
						if(cscore > aTolerance) 
							++metric1;
						else
							//if(tcscore > bTolerance)
							//	++metric2;
							//else
							//	if(ecscore > b2Tolerance)
							//		++metric3;
							//	else
									if(iscore > cTolerance)
										++metric4;
					}
			    } // right node loop
				metric5 += right;
				
				// get left node key
				long lnode[] = genChromoSpatialKeys(inode);
				// see if match
				oscore = radixTree.get(lnode[1]);
				// cant get straightup match, try range
				if( oscore == null) {
					long[] srange = genChromoSpatialKeyRange(lnode[1]);
					SortedMap<Long, octree_t> smap = radixTree.subMap(srange[0], srange[1]);
					//Entry<Long, octree_t> fscore = radixTree.floorEntry(lnode[1]);
					//Entry<Long, octree_t> cscore = radixTree.ceilingEntry(lnode[1]);		
					if(smap == null || smap.isEmpty()) {					
							if(DEBUGTEST2) 
								System.out.println("matchRegionsAssignDepth rejection left node"+inode+" no matching right node out of "+nscore+" ***** "+Thread.currentThread().getName()+" ***");
							indexUnproc2.add(new IndexDepth(inode, 0));
							continue;
					}
					Collection<octree_t> svals = smap.values();
					//System.out.println("Range subset Got "+svals.size());
					Iterator<octree_t> siter = svals.iterator();
					synchronized(radixTree) {
					while(siter.hasNext()) {
						octree_t jnode = siter.next();
						synchronized(jnode) {
							if( jnode.getVotes() == 1)
								continue;
						}
						double cscore = Math.acos(inode.getNormal1().and(jnode.getNormal1()));
						double vscore1 = Math.abs(inode.getVariance1()-jnode.getVariance1());
						double vscore2 = Math.abs(inode.getVariance2()-jnode.getVariance2());
						double vscore3 = Math.abs(inode.getVariance3()-jnode.getVariance3());
						if(cscore > aTolerance || vscore1 > bTolerance || vscore2 > bTolerance || vscore3 > bTolerance) {
							continue;
						}
						// we may have to SAD them
						// yes, SAD
						double sadf = genSAD(inode, jnode, Math.min(inode.getIndexes().size(), jnode.getIndexes().size()));
						if( sadf < sum ) {
							sum = sadf;
							oscore = jnode;
						}
					}
					}
				} else
					System.out.println("DIRECT MATCH:"+inode+" TO \r\n"+oscore+" for key:"+lnode[1]);
				// set right node as matched
				synchronized(oscore) {
					oscore.setVotes(1);
				}
				if(DEBUGTEST2 && right > 1)
					System.out.println("Found "+right+" right nodes for left node "+inode+" out of tolerance nodes="+nscore+" ***** "+Thread.currentThread().getName()+" ***");
				double xDiff = Math.abs(inode.getCentroid().x-oscore.getCentroid().x);
				double yDiff =  Math.abs(inode.getCentroid().y-oscore.getCentroid().y);
				//calc the disparity and insert into collection
				//we will call disparity the distance to centroid of right
				double pix = Bf/Math.hypot(xDiff, yDiff);	
				if( pix >=maxHorizontalSep) {
					if(DEBUGTEST2)
						System.out.println("matchRegionsAssignDepth left node "+inode+" right node "+oscore+" PIX TRUNCATED FROM:"+pix+" ***** "+Thread.currentThread().getName()+" *** ");
					pix = maxHorizontalSep;
				}
				// insert it into the synchronized list of maximal envelopes
				if( pairCount < WRITEMATCHEDPAIRS) {
					IndexDepth d1 = new IndexDepth(inode, pix);
					IndexDepth d2 = new IndexDepth(oscore, pix);
					writeTestPerp(d1, d2,"match"+(pairCount++));
					indexDepth2.add(d1);
				} else
					indexDepth2.add(new IndexDepth(inode, pix));
				//
				if(DEBUGTEST2 && ASSIGNPOINTS)
					System.out.println("matchRegionsAssignDepth left node "+inode+" right node "+oscore+" should set set "+isize+" points to "+pix+" ***** "+Thread.currentThread().getName()+" *** ");
			} // inode synch
		} // left octree nodes
		//----------------deprecated

		if( SAMPLERATE )
			System.out.println("matchRegionAssignDepth ***** "+Thread.currentThread().getName()+" *** "+(System.currentTimeMillis()-etime)+" ms.***");
	}
	*/
	/**
	 * 
	 * @param yStart
	 * @param leftYRange2
	 * @param width
	 * @param height
	 * @param leftNodes
	 * @param radixTree4
	 * @param indexDepth2
	 * @param indexUnproc2
	 */
	private void matchRegions(int yStart, List<int[]> leftYRange2, 
			int width, int height, 
			ArrayList<octree_t> leftNodes, RadixTree<Integer, AxisNodes> radixTree4,
			/*List<envInterface> indexDepth2*/float[] depth, List<envInterface> indexUnproc2) {
		long etime = System.currentTimeMillis();
		double pix = 0;
		int[] irange = leftYRange2.get(yStart);
		//Object[] enr = radixTree4.getTreeMap().entrySet().toArray();
		if(DEBUGTEST2)
			System.out.println("matchRegion from "+irange[0]+","+irange[1]+" ***** "+Thread.currentThread().getName());
		for(int i = irange[0]; i < irange[1]; i++) {
			if(i >= leftNodes.size()) {
				return;
			}
			octree_t inode = leftNodes.get(i);
			//etime = System.currentTimeMillis();
			Map<Integer, AxisNodes> noderD = radixTree4.subMap((short)inode.getCentroid().z, (short)inode.getCentroid().y, 5); // last args is bit mask for search low and high, and and or
			//System.out.println("matchRegion radix tree set "+noderD.size()+" "+(System.currentTimeMillis()-etime)+" ***** "+Thread.currentThread().getName());
			if( noderD.isEmpty()) {
				indexUnproc2.add(new IndexDepth(inode, 0));
				continue;
			}
			double sum = Double.MAX_VALUE;
			octree_t oscore = null;
			int nscore = 0;
			// All the right nodes of all X in the spatial window of color diffence Z as X, and Y
			for(AxisNodes anr :noderD.values()) {
					//AxisNodes anr = ((Entry<Integer, AxisNodes>)enr[k]).getValue();
					//if( inode.getVariance1() == 0 && inode.getVariance2() == 0 && inode.getVariance3() == 0) {
					//	indexUnproc2.add(new IndexDepth(inode, 0));
					//	continue;
					//}
					//short[] vals = genChromoSpatialKeys2(inode);
					// the color values as Z have to overlap, there have to be common colors. So the max Z axis have to overlap
					//double izMin = Math.floor(inode.getCentroid().z - (inode.getSize()/2));
					//double izMax = Math.ceil(inode.getCentroid().z + (inode.getSize()/2));
					//double izMin = Math.floor(inode.getCentroid().x - (inode.getSize()/2));
					//double izMax = Math.ceil(inode.getCentroid().x + (inode.getSize()/2));
					//double jzMin = 0;
					//double jzMax = 0;
					//int options = (vals[2] << 20) |(vals[3] << 10) | vals[4];
					//SortedMap<Integer, octree_t> noderD = radixTree.subMap(vals[0], vals[1], (short)0xFFFFFFC0, (short)0x3F); // last args is bit mask for search low and high, and and or
					// FFFFFFFFFFFFFFF000000000
					// 000000000000000FFFFFFFFF
					//SortedMap<BigInteger, octree_t> noderD = radixTree3.subMap(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], 10);
					// last args is bit mask for search low and high, 'and' and 'or' 6 bits at a time for both tolerances
					// so magnitude 6 is 6 * 6 bits or 36 bits, equivaluent to FFFFFFFFF
					// All the right nodes of THIS individual X from spatial window Z,Y
				for(octree_t jnode : anr.nodes ) {
					synchronized(jnode) {
						if(jnode.getVotes() > 0)
							continue;
					}
					//double rms = IOUtils.computeRMSE(anl.dct, anr.dct, Math.min(anl.dct.length, anr.dct.length));
					// Z nodes represent color difference, we search for minimum color difference
					// and minimum y displacement
					double rms = Math.abs(inode.getCentroid().z - jnode.getCentroid().z) + Math.abs(inode.getCentroid().y - jnode.getCentroid().y);
					if( rms < sum ) {
						sum = rms;
						oscore = jnode;
						++nscore;
					}
				}
			}
			// finished loop examining all right nodes
			if( nscore == 0 ) {
				// All elements have already voted., flag with non zero depth
				indexUnproc2.add(new IndexDepth(inode, .1));
				continue;
			}
			synchronized(oscore) {
				oscore.setVotes(1);
			}
			double xDiff = Math.abs(inode.getCentroid().x-oscore.getCentroid().x);
			if( xDiff < 1 ) { // epipolar constraint says they both have to be at least 1 pixel apart in X
				continue;
			}
			//calc the disparity and insert into collection
			//we will call disparity the distance to centroid of right
			pix = Bf/xDiff;///Math.hypot(xDiff, yDiff);
					// for each point in this maximal node
					for(int c = 0; c < inode.getIndexes().size(); c++) {
						//double cdist = Double.MAX_VALUE;
						Vector4d tpoint = inode.getRoot().m_points.get(inode.getIndexes().get(c));
						int index = (((int)tpoint.y+(height/2))* width) + ((int)tpoint.x + (width/2));
						depth[index] = (float) pix;
					}
			//IndexDepth d1 = new IndexDepth(inode, pix);			
			//indexDepth2.add(d1);
			//if( pairCount < WRITEMATCHEDPAIRS) {
				//IndexDepth d2 = new IndexDepth(oscore, pix);
				//writeTestPerp(d1, d2,"/matched"+(pairCount++));
			//}
			//System.out.println("matchRegions depth="+pix+" candidates="+nscore+" Left node:"+inode+"\r\nRight node:"+jnode+" ***"+Thread.currentThread().getName()+"***");
		}// left range
		
		if( SAMPLERATE )
			System.out.println("matchRegion ***** "+Thread.currentThread().getName()+" *** "+(System.currentTimeMillis()-etime)+" ms.***");
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
	 * @param maxEnv maximal envelopes to which a depth was assigned, a list filled by this method
	 * @param zeroEnc zero enclosing maximal envelopes that did not go to maxEnv, a list filled by this method
	 */
	/*
	private void findEnclosedRegionsSetPointDepth(int yStart, List<octree_t> nodelA, List<envInterface> indexDepth2, List<envInterface> maxEnv, List<envInterface> zeroEnc) {
		octree_t inode = null;
		//double tdepth = -1;
		//Vector4d iMiddle = null;
		//Vector4d iNorm1 = null;
		//Vector4d iNorm2 = null;
		//double iMag1, iMag2;
		IndexMax env;
		double[] denv;
		//double iSize;
		double smean, svariance, ssigma;
		// get first available octree node and process it for this thread
		inode = nodelA.get(yStart);
		synchronized(inode) {
			//iMiddle = inode.getMiddle();
			//iSize = inode.getSize();
			denv = getEnvelope(inode.getCentroid(), inode.getNormal2(), inode.getVariance2(), inode.getNormal3(), inode.getVariance3());
			env = new IndexMax(inode, denv, 0);
		}
		if(DEBUGTEST3)
			System.out.println("findEnclosedRegionsSetPointDepth node="+yStart+" xmin="+denv[0]+" ymin="+denv[1]+" xmax="+denv[2]+" ymax="+denv[3]+" ***** "+Thread.currentThread().getName()+" *** ");
		ArrayList<envInterface> enclosed = new ArrayList<envInterface>();
		
		// find all minimal nodes with an envelope inside this node from previous step
		synchronized(indexDepth2) {
			for(int j = 0; j < indexDepth2.size(); j++) {
				envInterface d = indexDepth2.get(j);
				//if(d.enclosedBy(iMiddle, iSize)) {
				if(d.enclosedBy(env)) {
					//indexDepth.remove(j);
					// if we are more than 2 standard deviations from the overall mean, dont use this
					if( d.getDepth() <= (mean+(sigma*2)) )
						enclosed.add(d); // save for later point assignment
				}
			}
			for(envInterface r : enclosed)
				indexDepth2.remove(r);
		}
		if(enclosed.isEmpty()) {
			if(DEBUGTEST3)
				System.out.println("findEnclosedRegionsSetPointDepth node="+yStart+" encloses 0 nodes of "+indexDepth2.size()+" ***** "+Thread.currentThread().getName()+" *** ");
			zeroEnc.add(env);
			// save to see if we overlap or enclose a smaller maximal region which has a depth assignment
			return;
		}
		smean = svariance = 0;
		for(envInterface ind : enclosed) {
				  smean += ind.getDepth();
		}
		smean /= (double)enclosed.size();
		// give our saved maximal envelope the mean depth for later when we try to assign depth to point in
		// maximal areas that enclosed 0 nodes
		env.setDepth(smean);
		// add this maximal envelope, later try to match those that enclosed 0 with those that got a depth
		maxEnv.add(env);
		//for(IndexDepth ind : enclosed) {
		//		  svariance += Math.pow((smean - ind.depth),2);
		//}
		//svariance /= (double)enclosed.size();
		//ssigma = Math.sqrt(svariance);
		if( DEBUGTEST3 )
			  System.out.println("findEnclosedRegionsSetPointDepth node="+yStart+" Mean="+smean+" ***** "+Thread.currentThread().getName()+" *** "); //+" variance="+svariance+" standard deviation="+ssigma
		// done with indexDepth, process enclosed array for each point, assign depth
		if(ASSIGNPOINTS) {
		synchronized(inode) {
			synchronized(inode.getRoot().m_points) {
				// for each point in this maximal node
				for(int c = 0; c < inode.getIndexes().size(); c++) {
					//double cdist = Double.MAX_VALUE;
					Vector4d tpoint = inode.getRoot().m_points.get(inode.getIndexes().get(c));
					if(DEBUGTEST3)
						if(tpoint.z != 1.0)
							System.out.println("findEnclosedRegionsSetPointDepth node="+yStart+" WARNING point may have been assigned depth already, tpoint="+tpoint+" ***** "+Thread.currentThread().getName()+" *** ");
					boolean notAssigned = true;
					// for each minimal node that was enclosed by maximal, does target point lie within?
					for( envInterface d : enclosed ) {
						//System.out.println("processImageChunkTest3 node="+inode+" encloses "+d+" ***** "+Thread.currentThread().getName()+" *** ");
						if(d.encloses(tpoint)) {
							notAssigned = false;
							tpoint.z = d.getDepth();
							break;
						} //else {
							// see if this point is closer to a new envelope, then use that depth
							// we are not looking for a real point to measure to, just the sides of the envelope
							//double txmin = Math.abs(tpoint.x-d.xmin);
							//double txmax = Math.abs(tpoint.x-d.xmax);
							//double tymin = Math.abs(tpoint.y-d.ymin);
							//double tymax = Math.abs(tpoint.y-d.ymax);
							//txmin = Math.min(txmin,  txmax);
							//tymin = Math.min(tymin, tymax);
							//txmin = Math.min(txmin,  tymin);
							//if( txmin < cdist ) {
							//	cdist = txmin; // distance to edge
							//	tdepth = d.depth; // depth of cell we assign to target
							//}
						//}
					}
					// did we assign the target point in maximal node to one of the minimal nodes inside?
					if( notAssigned ) {
						if(DEBUGTEST3)
							System.out.println("findEnclosedRegionsSetPointDepth node "+yStart+"="+inode+" point="+tpoint+" not enclosed by any "+enclosed.size()+" minimal areas, assigning default depth "+smean+" ***** "+Thread.currentThread().getName()+" *** ");
						tpoint.z = smean;//tdepth;
					} else 
						if(DEBUGTEST3)
							System.out.println("findEnclosedRegionsSetPointDepth node "+yStart+"="+inode+" point="+tpoint+" assigned depth "+tpoint.z+" ***** "+Thread.currentThread().getName()+" *** ");		
				}
				
			}
		}
		}
		if(DEBUGTEST3)
			System.out.println("findEnclosedRegionsSetPointDepth Maximal Envelope "+yStart+" of "+nodelA.size()+" node="+inode+" exits with "+indexDepth2.size()+" minimal envelopes ***** "+Thread.currentThread().getName()+" *** ");
	}
	*/
	/**
	 * Find the maximal regions that enclosed no minimal regions. Try to find another maximal region that overlaps
	 * partially or fully that has a mean depth
	 * we can use to assign depths to points in the zero enclosing region. We are ratcheting up the coplanar areas this way.
	 * @param yStart
	 * @param env Maximal regions that have a mean depth from last step
	 * @param zenc zero enclosing regions whose points need assignment from last step
	 */
	/*
	private void findZeroEnclosedSetPointDepth(int yStart, List<envInterface> env, List<envInterface> zenc) {
		envInterface senv = zenc.get(yStart);
		octree_t inode = ((IndexMax)senv).node;
		double smean = 0;
		ArrayList<envInterface> enclosed = new ArrayList<envInterface>();
		synchronized(env) {
			// try all maximal for this zero encloser, if we find one, use it and bail
			for(envInterface e: env) {
				if( e.enclosedBy(senv) || senv.enclosedBy(e)) {
					if(DEBUGTEST4) {
						if( e.enclosedBy(senv) )
							System.out.println("findZeroEnclosedSetPointDepth node="+yStart+" main envelope "+e+" enclosed by zero "+senv+" ***** "+Thread.currentThread().getName()+" *** ");
						else
							System.out.println("findZeroEnclosedSetPointDepth node="+yStart+" zero envelope "+senv+" enclosed by main "+e+" ***** "+Thread.currentThread().getName()+" *** ");
					}
					smean += e.getDepth();
					enclosed.add(e);
				}
			}
		}
		if( !enclosed.isEmpty()) {
			smean /= (double)enclosed.size();
			//senv.setDepth(e.getDepth());
			if(ASSIGNPOINTS) {
				synchronized(inode) {
					synchronized(inode.getRoot().m_points) {
						// for each point in this maximal node
						for(int c = 0; c < inode.getIndexes().size(); c++) {
							Vector4d tpoint = inode.getRoot().m_points.get(inode.getIndexes().get(c));
							if(tpoint.z != 1.0) {
								if(DEBUGTEST4)
									System.out.println("findZeroEnclosedSetPointDepth node="+yStart+" WARNING point has depth already, point="+tpoint+" ***** "+Thread.currentThread().getName()+" *** ");
							} else {
								// find the envelope this point lie in, if none, give it the mean of all regions
								boolean notAssigned = true;
								// for each minimal node that was enclosed by maximal, does target point lie within?
								for( envInterface d : enclosed ) {
									//System.out.println("processImageChunkTest3 node="+inode+" encloses "+d+" ***** "+Thread.currentThread().getName()+" *** ");
									if(d.encloses(tpoint)) {
										notAssigned = false;
										tpoint.z = d.getDepth();
										if(DEBUGTEST4)
											System.out.println("findZeroEnclosedSetPointDepth node "+yStart+"="+inode+" point="+tpoint+" set from env="+d+" from "+enclosed.size()+" envelopes. ***** "+Thread.currentThread().getName()+" *** ");	
										break;
									} //else {
								}
								if(notAssigned) {
									tpoint.z = smean;//e.getDepth();
									if(DEBUGTEST4)
										System.out.println("findZeroEnclosedSetPointDepth node "+yStart+"="+inode+" point="+tpoint+" set from mean, none found in "+enclosed.size()+" envelopes. ***** "+Thread.currentThread().getName()+" *** ");
								}
							}
						}
					}
				}
			}
			// move the former zero encloser to maximal nodes with mean of all the maximals that enclose it/it encloses
			senv.setDepth(smean);
			return;
		}
		//} synchronized env
		if(DEBUGTEST4)
			System.out.println("findZeroEnclosedSetPointDepth node "+yStart+"="+inode+" found no valid alternate envelope ***** "+Thread.currentThread().getName()+" *** ");	
	}
	*/
	//------------------------------------------
	// support methods
	//------------------------------------------
	/**
	 * Generate the sum of the absolute differences of the octree nodes in the same cells as the ones in
	 * question passed as the 2 parameters. The is done by going to the parent, then choosing each child 
	 * node (of which the passed params will be part) and subtracting the weighted sum
	 * of the differences of the primary normalized eigenvector angles and the eigenvalues of all 3 axis.
	 * The same way we do with pixel color in standard stereo correspondence but here we are using the
	 * properties of the coplanar regions instead of pixel color. Of course pixel color, forming
	 * part of the chromospatial model as Z coordinates, gives us angles between eigenvectors 
	 * and differences in magnitude of eigenvectors as eigenvalues to roll up the whole thing as a more 
	 * comprehensive expression of the image characteristics.
	 * @param inode the left image node
	 * @param jnode the right image node
	 * @return -1 or the weighted sum of absolute differences of the chromospatial components of the children of the octree parent of which the 2 nodes are a part.
	 */
	private static double genDCA(octree_t inode, octree_t jnode) {
		double[] weights = new double[]{10.0, 1.5, .2, .01};
		// apply this theoretical maximum to missing nodes
		//double defVal = (Math.PI*weights[0]*2)+(9999*weights[1])+(9999*weights[2])+(9999*weights[3]);
		if( inode.getParent() == null ) {
			System.out.println("No parent for "+inode);
			return -1;
		}
		if( jnode.getParent() == null) {
			System.out.println("No parent for "+jnode);
			return -1;
		}
		boolean found = false;
		octree_t[] inodes = inode.getParent().getChildren();
		octree_t[] jnodes = jnode.getParent().getChildren();
		int ii = 0;
		for(; ii < 8; ii++) {
			if(inode == inodes[ii]) {
				found = true;
				break;
			}
		}
		if(!found) {
			System.out.println("Cant find inode in children "+inode+" **"+Thread.currentThread().getName()+" ***");
			return -1;
		}
		found = false;
		int jj = 0;
		for(; jj < 8; jj++) {
			if( jnode == jnodes[jj]) {
				found = true;
				break;
			}
		}
		if(!found) {
			System.out.println("Cant find jnode in children "+jnode+" **"+Thread.currentThread().getName()+" ***");
			return -1;
		}
		// i and j have child positions
		ArrayList<octree_t> in = new ArrayList<octree_t>();
		ArrayList<octree_t> jn = new ArrayList<octree_t>();
		in.add(inode);
		jn.add(jnode);
		found = false;
		double res = 0;
		for(int i = 0; i < in.size(); i++) {
			octree_t ixnode = in.get(i);
			octree_t jxnode = jn.get(i);
			if( ixnode.getNormal1() == null || ixnode.getNormal2() == null || ixnode.getNormal3() == null) {
				System.out.println("No child normal "+i+" for inode "+inode+" **"+Thread.currentThread().getName()+" ***");
				continue;
			}
			if( jxnode.getNormal1() == null || jxnode.getNormal2() == null || jxnode.getNormal3() == null) {
				System.out.println("No child normal "+i+" for jnode "+jnode+" **"+Thread.currentThread().getName()+" ***");
				continue;
			}
			found = true;
			Vector4d cross1 = ixnode.getNormal1().multiplyVectorial(jxnode.getNormal1());
			double dot1 = ixnode.getNormal1().and(jxnode.getNormal1()); // dot
			double crossMag1 = Math.sqrt(cross1.x*cross1.x + cross1.y*cross1.y + cross1.z*cross1.z);
			double angle1 = Math.atan2(crossMag1, dot1)*weights[0];
			//
			Vector4d cross2 = ixnode.getNormal2().multiplyVectorial(jxnode.getNormal2());
			double dot2 = ixnode.getNormal2().and(jxnode.getNormal2()); // dot
			double crossMag2 = Math.sqrt(cross2.x*cross2.x + cross2.y*cross2.y + cross2.z*cross2.z);
			double angle2 = Math.atan2(crossMag2, dot2)*weights[0];
			//
			Vector4d cross3 = ixnode.getNormal3().multiplyVectorial(jxnode.getNormal3());
			double dot3 = ixnode.getNormal3().and(jxnode.getNormal3()); // dot
			double crossMag3 = Math.sqrt(cross3.x*cross3.x + cross3.y*cross3.y + cross3.z*cross3.z);
			double angle3 = Math.atan2(crossMag3, dot3)*weights[0];
			//
			double vscore1 = Math.abs(ixnode.getVariance1()-jxnode.getVariance1())*weights[1];
			double vscore2 = Math.abs(ixnode.getVariance2()-jxnode.getVariance2())*weights[2];
			double vscore3 = Math.abs(ixnode.getVariance3()-jxnode.getVariance3())*weights[3];
			//System.out.println("Child ="+i+" ang1="+angle1+" ang2="+angle2+" s1="+vscore1+" s2="+vscore2+" s3="+vscore3+" **"+Thread.currentThread().getName()+" ***");
			res += (angle1+ angle2 + angle3 + vscore1 + vscore2 + vscore3);
		}
		if(!found) {
			System.out.println("No ELEMENTS found for inode:\r\n"+inode+"\r\njnode:"+jnode+" **"+Thread.currentThread().getName()+" ***");
			return -1;
		}
		//System.out.println("RES="+res+" **"+Thread.currentThread().getName()+" ***");
		return res;
	}

	private static short[] genSpatialKey(octree_t node) {
		short x = (short)(node.getCentroid().z);
		short y = (short)(node.getCentroid().y);
		return new short[]{x,y};
	}
	/**
	 * Generate the Radix tree for the set of nodes specified using the chromospatial keys.
	 * We are transforming the theta, phi of the normalized main eigenvector to an x,y space of 3600x3600
	 * (degrees x 10) and creating a peano key to rapidly retrive all nodes in a target window
	 * later on.
	 * @param nodes the octree node set
	 * @param radixTree the Radix tree with chromospatial key hash as key, and octree node as value.
	 */
	private static void genRadixTree(List<octree_t> nodes, RadixTree<Integer, AxisNodes> radixTree2) {
		int vert = 0;
		int i = 0;
		AxisNodes an = null;
		for(octree_t node: nodes) {
			//if(node.getVariance1() != 0) { // PCA
				short[] vals = genSpatialKey(node);
				Integer skey = radixTree2.makeKey(vals[0], vals[1]);
				//skey &= 0xFFFFFFFC; // expands key by each dimension
				an = radixTree2.getTreeMap().get(skey);
				if(an == null) {
					an = new AxisNodes();
					AxisNodes ot = radixTree2.put(skey, an);
					if( ot != null )
						System.out.println("Spatial Key NODE INCONSISTENCY:"+node+" "+vals[0]+","+vals[1]);
					else
						++i;
				}
				an.nodes.add(node);
				//an.eigens.add(bi.doubleValue());
			//} else
				 //++vert;
		}
		//System.out.println("Built "+i+" element radix tree from "+nodes.size()+" nodes with "+vert+" vertical");
		/*
		TreeMap<Integer, AxisNodes> txr = leftNodes.getTreeMap();
		Collection<AxisNodes> si = txr.values();
		Iterator<AxisNodes> it = si.iterator();
		while(it.hasNext()) {
			an = it.next();
			an.dct = new double[an.eigens.size()];
			int j = 0;
			for(Double f: an.eigens) {
				an.dct[j++] = f.doubleValue();
			}
		}
		DoubleDCT_1D fdct1D = new DoubleDCT_1D(an.dct.length);
		fdct1D.forward(an.dct, false);
		*/
	}
	

	/**
	 * Generate ersatz key range.
	 * @param key
	 * @return
	 */
	private static long[] genChromoSpatialKeyRange(long key) {
		long degThetaL = (key & 0xFFFF000000000000L) >> 48;
		long degPhiL =   (key & 0x0000FFFF00000000L) >> 32;
		long var1L =     (key & 0x00000000FFC00000L) >> 20;
		long var2L =     (key & 0x0000000000FFC00L) >> 10;
		long var3L =     (key & 0x000000000000FFCL);
		//System.out.println("key="+key+" theta= "+degThetaL+" phi= "+degPhiL+" var1= "+var1L+" var2= "+var2L+" var3= "+var3L);
		// deg*100 = 0-36099 = 1000110100000011b
		/*
		long degThetaH = degThetaL + 1;
		degThetaL -= 1;
		if(degThetaL < 0)
			degThetaL = 0;
		if( degThetaH > 36000)
			degThetaH = 36000;
		// TODO:if range > 360 or < 0 handle better
		long degPhiH = degPhiL + 1;
		degPhiL -= 1;
		if(degPhiL < 0)
			degPhiL = 0;
		if( degPhiH > 36000)
			degPhiH = 36000;
		// axis of middle variance, perp to normal
		long var1H = var1L + 1;
		var1L -= 1;
		if( var1L < 0)
			var1L = 0;
		if( var1H > 1023)
			var1H = 1023;
		long var2H = var2L + 1;
		var2L -= 1;
		if( var2L < 0)
			var2L = 0;
		if( var2H > 1023)
			var2H = 1023;
		long var3H = var3L + 1;
		var3L -= 1;
		if( var3L < 0)
			var3L = 0;
		if( var3H > 1023)
			var3H = 1023;
		*/
		long degThetaH = degThetaL + 1;
		degThetaL -= 1;
		if(degThetaL < 0)
			degThetaL = 0;
		if( degThetaH > 36099)
			degThetaH = 36099;
		degPhiL = 0;
		long degPhiH = 36000;
		var1L = 0;
		long var1H = 0x33F;
		var2L = 0;
		long var2H = 0x33F;
		var3L = 0;
		long var3H = 0x33F;
		// 0x3FF 10 bits 1023 or 102.3 max variance
		long val1 = ((degThetaL & 0xFFFF) << 48) | ((degPhiL & 0xFFFF) << 32) | 
				   ((var1L & 0x3FF) << 20) | ((var2L & 0x3FF) << 10) | ((var3L & 0x3FF)); 
		long val2 = ((degThetaH & 0xFFFF) << 48) | ((degPhiH & 0xFFFF) << 32) | 
				   ((var1H & 0x3FF) << 20) | ((var2H & 0x3FF) << 10) | ((var3H & 0x3FF)); 
		//System.out.println("keylow="+val1+" keyhigh="+val2);
		return new long[]{val1, val2};
	}
	
	protected void genCloud(List<envInterface> d, sensor_msgs.PointCloud pointCloud2) {
		ArrayList<geometry_msgs.Point32> pnodes = new ArrayList<geometry_msgs.Point32>();
		pointCloud2.setPoints(pnodes);
		sensor_msgs.ChannelFloat32 rgb = new sensor_msgs.ChannelFloat32();
		rgb.setName("RGB");
		List<sensor_msgs.ChannelFloat32> rgbs = new ArrayList<sensor_msgs.ChannelFloat32>();
		rgbs.add(rgb);
		pointCloud2.setChannels(rgbs);
		ArrayList<Float> acolors = new ArrayList<Float>();
		for(envInterface id : d) {
			if( ((AbstractDepth)id).node.getRoot().m_points == null ) {
				System.out.println("node points null in writeFile");
				continue;
			}
			if( ((AbstractDepth)id).node.getRoot().m_colors == null ) {
				System.out.println("node colors null in writeFile");
				continue;
			}
			for(int i = 0; i < ((AbstractDepth)id).node.getIndexes().size(); i++) {
				geometry_msgs.Point32 ppoint = new geometry_msgs.Point32();
				int j = ((AbstractDepth)id).node.getIndexes().get(i);
				Vector4d pnode = ((AbstractDepth)id).node.getRoot().m_points.get(j);
				ppoint.setX((float) pnode.x); // X
				ppoint.setY((float) pnode.y);// Y
				ppoint.setZ((float) ((AbstractDepth)id).depth); // Z
				pnodes.add(ppoint);
				Vector4d pcolor = ((AbstractDepth)id).node.getRoot().m_colors.get(i);
				acolors.add((float)((((int)pcolor.x)<<16) | (((int)pcolor.y)<<8) | (int)pcolor.z ));
			}
		}
		float[] colors = new float[acolors.size()];
		for(int s = 0; s < acolors.size(); s++) colors[s] = acolors.get(s);
		rgb.setValues(colors);
	}
	/**
	 * Gen the RMS error for the 2 DCT arrays of left and right chromospatial sets
	 * @param inode
	 * @param jnode
	 * @param nsize
	 * @return
	 */
	@Deprecated
	private static double genDCT2(octree_t inode, octree_t jnode, int nsize) {
		   float[] coeffsi = new float[nsize];
		   float[] coeffsj = new float[nsize];
		   int i = 0;
		   for(int c = 0; c < nsize; c++) {
				Vector4d lcolor = inode.getRoot().m_colors.get(inode.getIndexes().get(c));
				Vector4d rcolor = jnode.getRoot().m_colors.get(jnode.getIndexes().get(c));				
				coeffsi[c] = (int)lcolor.x<<16 | (int)lcolor.y<<8 | (int)lcolor.z;
				coeffsj[c] = (int)rcolor.x<<16 | (int)rcolor.y<<8 | (int)rcolor.z;
		   }
		   FloatDCT_1D fdct1d = new FloatDCT_1D(nsize);
		   fdct1d.forward(coeffsi, false);
		   fdct1d.forward(coeffsj, false);
		   return IOUtils.computeRMSE(coeffsi, coeffsj);
	}
	/**
	 * Generate the sum of absolute differences on the actual pixel color values for the pixels in each
	 * octree node passed. If they differ in pixel count, fill the missing values with maximum color value.
	 * @param inode
	 * @param jnode
	 * @return
	 */
	@Deprecated
	private static double genSAD(octree_t inode, octree_t jnode) {
		double osum = 0;
		for(int c = 0; c < Math.max(inode.getIndexes().size(), jnode.getIndexes().size()); c++) {
			if( c >= inode.getIndexes().size() || c >= jnode.getIndexes().size()) {
				osum += 16777215; //max 24 bit val
				continue;
			}
			Vector4d lcolor = inode.getRoot().m_colors.get(inode.getIndexes().get(c));
			Vector4d rcolor = jnode.getRoot().m_colors.get(jnode.getIndexes().get(c));				
			osum += (Math.abs((int)lcolor.x-(int)rcolor.x)<<16 |
					Math.abs((int)lcolor.y-(int)rcolor.y)<<8 |
					Math.abs((int)lcolor.z+(int)rcolor.z));
		}
		return osum;
	}
	
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
		/**
		 * Write the pixels from source image at depth * 2
		 * @param d List of envInterface post-processed nodes data
		 * @param filename file to write to
		 * @param compVal if >= 0 the value to compare to depth to write conditionally
		 */
		protected void writeFile(List<envInterface> d, String filename, double compVal) {
			DataOutputStream dos = null;
			File f = new File(outDir+filename+".asc");
			try {
				dos = new DataOutputStream(new FileOutputStream(f));
				for(envInterface id : d) {
					if(compVal < 0 || compVal == ((AbstractDepth)id).depth) {
					if( ((AbstractDepth)id).node.getRoot().m_points == null ) {
						System.out.println("node points null in writeFile");
						continue;
					}
					if( ((AbstractDepth)id).node.getRoot().m_colors == null ) {
						System.out.println("node colors null in writeFile");
						continue;
					}
					for(int i = 0; i < ((AbstractDepth)id).node.getIndexes().size(); i++) {
						int j = ((AbstractDepth)id).node.getIndexes().get(i);
						Vector4d pnode = ((AbstractDepth)id).node.getRoot().m_points.get(j);
						Vector4d pcolor = ((AbstractDepth)id).node.getRoot().m_colors.get(j);
						dos.writeBytes(String.valueOf(pnode.x)); // X
							dos.writeByte(' ');
							dos.writeBytes(String.valueOf(pnode.y));// Y
							dos.writeByte(' ');
							dos.writeBytes(String.valueOf(((AbstractDepth)id).depth*2)); // Z
							dos.writeByte(' ');
							dos.writeBytes(String.valueOf((int)pcolor.x)); // R
							dos.writeByte(' ');
							dos.writeBytes(String.valueOf((int)pcolor.y)); // G
							dos.writeByte(' ');
							dos.writeBytes(String.valueOf((int)pcolor.z)); // B
							dos.writeByte('\r');
							dos.writeByte('\n');
					}
					
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
		
		protected void writeTestPair(envInterface d, envInterface dd, String filename) {
			DataOutputStream dos = null;
			File f = new File(outDir+filename+".asc");
			try {
				dos = new DataOutputStream(new FileOutputStream(f));
					writer_file.line3D(dos, (int)d.getEnv()[0]/*xmin*/, (int)d.getEnv()[1]/*ymin*/, (int)(d.getDepth()*10.0d), 
							(int)d.getEnv()[2]/*xmax*/, (int)d.getEnv()[1]/*ymin*/,  (int)(d.getDepth()*10.0d), 0, 255, 255);
					writer_file.line3D(dos, (int)d.getEnv()[2]/*xmax*/, (int)d.getEnv()[1]/*ymin*/,  (int)(d.getDepth()*10.0d), 
							(int)d.getEnv()[2]/*xmax*/, (int)d.getEnv()[3]/*ymax*/,  (int)(d.getDepth()*10.0d), 0, 255, 255);
					writer_file.line3D(dos, (int)d.getEnv()[2]/*xmax*/, (int)d.getEnv()[3]/*ymax*/,  (int)(d.getDepth()*10.0d), 
							(int)d.getEnv()[0]/*xmin*/, (int)d.getEnv()[3]/*ymax*/, (int)(d.getDepth()*10.0d), 0, 255, 255);
					writer_file.line3D(dos, (int)d.getEnv()[0]/*xmin*/, (int)d.getEnv()[3]/*ymax*/,  (int)(d.getDepth()*10.0d), 
							(int)d.getEnv()[0]/*xmin*/, (int)d.getEnv()[1]/*ymin*/,  (int)(d.getDepth()*10.0d), 0, 255, 255);
					//
					writer_file.line3D(dos, (int)dd.getEnv()[0]/*xmin*/, (int)dd.getEnv()[1]/*ymin*/,  (int)(d.getDepth()*10.0d), 
							(int)dd.getEnv()[2]/*xmax*/, (int)dd.getEnv()[1]/*ymin*/,  (int)(d.getDepth()*10.0d), 255, 255, 0);
					writer_file.line3D(dos, (int)dd.getEnv()[2]/*xmax*/, (int)dd.getEnv()[1]/*ymin*/,  (int)(d.getDepth()*10.0d), 
							(int)dd.getEnv()[2]/*xmax*/, (int)dd.getEnv()[3]/*ymax*/,  (int)(d.getDepth()*10.0d), 255, 255, 0);
					writer_file.line3D(dos, (int)dd.getEnv()[2]/*xmax*/, (int)dd.getEnv()[3]/*ymax*/,  (int)(d.getDepth()*10.0d), 
							(int)dd.getEnv()[0]/*xmin*/, (int)dd.getEnv()[3]/*ymax*/, (int)(d.getDepth()*10.0d), 255, 255, 0);
					writer_file.line3D(dos, (int)dd.getEnv()[0]/*xmin*/, (int)dd.getEnv()[3]/*ymax*/,  (int)(d.getDepth()*10.0d), 
							(int)dd.getEnv()[0]/*xmin*/, (int)dd.getEnv()[1]/*ymin*/,  (int)(d.getDepth()*10.0d), 255, 255, 0);
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
		 * output left processed and unprocessed nodes with 2 different outer envelope colors
		 * @param iproc processed left nodes
		 * @param indexUnproc2 unprocessed left
		 * @param r proc red
		 * @param g proc green
		 * @param b proc blue
		 * @param i unproc red
		 * @param j unproc green
		 * @param k unproc blue
		 * @param fileName
		 */
		public static void writePerp(List<envInterface> iproc, List<envInterface> indexUnproc2, int r, int g, int b, int i, int j, int k, String fileName) {
			DataOutputStream dos = null;
			File f = new File(hough_settings.file+fileName+hough_settings.extension);
		  try {
			dos = new DataOutputStream(new FileOutputStream(f));
			Vector4d cen1 = new Vector4d();
			Vector4d cen2 = new Vector4d();
			Vector4d cen3 = new Vector4d();
			Vector4d cen4 = new Vector4d();
			if( iproc == null || iproc.isEmpty() ){ 
				return;
			}
			for(envInterface c : iproc )  {
				octree_t cnode = ((AbstractDepth)c).node;
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
			if(indexUnproc2 == null || indexUnproc2.isEmpty()) {
				return;
			}
			for(envInterface c : indexUnproc2 )  {
				octree_t cnode = ((AbstractDepth)c).node;
				// spherical of centroid
				writer_file.genPerp(cnode, cen1, cen2, cen3, cen4);
				writer_file.line3D(dos, (int)cnode.getCentroid().x, (int)cnode.getCentroid().y, (int)cnode.getCentroid().z, (int)cen1.x, (int)cen1.y, (int)cen1.z, 255, 255, 255);
				// axis of middle variance, perp to normal
				writer_file.line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, 255, 0, 0);
				// long axis
				// generate point at end of long segment
				writer_file.line3D(dos, (int)cen3.x, (int)cen3.y, (int)cen3.z, (int)cen4.x, (int)cen4.y, (int)cen4.z, 0, 0, 255);
				// outside
				writer_file.line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen4.x, (int)cen4.y, (int)cen4.z, i, j, k);
				writer_file.line3D(dos, (int)cen4.x, (int)cen4.y, (int)cen4.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, i, j, k);
				writer_file.line3D(dos, (int)cen2.x, (int)cen2.y, (int)cen2.z, (int)cen3.x, (int)cen3.y, (int)cen3.z, i, j, k);
				writer_file.line3D(dos, (int)cen3.x, (int)cen3.y, (int)cen3.z, (int)cen1.x, (int)cen1.y, (int)cen1.z, i, j, k);
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
		
		public void writeTestPerp(AbstractDepth d1, IndexDepth d2, String filename) {
			DataOutputStream dos = null;
			File f = new File(outDir+filename+".asc");
			try {
				dos = new DataOutputStream(new FileOutputStream(f));
				Vector4d cen1 = new Vector4d();
				Vector4d cen2 = new Vector4d();
				Vector4d cen3 = new Vector4d();
				Vector4d cen4 = new Vector4d();
				writer_file.genPerp(d1.node, cen1, cen2, cen3, cen4);
				writer_file.line3D(dos, (int)d1.node.getCentroid().x, (int)d1.node.getCentroid().y, (int)d1.node.getCentroid().z, (int)cen1.x, (int)cen1.y, (int)cen1.z, 255, 255, 255);
				// axis of middle variance, perp to normal
				writer_file.line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, 255, 0, 0);
				// long axis
				// generate point at end of long segment
				writer_file.line3D(dos, (int)cen3.x, (int)cen3.y, (int)cen3.z, (int)cen4.x, (int)cen4.y, (int)cen4.z, 0, 0, 255);
				// outside
				writer_file.line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen4.x, (int)cen4.y, (int)cen4.z, 0, 255, 0);
				writer_file.line3D(dos, (int)cen4.x, (int)cen4.y, (int)cen4.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, 0, 255, 0);
				writer_file.line3D(dos, (int)cen2.x, (int)cen2.y, (int)cen2.z, (int)cen3.x, (int)cen3.y, (int)cen3.z, 0, 255, 0);
				writer_file.line3D(dos, (int)cen3.x, (int)cen3.y, (int)cen3.z, (int)cen1.x, (int)cen1.y, (int)cen1.z, 0, 255, 0);
				writer_file.genPerp(d2.node, cen1, cen2, cen3, cen4);
				writer_file.line3D(dos, (int)d2.node.getCentroid().x, (int)d2.node.getCentroid().y, (int)d2.node.getCentroid().z, (int)cen1.x, (int)cen1.y, (int)cen1.z, 255, 255, 255);
				// axis of middle variance, perp to normal
				writer_file.line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, 255, 0, 0);
				// long axis
				// generate point at end of long segment
				writer_file.line3D(dos, (int)cen3.x, (int)cen3.y, (int)cen3.z, (int)cen4.x, (int)cen4.y, (int)cen4.z, 0, 0, 255);
				// outside
				writer_file.line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen4.x, (int)cen4.y, (int)cen4.z, 255, 255, 0);
				writer_file.line3D(dos, (int)cen4.x, (int)cen4.y, (int)cen4.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, 255, 255, 0);
				writer_file.line3D(dos, (int)cen2.x, (int)cen2.y, (int)cen2.z, (int)cen3.x, (int)cen3.y, (int)cen3.z, 255, 255, 0);
				writer_file.line3D(dos, (int)cen3.x, (int)cen3.y, (int)cen3.z, (int)cen1.x, (int)cen1.y, (int)cen1.z, 255, 255, 0);
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
	 * Transform the coord at x,y by the Matrix3 transform provided, translate and rotate.
	 * @param x
	 * @param y
	 * @param transform
	 * @return the transformed coords
	 */
	private int[] scale(int x, int y, Matrix3T transform) {
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
	
	   //---------------------------------
	   // supplementary classes
	   //---------------------------------
	   
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

	class Matrix3T extends Matrix3 {
	    Vertex transform(Vertex in) {
	    	double[] values = getValues();
	        return new Vertex(
	            in.x * values[0] + in.y * values[3] + in.z * values[6],
	            in.x * values[1] + in.y * values[4] + in.z * values[7],
	            in.x * values[2] + in.y * values[5] + in.z * values[8]
	        );
	    }
	    public String toString() {
	    	double[] values = getValues();
	    	return String.format("[%15.8f,%15.8f,%15.8f]\r\n[%15.8f,%15.8f,%15.8f]\r\n[%15.8f,%15.8f,%15.8f]\r\n",
	    			values[0],values[1],values[2],values[3],values[4],values[5],values[6],values[7],values[8]);	
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
		/**
		 * The envelope of the vector of centroid cent with magnitude eigenvalue plus and minus vmag1,
		 * in the direction of eigenvector norm1, to the vector of centroid to norm2 of length plus and minus vmag2. Since
		 * eigenvectors are represented by normal vectors, we scale them to centroid, with angle eigenvector,
		 * magnitude eigenvalue plus and eigenvalue minus. So its a cross centered on centroid with the axis in the direction of
		 * the 2 normal vectors, perpendicular to each other of course, with the coordinates of centroid plus and minus the 2 magnitudes, 
		 * of which the envelope min and max x and y are taken. When we use the spherical_to_cartesian it gives us a 3D
		 * point even though we are concentrating on getting a 2D envelope, and the axis are oriented orthogonal to the 2D
		 * point cloud, so we have to take the maximum Z offset and 'rotate' that to the x axis. We rotate because the x 
		 * coordinates of our 2 generated cross axis are always the same as they are oriented perpendicular to the plane 
		 * of the image.
		 * @param cent centroid of envelope
		 * @param norm1 normal vector of first axis
		 * @param vmag1 magnitude of first normal vector
		 * @param norm2 normal vector of second axis
		 * @param vmag2 magnitude of second normal vector
		 * @return xmin, ymin, xmax, ymax, of envelope of generated cross from passed parameters
		 */
		public static double[] getEnvelope(Vector4d cent, Vector4d norm1, double vmag1, Vector4d norm2, double vmag2) {
			Vector4d cen1 = new Vector4d();
			Vector4d cen2 = new Vector4d();
			Vector4d cen3 = new Vector4d();
			Vector4d cen4 = new Vector4d();
			double[] tpr = octree_t.cartesian_to_spherical(norm1);
			// mid axis normal2
			octree_t.spherical_to_cartesian(cen1, cent, tpr[0], tpr[1], tpr[2], -vmag1);
			// generate point at end of segment
			octree_t.spherical_to_cartesian(cen2, cent, tpr[0], tpr[1], tpr[2], vmag1);
			//System.out.println("IndexDepth enclosedBy cen1="+cen1+" cen2="+cen2+" vmag1="+vmag1+" tpr:"+tpr[0]+" "+tpr[1]+" "+tpr[2]);
			// long axis
			tpr = octree_t.cartesian_to_spherical(norm2);
			// subtract length of longest axis normal3
			octree_t.spherical_to_cartesian(cen3, cent, tpr[0], tpr[1], tpr[2], -vmag2);
			// generate point at end of long segment
			octree_t.spherical_to_cartesian(cen4, cent, tpr[0], tpr[1], tpr[2], vmag2);
			// find greatest z diff, make that x
			double zdiff = Math.max(Math.abs(cent.z-cen4.z), Math.max(Math.abs(cent.z-cen3.z),  
					Math.max(Math.abs(cent.z-cen1.z), Math.abs(cent.z-cen2.z))));
			//System.out.println("IndexDepth enclosedBy cen3="+cen3+" cen4="+cen4+" vmag2="+vmag2+" tpr:"+tpr[0]+" "+tpr[1]+" "+tpr[2]);
			double txmin = Math.min(cen4.x-zdiff,Math.min(cen3.x-zdiff, Math.min(cen1.x-zdiff, cen2.x-zdiff)));
			double txmax = Math.max(cen4.x+zdiff,Math.max(cen3.x+zdiff, Math.max(cen1.x+zdiff, cen2.x+zdiff)));
			double tymin = Math.min(cen4.y,Math.min(cen3.y, Math.min(cen1.y, cen2.y)));
			double tymax = Math.max(cen4.y,Math.max(cen3.y, Math.max(cen1.y, cen2.y)));
			//if(DEBUG)
			//	System.out.println("getEnvelope txmin="+txmin+" tymin="+tymin+" txmax="+txmax+" tymax="+tymax);
			return new double[]{txmin, tymin, txmax, tymax};
		}
		/**
		 * Return the components of the node as real vectors in model space
		 * @param cnode The octree node with centroid, 3 eigenvector normals, eigenvalues 3
		 * @return 5 Vector4d of centroid to normal1 at variance1, centroid to normal2 at variance2, centroid to -normal2, centroid to normal3 at variance3, centroid to -normal3
		 */
		public static Vector4d[] getEnvelope(octree_t cnode) {
			double[] tpr = octree_t.cartesian_to_spherical(cnode.getNormal1());
			// generate normal to plane from centroid scaled to variance1 eigenvalue
			Vector4d cen0 = new Vector4d();
			octree_t.spherical_to_cartesian(cen0, cnode.getCentroid(), tpr[0], tpr[1], tpr[2], cnode.getVariance1());
			// axis of middle variance, perp to normal
			tpr = octree_t.cartesian_to_spherical(cnode.getNormal2());
			if( DEBUG )
				System.out.println("variance2="+cnode.getVariance2()+" tpr:"+tpr[0]+" "+tpr[1]+" "+tpr[2]);
			// mid axis normal2
			Vector4d cen1 = new Vector4d();
			octree_t.spherical_to_cartesian(cen1, cnode.getCentroid(), tpr[0], tpr[1], tpr[2], -cnode.getVariance2());
			// generate point at end of segment
			Vector4d cen2 = new Vector4d();
			octree_t.spherical_to_cartesian(cen2, cnode.getCentroid(), tpr[0], tpr[1], tpr[2], cnode.getVariance2());
			if( DEBUG )
				System.out.println("cen1="+cen1+" "+" cen2="+cen2);
			// long axis
			tpr = octree_t.cartesian_to_spherical(cnode.getNormal3());
			if( DEBUG )
				System.out.println("variance3="+cnode.getVariance3()+" tpr:"+tpr[0]+" "+tpr[1]+" "+tpr[2]);
			// subtract length of longest axis normal3
			Vector4d cen3 = new Vector4d();
			octree_t.spherical_to_cartesian(cen3, cnode.getCentroid(), tpr[0], tpr[1], tpr[2], -cnode.getVariance3());
			Vector4d cen4 = new Vector4d();
			// generate point at end of long segment
			octree_t.spherical_to_cartesian(cen4, cnode.getCentroid(), tpr[0], tpr[1], tpr[2], cnode.getVariance3());
			if( DEBUG )
				System.out.println("cen3="+cen3+" "+" cen4="+cen4);
			return new Vector4d[]{cen0,cen1,cen2,cen3,cen4};
		}
		/**
		 * 
		 *
		 */
		static class AxisNodes {
			public ArrayList<octree_t> nodes = new ArrayList<octree_t>();
			/*public double[] dct;
			public ArrayList<Double> eigens = new ArrayList<Double>();
			public ArrayList<octree_t> nodes = new ArrayList<octree_t>();*/
		}
		/**
		 * Abstract away operations on an envelope that includes a depth. A plane if you will.
		 * A 2D plane set in 3D.
		 *
		 */
		static public interface envInterface {
			public double[] getEnv();
			public double getDepth();
			public boolean encloses(Vector4d tpoint);
			public boolean encloses(envInterface e);
			public boolean enclosedBy(envInterface e);
			public void setDepth(double depth);
		}
		
		static abstract class AbstractDepth implements envInterface {
			public octree_t node;
			public double xmin, ymin, xmax, ymax, zmin, zmax;
			public double depth;
			/**
			 * does the passed point lie within this envelope edges included?
			 * @param tmiddle
			 * @param tm_size
			 * @return
			 */
			@Override
			public boolean encloses(Vector4d point) {
				return (xmin <= point.x && xmax >= point.x && 
						ymin <= point.y && ymax >= point.y &&
						zmin <= point.z && zmax >= point.z);
			}
			/**
			 * does the passed square lie within this one, even partially?
			 * is xmin of this between xmin and xmax of passed and ymin of
			 * this between ymin and ymax of passed OR
			 * xmax of this between xmin and xmax of passed and ymax of this
			 * between ymin and ymax of passed
			 * @param env
			 * @return true if passed envelope lies within this one
			 */
			@Override
			public boolean encloses(envInterface env) {
				return ((xmin >= env.getEnv()[0] && xmin <= env.getEnv()[2] &&
						 ymin >= env.getEnv()[1] && ymin <= env.getEnv()[3]) || 
						(xmax >= env.getEnv()[0] && xmax <= env.getEnv()[2] &&
						 ymin >= env.getEnv()[1] && ymin <= env.getEnv()[3]) ||
						(xmax >= env.getEnv()[0] && xmax <= env.getEnv()[2] &&
						 ymax >= env.getEnv()[1] && ymax <= env.getEnv()[3]) ||
						(xmin >= env.getEnv()[0] && xmin <= env.getEnv()[2] &&
						 ymax >= env.getEnv()[1] && ymax <= env.getEnv()[3]) ||
						(zmin >= env.getEnv()[4] && zmin <= env.getEnv()[5] &&
						 zmax >= env.getEnv()[4] && zmax <= env.getEnv()[5]));
							 
			}
			public abstract boolean enclosedBy(envInterface e);
			@Override
			public boolean equals(Object tind) {
				return ( xmin == ((envInterface)tind).getEnv()[0] && xmax == ((envInterface)tind).getEnv()[2] &&
						 ymin == ((envInterface)tind).getEnv()[1] && ymax == ((envInterface)tind).getEnv()[3] &&
						 zmin == ((envInterface)tind).getEnv()[4] && zmax == ((envInterface)tind).getEnv()[5]);
			}
			@Override
			public double[] getEnv() {
				return new double[]{xmin,ymin,xmax,ymax,zmin,zmax};
			}
			@Override
			public double getDepth() {
				return depth;
			}
			@Override
			public void setDepth(double depth) {
				this.depth = depth;		
			}
		}
		/**
		 * Abstraction of processed envelopes with the addition of depth. Middle typically refers to centroid
		 * of point cloud in octree node. size is typically derived from octree node edge length, but can
		 * also include the length of the vector of centroid with magnitude eigenvalue in direction eigenvector, since
		 * eigenvectors are represented by normal vectors, we scale them to centroid with angle eigenvecotr magnitude eigenvalue plus
		 * and eigenvalue minus.
		 * @author jg
		 *
		 */
		final class IndexDepth extends AbstractDepth {
			public Vector4d middle;
			public IndexDepth(octree_t node, double depth) {
				this.node = node;
				this.middle = node.getMiddle();
				xmin = middle.x-(node.getSize()/2);
				xmax = middle.x+(node.getSize()/2);
				ymin = middle.y-(node.getSize()/2);
				ymax = middle.y+(node.getSize()/2);
				zmin = middle.z-(node.getSize()/2);
				zmax = middle.z+(node.getSize()/2);
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
				double tzmin = tmiddle.z-(tm_size/2);
				double tzmax = tmiddle.z+(tm_size/2);
				return (xmin <= txmin && xmax >= txmax && 
						ymin <= tymin && ymax >= tymax &&
						zmin <= tzmin && zmax >= tzmax );
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
				double tzmin = tmiddle.z-(tm_size/2);
				double tzmax = tmiddle.z+(tm_size/2);
				return (txmin <= middle.x && txmax >= middle.x && 
						tymin <= middle.y && tymax >= middle.y &&
						tzmin <= middle.z && tzmax >= middle.z);
			}
			/**
			 * 
			 * @param tenv txmin,tymin,txmax,tymax
			 * @return
			 */
			@Override
			public boolean enclosedBy(envInterface tenv) {
				//System.out.println("IndexDepth enclosedBy txmin="+txmin+" middle.x="+middle.x+" txmax="+txmax+" tymin="+tymin+" middle.y="+middle.y+" tymax="+tymax);
				return (tenv.getEnv()[0]/*txmin*/ <= middle.x && tenv.getEnv()[2]/*txmax*/ >= middle.x && 
						tenv.getEnv()[1]/*tymin*/ <= middle.y && tenv.getEnv()[3]/*tymax*/ >= middle.y &&
						tenv.getEnv()[4] <= middle.z && tenv.getEnv()[5] >= middle.z);
			}
			@Override
			public String toString() {
				return "IndexDepth xmin="+xmin+" ymin="+ymin+" xmax="+xmax+" ymax="+ymax+" zmin="+zmin+" zmax="+zmax+" depth="+depth+" middle="+middle;
			}
		}
		/**
		 * This class has a similar structure, but quite different behavior , than the IndexDepth class.
		 * It represents the maximal regions along with the octree node at their core
		 * @author jg
		 *
		 */
		final class IndexMax extends AbstractDepth {
			public IndexMax(octree_t node, double[] env, double depth) {
				this.node = node;
				xmin = env[0];
				xmax = env[2];
				ymin = env[1];
				ymax = env[3];
				this.depth = depth;
			}
			/**
			 * does 'this' lie within the passed square?
			 * @param tmiddle
			 * @param tm_size
			 * @return
			 */
			@Override
			public boolean enclosedBy(envInterface env) {
				return ((env.getEnv()[0] >= xmin && env.getEnv()[0] <= xmax &&
						 env.getEnv()[1] >= ymin && env.getEnv()[1] <= ymax) || 
						(env.getEnv()[2] >= xmin && env.getEnv()[2] <= xmax &&
						 env.getEnv()[1] >= ymin && env.getEnv()[1] <= ymax) ||
						(env.getEnv()[2] >= xmin && env.getEnv()[2] <= xmax &&
						 env.getEnv()[3] >= ymin && env.getEnv()[3] <= ymax) ||
						(env.getEnv()[0] >= xmin && env.getEnv()[0] <= xmax &&
						 env.getEnv()[3] >= ymin && env.getEnv()[3] <= ymax) );
			}
	
			@Override
			public String toString() {
				return "IndexMax xmin="+xmin+" ymin="+ymin+" xmax="+xmax+" ymax="+ymax+" depth="+depth+" node="+node;
			}
			
		}
		/**
		 * Parallel left/right image pixels to processing model generator class:<br/>
		 * To initialize:<br/>
		 *  PixelsToModel modelGen = new PixelsToModel();<br/>
		 *  modelGen.spinGen();<br/>
		 * Then for each iteration of new image set:<br/>
		 * 	modelGen.imageL1 = imageL1;<br/>
		 *	modelGen.imageR1 = imageR1;<br/>
		 *  modelGen.latchOutL.reset(); // reset to synch at barrier<br/>
		 *  modelGen.latchOutR.reset();<br/>
		 *  modelGen.latchL.reset(); // after all resets we wait at these barriers at top of thread processing<br/>
		 *  modelGen.latchR.reset();<br/>
		 *  modelGen.latchL.await(); // these trip the barriers to start the threads<br/>
		 *  modelGen.latchR.await();<br/>
		 *  modelGen.latchOutL.await(); // once these 2 barriers are tripped we are done<br/>
		 *  modelGen.latchOutR.await();<br/>
		 * @author jg
		 *
		 */
		final class PixelsToModel {
			private static final boolean DEBUG = true;
			private int[] dataLx; // left array return from model generation (i.e. canny with magnitudes or mean color generator with means)
			private int[] dataRx; // right model array with magnitudes
			BufferedImage imageL1 = null;
			BufferedImage imageR1 = null;
			CyclicBarrier latchL = new CyclicBarrier(2);
			CyclicBarrier latchOutL = new CyclicBarrier(2);
			CyclicBarrier latchR = new CyclicBarrier(2);
			CyclicBarrier latchOutR = new CyclicBarrier(2);
			/**
			 * Activates two threads, one for left, one for right image from stereo camera setup
			 * and enter loop to run genModel method using barrier synchronization to control execution
			 */
			public void spinGen() {
				if(DEBUG)
					System.out.println("Spinning Left Pixels to Model Generator");
				SynchronizedThreadManager.getInstance().spin(new Runnable() {
					@Override
					public void run() {			
						while(true) {
							try {
								latchL.await();
								dataLx = genModel(imageL1, dataLp, "MODELGENL");
								if( VideoProcessor.imageIntegrate > 0 ) {
									if( dataLp == null ) {
										dataLp = dataLx;
									}
								} else {
									dataLp = dataLx;
								}
								latchOutL.await();
							} catch (InterruptedException| BrokenBarrierException e) {}
						}
					}
				});
				if(DEBUG)
					System.out.println("Spinning Right Pixels to Model Generator");
				SynchronizedThreadManager.getInstance().spin(new Runnable() {
					@Override
					public void run() {	
						while(true) {
							try {
								latchR.await();
								dataRx = genModel(imageR1, dataRp, "MODELGENR");
								if( VideoProcessor.imageIntegrate > 0 ) {
									if( dataRp == null ) {
										dataRp = dataRx;
									}
								} else {
									dataRp = dataRx;
								}
								latchOutR.await();
							} catch (InterruptedException | BrokenBarrierException e) {}
						}
					}
				});
			}
			/**
			 * Always return the processed image, but if datap is not null and imageIntegrate has
			 * a value greater than 0 xor it with that as well, for as many iterations as the value of imageIntegrate.
			 * From the caller, do the same check and if datap was null, assign it the returned array.
			 * We are trying to 'build up' the image progressively if doing so will add more fidelity
			 * as in edge generation, but if it wont, as in mean color model the VideoProcessor.imageIntegrate
			 * value will be zero vs a limit number of images we use.
			 * @param img The BufferedImage of our source pixels
			 * @param datap The resulting model generated from image
			 * @param threadGroupName The group name since we are using multiple instances of model generator
			 * @return
			 */
			private int[] genModel(BufferedImage img, int[] datap, String threadGroupName) {
		   	    //ParallelCannyEdgeDetector ced = new ParallelCannyEdgeDetector(threadGroupName);
			    //ced.setLowThreshold(.1f);
			    //ced.setHighThreshold(.2f);
			    //ced.setGaussianKernelRadius(2);
			    //ced.setGaussianKernelWidth(16);
				//---SobelEdgeDetector ced = new SobelEdgeDetector();
			    //ced.setSourceImage(img);
			    //int[] datax = ced.semiProcess();
				MeanColorGenerator mcg = new MeanColorGenerator(img, camWidth, camHeight, false);
				//System.out.println("Mean color value ="+mcg.meanValue());
				int[] datax = mcg.getData();//mcg.mean();
				if( VideoProcessor.imageIntegrate > 0 ) {
					if(datap != null) {
						for(int i = 0; i < datap.length; i++) {
							//for(int j = 0; j < datap[i].length; j++) {
			    			 datap[i] = datap[i] | datax[i];
							//}
						}
					}
				}
				//System.out.println(threadGroupName+" mean="+mcg.meanValue());
				// get image mean
			    return datax;
			}
		}

}


