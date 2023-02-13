package com.neocoretechs.machinevision;

import java.awt.image.BufferedImage;
import java.util.Arrays;

import com.neocoretechs.robocore.SynchronizedFixedThreadPoolManager;

/**
 * <p><em>This software has been released into the public domain.
 * <strong>Please read the notes in this source file for additional information.
 * </strong></em></p>
 * 
 * <p>This class provides a configurable implementation of the Canny edge
 * detection algorithm. This classic algorithm has a number of shortcomings,
 * but remains an effective tool in many scenarios. <em>This class uses multithreading
 * in the computeGradients phase, but otherwise does not have, or really need,
 * explicit method synchronization as there will not be much to gain by external multithreading.</em></p>
 * 
 * <p>Sample usage:</p>
 * 
 * <pre><code>
 * //create the detector
 * ParallelCannyEdgeDetector detector = new ParallelCannyEdgeDetector();
 * //adjust its parameters as desired
 * detector.setLowThreshold(0.5f);
 * detector.setHighThreshold(1f);
 * //apply it to an image
 * detector.setSourceImage(frame);
 * detector.process();
 * BufferedImage edges = detector.getEdgesImage();
 * </code></pre>
 * If you are not after an image, but just want the data array with magnitudes, skip the last 2 steps and just call:
 * <code>
 * int[] data = detector.semiProcess();
 * </code>
 * <p>For a more complete understanding of this edge detector's parameters
 * consult an explanation of the algorithm.</p>
 * 
 * @author Tom Gibara
 * @author Groff Copyright (c) NeocoreTechs 2019
 *
 */
public class ParallelCannyEdgeDetector {
	// statics
	private final static boolean TIMER = false; 
	private final static float GAUSSIAN_CUT_OFF = 0.005f;
	private final static float MAGNITUDE_SCALE = 100F;
	private final static float MAGNITUDE_LIMIT = 1000F;
	private final static int MAGNITUDE_MAX = (int) (MAGNITUDE_SCALE * MAGNITUDE_LIMIT);

	// fields
	
	private int height;
	private int width;
	private int picsize;
	private int[] data;
	private int[] magnitude;
	private BufferedImage sourceImage;
	private BufferedImage edgesImage;
	
	private float gaussianKernelRadius;
	private float lowThreshold;
	private float highThreshold;
	private int gaussianKernelWidth;
	private boolean contrastNormalized;

	private float[] xConv;
	private float[] yConv;
	private float[] xGradient;
	private float[] yGradient;
	
	private String threadGroupName;
	
	// constructors
	public ParallelCannyEdgeDetector() {
		this("EDGEDETECT");
	}
	/**
	 * Constructs a new detector with default parameters.
	 */
	public ParallelCannyEdgeDetector(String threadGroupName) {
		lowThreshold = 2.5f;
		highThreshold = 7.5f;
		gaussianKernelRadius = 2f;
		gaussianKernelWidth = 16;
		contrastNormalized = false;
		this.threadGroupName = threadGroupName;
	}

	// accessors
	/**
	 * Valid after call to thresholdEdges
	 * @return
	 */
	public int[] getPixelData() { return data; }
	/**
	 * The image that provides the luminance data used by this detector to
	 * generate edges.
	 * 
	 * @return the source image, or null
	 */
	public BufferedImage getSourceImage() {
		return sourceImage;
	}
	
	/**
	 * Specifies the image that will provide the luminance data in which edges
	 * will be detected. A source image must be set before the process method
	 * is called.
	 *  
	 * @param image a source of luminance data
	 */
	public void setSourceImage(BufferedImage image) {
		sourceImage = image;
	}

	/**
	 * Obtains an image containing the edges detected during the last call to
	 * the process method. The buffered image is an opaque image of type
	 * BufferedImage.TYPE_INT_ARGB in which edge pixels are white and all other
	 * pixels are black.
	 * 
	 * @return an image containing the detected edges, or null if the process
	 * method has not yet been called.
	 */
	public BufferedImage getEdgesImage() {
		return edgesImage;
	}
 
	/**
	 * Sets the edges image. Calling this method will not change the operation
	 * of the edge detector in any way. It is intended to provide a means by
	 * which the memory referenced by the detector object may be reduced.
	 * 
	 * @param edgesImage expected (though not required) to be null
	 */
	public void setEdgesImage(BufferedImage edgesImage) {
		this.edgesImage = edgesImage;
	}

	/**
	 * The low threshold for hysteresis. The default value is 2.5.
	 * 
	 * @return the low hysteresis threshold
	 */
	public float getLowThreshold() {
		return lowThreshold;
	}
	
	/**
	 * Sets the low threshold for hysteresis. Suitable values for this parameter
	 * must be determined experimentally for each application. It is nonsensical
	 * (though not prohibited) for this value to exceed the high threshold value.
	 * 
	 * @param threshold a low hysteresis threshold
	 */
	public void setLowThreshold(float threshold) {
		if (threshold < 0) throw new IllegalArgumentException();
		lowThreshold = threshold;
	}
 
	/**
	 * The high threshold for hysteresis. The default value is 7.5.
	 * 
	 * @return the high hysteresis threshold
	 */
	public float getHighThreshold() {
		return highThreshold;
	}
	
	/**
	 * Sets the high threshold for hysteresis. Suitable values for this
	 * parameter must be determined experimentally for each application. It is
	 * nonsensical (though not prohibited) for this value to be less than the
	 * low threshold value.
	 * 
	 * @param threshold a high hysteresis threshold
	 */
	public void setHighThreshold(float threshold) {
		if (threshold < 0) throw new IllegalArgumentException();
		highThreshold = threshold;
	}

	/**
	 * The number of pixels across which the Gaussian kernel is applied.
	 * The default value is 16.
	 * 
	 * @return the radius of the convolution operation in pixels
	 */
	public int getGaussianKernelWidth() {
		return gaussianKernelWidth;
	}
	
	/**
	 * The number of pixels across which the Gaussian kernel is applied.
	 * This implementation will reduce the radius if the contribution of pixel
	 * values is deemed negligable, so this is actually a maximum radius.
	 * 
	 * @param gaussianKernelWidth a radius for the convolution operation in
	 * pixels, at least 2.
	 */
	public void setGaussianKernelWidth(int gaussianKernelWidth) {
		if (gaussianKernelWidth < 2) throw new IllegalArgumentException();
		this.gaussianKernelWidth = gaussianKernelWidth;
	}

	/**
	 * The radius of the Gaussian convolution kernel used to smooth the source
	 * image prior to gradient calculation. The default value is 16.
	 * 
	 * @return the Gaussian kernel radius in pixels
	 */
	
	public float getGaussianKernelRadius() {
		return gaussianKernelRadius;
	}
	
	/**
	 * Sets the radius of the Gaussian convolution kernel used to smooth the
	 * source image prior to gradient calculation.
	 * 
	 * @return a Gaussian kernel radius in pixels, must exceed 0.1f.
	 */	
	public void setGaussianKernelRadius(float gaussianKernelRadius) {
		if (gaussianKernelRadius < 0.1f) throw new IllegalArgumentException();
		this.gaussianKernelRadius = gaussianKernelRadius;
	}
	
	/**
	 * Whether the luminance data extracted from the source image is normalized
	 * by linearizing its histogram prior to edge extraction. The default value
	 * is false.
	 * 
	 * @return whether the contrast is normalized
	 */
	public boolean isContrastNormalized() {
		return contrastNormalized;
	}
	
	/**
	 * Sets whether the contrast is normalized
	 * @param contrastNormalized true if the contrast should be normalized,
	 * false otherwise
	 */
	public void setContrastNormalized(boolean contrastNormalized) {
		this.contrastNormalized = contrastNormalized;
	}
	
	// methods
	/**
	 * Perform processing all the way up to generation of edgesImage with max luminance white
	 * 0xFF000000 for empty space and max luminance black 0xFFFFFFFF (-1) for edge pixel
	 */
	public void process() {
		semiProcess();
		thresholdEdges();
		writeEdges(data);
		SynchronizedFixedThreadPoolManager.shutdown();
	}
	
	/**
	 * Perform processing up until performHysteresis, so we get data array with magnitudes back;
	 * @return The integer array of each pixel as gradient magnitude.
	 */
	public int[] semiProcess() {
		width = sourceImage.getWidth();
		height = sourceImage.getHeight();
		picsize = width * height;
		initArrays();
		readLuminance();
		if (contrastNormalized) normalizeContrast();
		computeGradients(gaussianKernelRadius, gaussianKernelWidth);
		int low = Math.round(lowThreshold * MAGNITUDE_SCALE);
		int high = Math.round( highThreshold * MAGNITUDE_SCALE);
		performHysteresis(low, high);
		return data;
	}
	
	// private utility methods
	
	private void initArrays() {
		if (data == null || picsize != data.length) {
			data = new int[picsize];
			magnitude = new int[picsize];

			xConv = new float[picsize];
			yConv = new float[picsize];
			xGradient = new float[picsize];
			yGradient = new float[picsize];
		}
	}
	/**
	 * Compute the gradients and perform the non-maximal suppression
	 * using parallelized loop interiors.
	 * @param kernelRadius
	 * @param kernelWidth
	 */
	private void computeGradients(float kernelRadius, int kernelWidth) {
		
		//generate the gaussian convolution masks
		final float kernel[] = new float[kernelWidth];
		float diffKernel[] = new float[kernelWidth];
		int kwidth;
		for (kwidth = 0; kwidth < kernelWidth; kwidth++) {
			float g1 = gaussian(kwidth, kernelRadius);
			if (g1 <= GAUSSIAN_CUT_OFF && kwidth >= 2) break;
			float g2 = gaussian(kwidth - 0.5f, kernelRadius);
			float g3 = gaussian(kwidth + 0.5f, kernelRadius);
			kernel[kwidth] = (g1 + g2 + g3) / 3f / (2f * (float) Math.PI * kernelRadius * kernelRadius);
			diffKernel[kwidth] = g3 - g2;
		}

		int initX = kwidth - 1;
		int maxX = width - (kwidth - 1);
		int initY = width * (kwidth - 1);
		int maxY = width * (height - (kwidth - 1));
		
		//perform convolution in x and y directions
		/*
		for (int x = initX; x < maxX; x++) {
			for (int y = initY; y < maxY; y += width) {
				int index = x + y;
				float sumX = data[index] * kernel[0];
				float sumY = sumX;
				int xOffset = 1;
				int yOffset = width;
				for(; xOffset < kwidth ;) {
					sumY += kernel[xOffset] * (data[index - yOffset] + data[index + yOffset]);
					sumX += kernel[xOffset] * (data[index - xOffset] + data[index + xOffset]);
					yOffset += width;
					xOffset++;
				}
				
				yConv[index] = sumY;
				xConv[index] = sumX;
			}
		}
		*/
		//-----
		try {
			long etime = System.currentTimeMillis();
			int execLimit = (maxX-initX);
			int numThreads = Math.min(16, execLimit);
			final int kkwidth = kwidth;
			//
			// spin all threads necessary for execution of convolve in x and y
			//
			for(int x = initX; x < maxX; x++) {
				final int xx = x;
				final int xinitY = initY;
				final int xmaxY = maxY;
				SynchronizedFixedThreadPoolManager.spin(new Runnable() {
					@Override
					public void run() {
						// convolve xy
						convolveXY(xx, xinitY, xmaxY, kkwidth, kernel);
					} // run									
				},threadGroupName); // spin
				
			}
			SynchronizedFixedThreadPoolManager.waitForGroupToFinish(threadGroupName);
		//-----
		/**
		for (int x = initX; x < maxX; x++) {
			for (int y = initY; y < maxY; y += width) {
				float sum = 0f;
				int index = x + y;
				for (int i = 1; i < kwidth; i++)
					sum += diffKernel[i] * (yConv[index - i] - yConv[index + i]);
				xGradient[index] = sum;
			}
		}
		 */
		//-----
		for(int x = initX; x < maxX; x++) {
			final int xx = x;
			final int xinitY = initY;
			final int xmaxY = maxY;
			final float[] diffKernelx = diffKernel;
			SynchronizedFixedThreadPoolManager.spin(new Runnable() {
				@Override
				public void run() {
					// gradient
					genXGradient(xx, xinitY, xmaxY, kkwidth, diffKernelx);
				} // run									
			},threadGroupName); // spin
			
		}
		SynchronizedFixedThreadPoolManager.waitForGroupToFinish(threadGroupName);
	
		//-----
		/**
		for (int x = kwidth; x < width - kwidth; x++) {
			for (int y = initY; y < maxY; y += width) {
				float sum = 0.0f;
				int index = x + y;
				int yOffset = width;
				for (int i = 1; i < kwidth; i++) {
					sum += diffKernel[i] * (xConv[index - yOffset] - xConv[index + yOffset]);
					yOffset += width;
				}
				yGradient[index] = sum;
			}
		}
		 */
		//-----
		execLimit = width - (2 * kwidth);
		numThreads = Math.min(16, execLimit);
		for (int x = kwidth; x < width - kwidth; x++) {
			final int xx = x;
			final int xinitY = initY;
			final int xmaxY = maxY;
			final float[] diffKernelx = diffKernel;
			SynchronizedFixedThreadPoolManager.spin(new Runnable() {
				@Override
				public void run() {
					// gradient
					genYGradient(xx, xinitY, xmaxY, kkwidth, diffKernelx);
				} // run									
			},threadGroupName); // spin
			
		}
		SynchronizedFixedThreadPoolManager.waitForGroupToFinish(threadGroupName);

		/**
		initX = kwidth;
		maxX = width - kwidth;
		initY = width * kwidth;
		maxY = width * (height - kwidth);
		for (int x = initX; x < maxX; x++) {
			for (int y = initY; y < maxY; y += width) {
				int index = x + y;
				int indexN = index - width;
				int indexS = index + width;
				int indexW = index - 1;
				int indexE = index + 1;
				int indexNW = indexN - 1;
				int indexNE = indexN + 1;
				int indexSW = indexS - 1;
				int indexSE = indexS + 1;
				
				float xGrad = xGradient[index];
				float yGrad = yGradient[index];
				float gradMag = hypot(xGrad, yGrad);

				//perform non-maximal supression
				float nMag = hypot(xGradient[indexN], yGradient[indexN]);
				float sMag = hypot(xGradient[indexS], yGradient[indexS]);
				float wMag = hypot(xGradient[indexW], yGradient[indexW]);
				float eMag = hypot(xGradient[indexE], yGradient[indexE]);
				float neMag = hypot(xGradient[indexNE], yGradient[indexNE]);
				float seMag = hypot(xGradient[indexSE], yGradient[indexSE]);
				float swMag = hypot(xGradient[indexSW], yGradient[indexSW]);
				float nwMag = hypot(xGradient[indexNW], yGradient[indexNW]);
				float tmp;
				//
				// An explanation of what's happening here, for those who want
				// to understand the source: This performs the "non-maximal
				// supression" phase of the Canny edge detection in which we
				// need to compare the gradient magnitude to that in the
				// direction of the gradient; only if the value is a local
				// maximum do we consider the point as an edge candidate.
				// 
				// We need to break the comparison into a number of different
				// cases depending on the gradient direction so that the
				// appropriate values can be used. To avoid computing the
				// gradient direction, we use two simple comparisons: first we
				// check that the partial derivatives have the same sign (1)
				// and then we check which is larger (2). As a consequence, we
				// have reduced the problem to one of four identical cases that
				// each test the central gradient magnitude against the values at
				// two points with 'identical support'; what this means is that
				// the geometry required to accurately interpolate the magnitude
				// of gradient function at those points has an identical
				// geometry (upto right-angled-rotation/reflection).
				// 
				// When comparing the central gradient to the two interpolated
				// values, we avoid performing any divisions by multiplying both
				// sides of each inequality by the greater of the two partial
				// derivatives. The common comparand is stored in a temporary
				// variable (3) and reused in the mirror case (4).
				// 
				//
				if (xGrad * yGrad <= (float) 0 
					? Math.abs(xGrad) >= Math.abs(yGrad) 
						? (tmp = Math.abs(xGrad * gradMag)) >= Math.abs(yGrad * neMag - (xGrad + yGrad) * eMag) 
							&& tmp > Math.abs(yGrad * swMag - (xGrad + yGrad) * wMag) 
						: (tmp = Math.abs(yGrad * gradMag)) >= Math.abs(xGrad * neMag - (yGrad + xGrad) * nMag) 
							&& tmp > Math.abs(xGrad * swMag - (yGrad + xGrad) * sMag) 
					: Math.abs(xGrad) >= Math.abs(yGrad) 
						? (tmp = Math.abs(xGrad * gradMag)) >= Math.abs(yGrad * seMag + (xGrad - yGrad) * eMag) 
							&& tmp > Math.abs(yGrad * nwMag + (xGrad - yGrad) * wMag) 
						: (tmp = Math.abs(yGrad * gradMag)) >= Math.abs(xGrad * seMag + (yGrad - xGrad) * sMag) 
							&& tmp > Math.abs(xGrad * nwMag + (yGrad - xGrad) * nMag) 
					) {
					magnitude[index] = gradMag >= MAGNITUDE_LIMIT ? MAGNITUDE_MAX : (int) (MAGNITUDE_SCALE * gradMag);
					//NOTE: The orientation of the edge is not employed by this
					//implementation. It is a simple matter to compute it at
					//this point as: Math.atan2(yGrad, xGrad);
				} else {
					magnitude[index] = 0;
				}
			}
		}
		*/
		//-----		
		initX = kwidth;
		maxX = width - kwidth;
		initY = width * kwidth;
		maxY = width * (height - kwidth);
		execLimit = maxX - initX;
		numThreads = Math.min(16, execLimit);
		for (int x = initX; x < maxX; x++) {
			final int xx = x;
			final int xinitY = initY;
			final int xmaxY = maxY;
			SynchronizedFixedThreadPoolManager.spin(new Runnable() {
				@Override
				public void run() {
					// gradient
					maxSuppression(xx, xinitY, xmaxY);
				} // run									
			},threadGroupName); // spin		
		}
		SynchronizedFixedThreadPoolManager.waitForGroupToFinish(threadGroupName);
		//-----
		if( TIMER )
			System.out.println("ParallelCannyEdgeDetector.computeGradients time="+(System.currentTimeMillis()-etime));
		} catch (InterruptedException e) { System.out.println("ParallelCannyEdgeDetector.computeGradients threading interrupted!"); }
	}
	/**
	 * Support for computeGradients parallel computation internal loops.
	 * @param x
	 * @param initY
	 * @param maxY
	 * @param kwidth
	 * @param kernel
	 */
	private void convolveXY(int x, int initY, int maxY, int kwidth, float[] kernel) {
		for (int y = initY; y < maxY; y += width) {
			int index = x + y;
			float sumX = data[index] * kernel[0];
			float sumY = sumX;
			int xOffset = 1;
			int yOffset = width;
			for(; xOffset < kwidth ;) {
				sumY += kernel[xOffset] * (data[index - yOffset] + data[index + yOffset]);
				sumX += kernel[xOffset] * (data[index - xOffset] + data[index + xOffset]);
				yOffset += width;
				xOffset++;
			}
			synchronized(yConv) {
				yConv[index] = sumY;
			}
			synchronized(xConv) {
				xConv[index] = sumX;
			}
		}
	}
	/**
	 * Support for computeGradients parallel computation internal loops.
	 * @param x
	 * @param initY
	 * @param maxY
	 * @param kwidth
	 * @param diffKernel
	 */
	private void genXGradient(int x, int initY, int maxY, int kwidth, float[] diffKernel) {
		for (int y = initY; y < maxY; y += width) {
			float sum = 0f;
			int index = x + y;
			for (int i = 1; i < kwidth; i++)
				sum += diffKernel[i] * (yConv[index - i] - yConv[index + i]);
			synchronized(xGradient) {
				xGradient[index] = sum;
			}
		}
	}
	/**
	 * Support for computeGradients parallel computation internal loops.
	 * @param x
	 * @param initY
	 * @param maxY
	 * @param kwidth
	 * @param diffKernel
	 */
	private void genYGradient(int x, int initY, int maxY, int kwidth, float[] diffKernel) {
		for (int y = initY; y < maxY; y += width) {
			float sum = 0.0f;
			int index = x + y;
			int yOffset = width;
			for (int i = 1; i < kwidth; i++) {
				sum += diffKernel[i] * (xConv[index - yOffset] - xConv[index + yOffset]);
				yOffset += width;
			}
			synchronized(yGradient) {
				yGradient[index] = sum;
			}
		}
	}
	/**
	 * Support for computeGradients parallel computation internal loops.
	 * This performs the "non-maximal suppression" phase of the Canny edge detection.
	 * We need to compare the gradient magnitude to that in the
	 * direction of the gradient; only if the value is a local maximum do we consider the point as an edge candidate.
	 * @param x base offset in X
	 * @param initY initial Y position
	 * @param maxY Y incremented from initY to maxY by width
	 */
	private void maxSuppression(int x, int initY, int maxY) {
		for (int y = initY; y < maxY; y += width) {
			int index = x + y;
			int indexN = index - width;
			int indexS = index + width;
			int indexW = index - 1;
			int indexE = index + 1;
			int indexNW = indexN - 1;
			int indexNE = indexN + 1;
			int indexSW = indexS - 1;
			int indexSE = indexS + 1;
			
			float xGrad = xGradient[index];
			float yGrad = yGradient[index];
			float gradMag = hypot(xGrad, yGrad);

			//perform non-maximal supression
			float nMag = hypot(xGradient[indexN], yGradient[indexN]);
			float sMag = hypot(xGradient[indexS], yGradient[indexS]);
			float wMag = hypot(xGradient[indexW], yGradient[indexW]);
			float eMag = hypot(xGradient[indexE], yGradient[indexE]);
			float neMag = hypot(xGradient[indexNE], yGradient[indexNE]);
			float seMag = hypot(xGradient[indexSE], yGradient[indexSE]);
			float swMag = hypot(xGradient[indexSW], yGradient[indexSW]);
			float nwMag = hypot(xGradient[indexNW], yGradient[indexNW]);
			float tmp;
			// 
			// We need to break the comparison into a number of different
			// cases depending on the gradient direction so that the
			// appropriate values can be used. To avoid computing the
			// gradient direction, we use two simple comparisons: first we
			// check that the partial derivatives have the same sign (1)
			// and then we check which is larger (2). As a consequence, we
			// have reduced the problem to one of four identical cases that
			// each test the central gradient magnitude against the values at
			// two points with 'identical support'; what this means is that
			// the geometry required to accurately interpolate the magnitude
			// of gradient function at those points has an identical
			// geometry (upto right-angled-rotation/reflection).
			// 
			// When comparing the central gradient to the two interpolated
			// values, we avoid performing any divisions by multiplying both
			// sides of each inequality by the greater of the two partial
			// derivatives. The common comparand is stored in a temporary
			// variable (3) and reused in the mirror case (4).
			// 
			//
			if (xGrad * yGrad <= (float) 0 /*(1)*/
				? Math.abs(xGrad) >= Math.abs(yGrad) /*(2)*/
					? (tmp = Math.abs(xGrad * gradMag)) >= Math.abs(yGrad * neMag - (xGrad + yGrad) * eMag) /*(3)*/
						&& tmp > Math.abs(yGrad * swMag - (xGrad + yGrad) * wMag) /*(4)*/
					: (tmp = Math.abs(yGrad * gradMag)) >= Math.abs(xGrad * neMag - (yGrad + xGrad) * nMag) /*(3)*/
						&& tmp > Math.abs(xGrad * swMag - (yGrad + xGrad) * sMag) /*(4)*/
				: Math.abs(xGrad) >= Math.abs(yGrad) /*(2)*/
					? (tmp = Math.abs(xGrad * gradMag)) >= Math.abs(yGrad * seMag + (xGrad - yGrad) * eMag) /*(3)*/
						&& tmp > Math.abs(yGrad * nwMag + (xGrad - yGrad) * wMag) /*(4)*/
					: (tmp = Math.abs(yGrad * gradMag)) >= Math.abs(xGrad * seMag + (yGrad - xGrad) * sMag) /*(3)*/
						&& tmp > Math.abs(xGrad * nwMag + (yGrad - xGrad) * nMag) /*(4)*/
				) {
				synchronized(magnitude) {
					magnitude[index] = gradMag >= MAGNITUDE_LIMIT ? MAGNITUDE_MAX : (int) (MAGNITUDE_SCALE * gradMag);
				}
				//NOTE: The orientation of the edge is not employed by this
				//implementation. It is a simple matter to compute it at
				//this point as: Math.atan2(yGrad, xGrad);
			} else {
				synchronized(magnitude) {
					magnitude[index] = 0;
				}
			}
		}
	}
	/**
	 * It is quite feasible to replace the implementation of this method
	 * with one which only loosely approximates the hypot function.
	 * simple approximations such as Math.abs(x) + Math.abs(y) work fine.
	 * @param x
	 * @param y
	 * @return
	 */
	private float hypot(float x, float y) {
		return (float) Math.hypot(x, y);
	}
 
	private float gaussian(float x, float sigma) {
		return (float) Math.exp(-(x * x) / (2f * sigma * sigma));
	}
	/**
	 * This implementation reuses the data array to store both
	 * luminance data from the image, and edge intensity from the processing.
	 * This is done for memory efficiency
	 * @param low
	 * @param high
	 */
	private void performHysteresis(int low, int high) {
		Arrays.fill(data, 0);
 
		int offset = 0;
		for (int y = 0; y < height; y++) {
			for (int x = 0; x < width; x++) {
				if (data[offset] == 0 && magnitude[offset] >= high) {
					follow(x, y, offset, low);
				}
				offset++;
			}
		}
 	}
	/**
	 * Recursive method supporting performHysteresis
	 * @param x1
	 * @param y1
	 * @param i1
	 * @param threshold
	 */
	private void follow(int x1, int y1, int i1, int threshold) {
		int x0 = x1 == 0 ? x1 : x1 - 1;
		int x2 = x1 == width - 1 ? x1 : x1 + 1;
		int y0 = y1 == 0 ? y1 : y1 - 1;
		int y2 = y1 == height -1 ? y1 : y1 + 1;
		
		data[i1] = magnitude[i1];
		for (int x = x0; x <= x2; x++) {
			for (int y = y0; y <= y2; y++) {
				int i2 = x + y * width;
				if ((y != y1 || x != x1)
					&& data[i2] == 0 
					&& magnitude[i2] >= threshold) {
					follow(x, y, i2, threshold);
					return;
				}
			}
		}
	}
	/**
	 * Convert the magnitude value in the data array to an ARGB black or white pixel
	 * for pixel positions with magnitude > 0
	 */
	private void thresholdEdges() {
		for (int i = 0; i < picsize; i++) {
			data[i] = data[i] > 0 ? -1 : 0xff000000;
		}
	}
	/**
	 * Luma represents the achromatic image while chroma represents the color component. 
	 * In video systems such as PAL, SECAM, and NTSC, a nonlinear luma component (Y') is calculated directly 
	 * from gamma-compressed primary intensities as a weighted sum, which, although not a perfect 
	 * representation of the colorimetric luminance, can be calculated more quickly without 
	 * the gamma expansion and compression used in photometric/colorimetric calculations. 
	 * In the Y'UV and Y'IQ models used by PAL and NTSC, the rec601 luma (Y') component is computed as
	 * Math.round(0.299f * r + 0.587f * g + 0.114f * b);
	 * rec601 Methods encode 525-line 60 Hz and 625-line 50 Hz signals, both with an active region covering 
	 * 720 luminance samples and 360 chrominance samples per line. The color encoding system is known as YCbCr 4:2:2.
	 * @param r
	 * @param g
	 * @param b
	 * @return Y'
	 */
	private int luminance(float r, float g, float b) {
		return Math.round(0.299f * r + 0.587f * g + 0.114f * b);
	}
	
	/**
	 * Fill the data array with grayscale adjusted image data from sourceImage
	 */
	private void readLuminance() {
		int type = sourceImage.getType();
		if (type == BufferedImage.TYPE_INT_RGB || type == BufferedImage.TYPE_INT_ARGB) {
			int[] pixels = (int[]) sourceImage.getData().getDataElements(0, 0, width, height, null);
			for (int i = 0; i < picsize; i++) {
				int p = pixels[i];
				int r = (p & 0xff0000) >> 16;
				int g = (p & 0xff00) >> 8;
				int b = p & 0xff;
				data[i] = luminance(r, g, b);
			}
		} else if (type == BufferedImage.TYPE_BYTE_GRAY) {
			byte[] pixels = (byte[]) sourceImage.getData().getDataElements(0, 0, width, height, null);
			for (int i = 0; i < picsize; i++) {
				data[i] = (pixels[i] & 0xff);
			}
		} else if (type == BufferedImage.TYPE_USHORT_GRAY) {
			short[] pixels = (short[]) sourceImage.getData().getDataElements(0, 0, width, height, null);
			for (int i = 0; i < picsize; i++) {
				data[i] = (pixels[i] & 0xffff) / 256;
			}
		} else if (type == BufferedImage.TYPE_3BYTE_BGR) {
            byte[] pixels = (byte[]) sourceImage.getData().getDataElements(0, 0, width, height, null);
            int offset = 0;
            int index = 0;
            for (int i = 0; i < picsize; i++) {
                int b = pixels[offset++] & 0xff;
                int g = pixels[offset++] & 0xff;
                int r = pixels[offset++] & 0xff;
                data[index++] = luminance(r, g, b);
            }
        } else {
			throw new IllegalArgumentException("Unsupported image type: " + type);
		}
	}
	
	/**
	 * Optional method to normalize contrast in data array using histogram.
	 */
	private void normalizeContrast() {
		int[] histogram = new int[256];
		for (int i = 0; i < data.length; i++) {
			histogram[data[i]]++;
		}
		int[] remap = new int[256];
		int sum = 0;
		int j = 0;
		for (int i = 0; i < histogram.length; i++) {
			sum += histogram[i];
			int target = sum*255/picsize;
			for (int k = j+1; k <=target; k++) {
				remap[k] = i;
			}
			j = target;
		}
		
		for (int i = 0; i < data.length; i++) {
			data[i] = remap[data[i]];
		}
	}
	/**
	 * There is currently no mechanism for obtaining the edge data
	 * in any other format other than an INT_ARGB type BufferedImage.
	 * This may be easily remedied by providing alternative accessors.
	 * @param pixels The pixel array which is converted to edgesImage BufferedImage instance.
	 */
	public void writeEdges(int pixels[]) {
		if (edgesImage == null) {
			edgesImage = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
		}
		edgesImage.getWritableTile(0, 0).setDataElements(0, 0, width, height, pixels);
	}
	
}