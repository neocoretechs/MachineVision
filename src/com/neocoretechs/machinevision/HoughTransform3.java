package com.neocoretechs.machinevision;

import java.util.Vector;

public class HoughTransform3 extends HoughTransform {
	int maxd;
	int doubled;
	int houghd;
	private int[][][] houghArray;
	public HoughTransform3(int width, int height, int maxd) {
		super();
	    this.width = width; 
        this.height = height; 
		this.maxd = maxd;
		initialize();	
	}
	  /** 
     * Initialises the hough array. Called by the constructor so you don't need to call it 
     * yourself, however you can use it to reset the transform if you want to plug in another 
     * image (although that image must have the same width and height) 
     */ 
    public void initialize() { 
    	System.out.println("3D init");
        // Calculate the maximum height the hough array needs to have 
        houghHeight = (int) (Math.sqrt(2) * Math.max(height, width)) / 2;
        houghd = (int) (Math.sqrt(2) * maxd) / 2;
 
        // Double the height of the hough array to cope with negative r values 
        doubleHeight = 2 * houghHeight;
        doubled = 2 * houghd;
 
        // Create the hough array 
        houghArray = new int[maxTheta][doubleHeight][doubled]; 
 
        // Find edge points and vote in array 
        centerX = width / 2; 
        centerY = height / 2; 
 
        // Count how many points there are 
        numPoints = 0; 
 
        // cache the values of sin and cos for faster processing 
        sinCache = new double[maxTheta]; 
        cosCache = new double[maxTheta]; 
        for (int t = 0; t < maxTheta; t++) { 
            double realTheta = t * thetaStep; 
            sinCache[t] = Math.sin(realTheta); 
            cosCache[t] = Math.cos(realTheta); 
        } 
    } 
    /** 
     * Adds a single point to the hough transform. You can use this method directly 
     * if your data isn't represented as a buffered image. 
     */ 
    public void addPoint(int x, int y, int d) { 
 
        // Go through each value of theta 
        for (int t = 0; t < maxTheta; t++) { 
 
            //Work out the r values for each theta step 
            int r = (int) (((x - centerX) * cosCache[t]) + ((y - centerY) * sinCache[t])); 
 
            // this copes with negative values of r 
            r += houghHeight; 
 
            if (r < 0 || r >= doubleHeight) continue; 
            if( d >= houghArray[t][r].length) {
            	System.out.println("d="+d+" exceeds "+houghArray[t][r].length);
            	continue;
            }
            // Increment the hough array 
            houghArray[t][r][d]++; 
        } 
        numPoints++; 
    } 
    /** 
     * Once points have been added in some way this method extracts the lines and returns them as a Vector 
     * of HoughLine3 objects, which can be used to draw on the 
     * 
     * @param percentageThreshold The percentage threshold above which lines are determined from the hough array 
     */ 
    public Vector<? extends HoughElem> getLines(int threshold) { 
 
        // Initialise the vector of lines that we'll return 
        Vector<HoughLine3> lines = new Vector<HoughLine3>(20); 
 
        // Only proceed if the hough array is not empty 
        if (numPoints == 0) return lines; 
 
        // Search for local peaks above threshold to draw 
        for (int t = 0; t < maxTheta; t++) { 
            loop: 
            for (int r = neighbourhoodSize; r < doubleHeight - neighbourhoodSize; r++) { 
            	for(int d = neighbourhoodSize; d < doubled -  neighbourhoodSize; d++) {
            		// Only consider points above threshold 
            		if (houghArray[t][r][d] > threshold) { 
            			int peak = houghArray[t][r][d]; 
            			// Check that this peak is indeed the local maxima 
            			for (int dx = -neighbourhoodSize; dx <= neighbourhoodSize; dx++) { 
            				for (int dy = -neighbourhoodSize; dy <= neighbourhoodSize; dy++) {
            					for(int dd = -neighbourhoodSize; dd <= neighbourhoodSize; dd++) {
            						int dt = t + dx; 
            						int dr = r + dy;
            						int ddd = d + dd;
            						if (dt < 0) dt = dt + maxTheta; 
            						else 
            							if (dt >= maxTheta) dt = dt - maxTheta; 
            						if( ddd < 0 )
            							ddd = 0;
            						if (houghArray[dt][dr][ddd] > peak) { 
            							// found a bigger point nearby, skip 
            							continue loop; 
            						}
            					}
            				}
                        } 
                    } 
                    // calculate the true value of theta 
                    double theta = t * thetaStep; 
                    // add the line to the vector 
                    lines.add(new HoughLine3(theta, r, d)); 
                } 
            } 
        }
        return lines; 
    }
}
