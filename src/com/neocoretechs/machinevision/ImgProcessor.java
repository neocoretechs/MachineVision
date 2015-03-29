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
import java.io.FileReader;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.JFrame;

import org.jtransforms.dct.FloatDCT_2D;
import org.jtransforms.utils.IOUtils;


public class ImgProcessor {
	
	public static final int INBUF_SIZE = 65535;
	private static float[] coeffs = null;
	
	private static void createArrayBuffer(HoughTransform ht) {
		coeffs = HoughTransform.createLinearArray(ht);
	}
	
	public ImgProcessor(String[] args) {
	}
	/**
	 * @param args
	 */
	public static void main(String[] args) {
		String fileName;
		PlayerFrame displayPanel;
		JFrame frame = null;
		new ImgProcessor(args);
		if(args.length<1) {
			System.out.println("Usage: java com.neocoretechs.machinevision.ImgProcessor <raw file>\n");
			return;
		} else {

			frame = new JFrame("Player");
			displayPanel = new PlayerFrame();

			frame.getContentPane().add(displayPanel, BorderLayout.CENTER);

			// Finish setting up the frame, and show it.
			frame.addWindowListener(new WindowAdapter() {
				public void windowClosing(WindowEvent e) {
					System.exit(0);
				}
			});
			displayPanel.setVisible(true);
			frame.pack();
			frame.setVisible(true);
			frame.setSize(new Dimension(672, 836));
			
			fileName = args[0];
			float[] a = processFile(fileName);
			// read in file 2 and do an RMS error compr
			File cmprf1 = new File(fileName);
			File cmprf2 = new File(args[1]);
			String rmsFile = cmprf1.getName()+cmprf2.getName();
			FileReader fis = null;
			try {
				fis = new FileReader(args[1]);
				String cos = null;
				BufferedReader dr = new BufferedReader(fis);
				float[] b = new float[a.length];
				float[] c = new float[a.length];
				for(int i = 0; i < a.length; i++) {
					//System.out.println("Res index:"+i+"="+a[i]);
					cos = dr.readLine();
					b[i]=Float.valueOf(cos);
					c[i]=a[i];
				}
				fis.close();
				System.out.println("RMSE:"+IOUtils.computeRMSE(c, b, 0, c.length, rmsFile));
			} catch (FileNotFoundException e1) {
			} catch (IOException e1) {
				e1.printStackTrace();
			}
		} // if
	}
	
	
	@SuppressWarnings("unused")
	public static float[] processFile(String filename) {
		float summ = 0;
		float[] coeff = null;
	    FileInputStream fin = null;
	    File f = new File(filename);
	    try {

		    fin = new FileInputStream(f);
		    BufferedImage oimg = ImageIO.read(f);
		    BufferedImage img = resizeImage(oimg, 512, 512);
			//displayPanel.lastFrame = img;
		    CannyEdgeDetector ced = new CannyEdgeDetector();
		    //BufferedImage bimage = new BufferedImage(img.getWidth(null), img.getHeight(null), BufferedImage.TYPE_INT_ARGB);
		    // Draw the image on to the buffered image
		    //Graphics2D bGr = bimage.createGraphics();
		    //bGr.drawImage(img, 0, 0, null);
		    //bGr.dispose();
		    // Return the buffered image
		    //return bimage;
		    System.out.println(img.getWidth(null)+" "+img.getHeight(null)+" "+img.getWidth()+" "+img.getHeight());
		    //gf.setWidth(bimage.getWidth());
		    //gf.setHeight(bimage.getHeight());
		    //gf.filter(img, bimage);
		    ced.setSourceImage(img);
		    ced.process();
		    int[] pced = ced.getPixelData();
		    //displayPanel.lastFrame = ced.getEdgesImage();
	        HoughTransform h = new HoughTransform(img.getWidth(), img.getHeight());      
	        // add the points from the image (or call the addPoint method separately if your points are not in an image 
	        h.addPoints((BufferedImage) ced.getEdgesImage()); 
	        // get the lines out 
	        //Vector<HoughLine> lines = h.getLines(30); 
	        // draw the lines back onto the image 
	        //for (int j = 0; j < lines.size(); j++) { 
	        //    HoughLine line = lines.elementAt(j); 
	        //    line.draw((BufferedImage) displayPanel.lastFrame, Color.RED.getRGB()); 
	        //} 
	        /*
	         * Everything going to displayPanel here views the hough transform representation
	         * of sinusoidal wave over theta as sine wave with points as radius and greyscale intensity as 
	         * point density
	         *
	        displayPanel.lastFrame = h.getHoughArrayImage();
			displayPanel.invalidate();
			displayPanel.updateUI();
			*/	
			FloatDCT_2D fdct2d = new FloatDCT_2D(h.getRows(), h.getColumns());
			coeff = h.getLinearArray();
			System.out.println("Linear Array: "+coeff.length+" hough max:"+h.getHighestValue());
			fdct2d.inverse(coeff, false);
			//
			/*
			 * The following will write the result array to a file which is input file name +.csv
			 
			FileOutputStream fos = new FileOutputStream(fileName+".csv");
			for(int i = 0; i < 185344; i++) { // significant coefficients for 256 theta by 512 (+1/2 height) high hough
				fos.write((String.valueOf(coeff[i])+"\r\n").getBytes());
				summ += coeff[i];
			}
			fos.flush();fos.close();
			*/
	
	    } catch(Exception e) {
	    	e.printStackTrace();
	    } finally {
	    	try { fin.close(); } catch(Exception ee) {}
	    } // try

	    //System.out.println("Stop. Sum="+summ);  
	    return coeff;
	}
	/**
	 * Use the encapsulated array coeffs to reduce overhead
	 * @param filename
	 */
	public static float[] processFile2(String filename) {
		float summ = 0;
	    FileInputStream fin = null;
	    File f = new File(filename);
	    try {

		    fin = new FileInputStream(f);
		    BufferedImage oimg = ImageIO.read(f);
		    BufferedImage img = resizeImage(oimg, 512, 512);
			//displayPanel.lastFrame = img;
		    CannyEdgeDetector ced = new CannyEdgeDetector();
		    //BufferedImage bimage = new BufferedImage(img.getWidth(null), img.getHeight(null), BufferedImage.TYPE_INT_ARGB);
		    // Draw the image on to the buffered image
		    //Graphics2D bGr = bimage.createGraphics();
		    //bGr.drawImage(img, 0, 0, null);
		    //bGr.dispose();
		    // Return the buffered image
		    //return bimage;
		    System.out.println(img.getWidth(null)+" "+img.getHeight(null)+" "+img.getWidth()+" "+img.getHeight());
		    ced.setSourceImage(img);
		    ced.process();
		    int[] pced = ced.getPixelData();
		    //displayPanel.lastFrame = ced.getEdgesImage();
	        HoughTransform h = new HoughTransform(img.getWidth(), img.getHeight()); 
	        if( coeffs == null ) {
	        	ImgProcessor.createArrayBuffer(h); // allocate the buffer based on image size set up in hough
	        }
	        // add the points from the image (or call the addPoint method separately if your points are not in an image 
	        h.addPoints((BufferedImage) ced.getEdgesImage()); 
			FloatDCT_2D fdct2d = new FloatDCT_2D(h.getRows(), h.getColumns());
			h.getLinearArray(coeffs);
			System.out.println("Linear Array: "+coeffs.length+" hough max:"+h.getHighestValue());
			fdct2d.inverse(coeffs, false);
	
	    } catch(Exception e) {
	    	e.printStackTrace();
	    } finally {
	    	try { fin.close(); } catch(Exception ee) {}
	    } // try
	    return coeffs;
	    //System.out.println("Stop. Sum="+summ);  
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

}
