package com.neocoretechs.machinevision;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;
public class SobelEdgeDetector {
	BufferedImage sourceImage;
	int[] data = null;
	int width;
	int height;
	int picsize;
	//static int [] sobel_y =    {1,	0, 	-1, 
	//		2,	0,	-2,
	//		1,	0,	-1};
	static int [] sobel_y =    {-1,	0, 	1, 
		2,	0,	-2,
		-1,	0,	1};
	static int [] sobel_x = 	{1,  2,  1, 
			 0,	 0,	 0,
			-1,	-2,	-1};
	//double[] direction;
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
			width = sourceImage.getWidth();
			height = sourceImage.getHeight();
			picsize = width * height;
			if( data == null)
				data = new int[picsize];
			readLuminance();
		}
		 public int[] sobel(int[] edgesX, int[] edgesY){
		    	int[] result = new int[picsize];
		    	 for(int x = 0; x < width ; x++){
		  		   for(int y = 0; y < height; y++){
		  			   int tmp = Math.abs(edgesX[y*width+x]) + Math.abs(edgesY[y*width+x]);
		  			   //System.out.println(tmp);
		               result[y*width+x]= tmp;//0xff000000|(tmp<<16)|(tmp<<8)|tmp;
				   } 
		  	   	}
		    	return result; 
		    }
		    	
		    private int[] edgeDetection(int[] img, int[] kernel){
		        int[] result = new int[picsize];
		        for(int x = 1; x < width -1 ; x++){
		            for(int y = 1; y < height - 1; y++){
		                int[] tmp = {img[(y-1)*width+(x-1)]&0xff,img[(y-1)*width+x]&0xff,img[(y-1)*width+(x+1)]&0xff,
		                img[y*width+(x-1)]&0xff,img[y*width+x]&0xff,img[y*width+(x+1)]&0xff,img[(y+1)*width+(x-1)]&0xff,
		                img[(y+1)*width+x]&0xff,img[(y+1)*width+(x+1)]&0xff};
		                result[y*width+x]=convolution(kernel, tmp);
		            }
		        }
		        return result;
		    }
		      
		    private int convolution (int [] kernel, int [] pixel){
		        int result = 0; 
		        for (int i = 0; i < pixel.length; i++){
		            result += kernel[i] * pixel[i];
		        }
		        return (int)(Math.abs(result) / 9);
		    }
		    
		    public int[] semiProcess() {
		    	 int[] edgesX = edgeDetection(data, sobel_x);
		         int[] edgesY = edgeDetection(data, sobel_y);
		         return sobel(edgesX,edgesY);
		    	
		    }
	
		
		private int luminance(float r, float g, float b) {
			return Math.round(0.299f * r + 0.587f * g + 0.114f * b);
		}
		
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
	            for (int i = 0; i < picsize; i++) {
	                int b = pixels[offset++] & 0xff;
	                int g = pixels[offset++] & 0xff;
	                int r = pixels[offset++] & 0xff;
	                data[i] = luminance(r, g, b);
	            }
	        } else {
				throw new IllegalArgumentException("Unsupported image type: " + type);
			}
		}
	    /*--------------------------------------------------------------------------------------------
	      Scale Magnitute result prior to writing to a file     
	      sqrt ( pow2( 1024 ) + pow2(1024) ) = 1448.154687
	    /*--------------------------------------------------------------------------------------------*/

	    private double[][] ScaleMagnitude(double[][] Mag) {
	        
	        double[][] mag = new double[Mag.length][Mag[0].length];
	        for (int i = 0; i < Mag.length; i++) {
	            for (int j = 0; j < Mag[i].length; j++) {
	                    mag[i][j] = (Mag[i][j] * 255) / 1448.154687;
	            }
	        }
	        return mag;
	    }
	    /*--------------------------------------------------------------------------------------------
	      Scale Direction result prior to writing to a file             
	      pi = 3.1415926
	      2 * pi = 6.2831853
	    /*--------------------------------------------------------------------------------------------*/

	    private double[][] ScaledDirection(double[][] Dir) {
	        
	        double[][] dir = new double[Dir.length][Dir[0].length];
	        for (int i = 0; i < Dir.length; i++) {
	            for (int j = 0; j < Dir[i].length; j++) {
	                    dir[i][j] = ((Dir[i][j] + 3.1415926) / 6.2831853) * 255;
	            }
	        }
	        return dir;
	    }
	    
	 
	    /*--------------------------------------------------------------------------------------------*/

	    private void ImageWrite(String filename, double img[][]) {
	        try {
	            BufferedImage bi = new BufferedImage(img[0].length, img.length, BufferedImage.TYPE_INT_RGB);

	            for (int i = 0; i < bi.getHeight(); ++i) {
	                for (int j = 0; j < bi.getWidth(); ++j) {
	                    int val = (int) img[i][j];
	                    int pixel = (val << 16) | (val << 8) | (val);
	                    bi.setRGB(j, i, pixel);
	                }
	            }

	            File outputfile = new File(filename);
	            ImageIO.write(bi, "png", outputfile);
	        } catch (IOException e) {
	            System.out.println(e);
	        }
	    }
	    /*--------------------------------------------------------------------------------------------*/
}
