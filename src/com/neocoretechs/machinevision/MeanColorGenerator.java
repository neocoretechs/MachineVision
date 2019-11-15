package com.neocoretechs.machinevision;

import java.awt.image.BufferedImage;


public class MeanColorGenerator {
	public static boolean DEBUG = false;
	int mean;
	private int[] data;
	private BufferedImage sourceImage;
	private int width, height;
	
	public MeanColorGenerator(BufferedImage img, int twidth, int theight) {
		sourceImage = img;
		width = twidth;
		height = theight;
		readImage();
	}
	public int[] mean() {
		mean = 0;
		int t = 1;
		for (int x : data) {
			mean += (x - mean) / t;
			++t;
		}
		for(int x = 0; x < data.length; x++)
			data[x] = data[x]-mean;
		return data;
	}
	
	private void readImage() {
		int type = sourceImage.getType();
		switch(type) {
		case BufferedImage.TYPE_INT_RGB:
			if( DEBUG)
			System.out.println(sourceImage+" INT_RGB");
			data = (int[]) sourceImage.getData().getDataElements(0, 0, width, height, null);
			break;
		case BufferedImage.TYPE_INT_ARGB:
			if( DEBUG)
			System.out.println(sourceImage+" INT_ARGB");
			data = (int[]) sourceImage.getData().getDataElements(0, 0, width, height, null);
			for(int x = 0; x < data.length; x++)
				data[x] = data[x] & 0xFFFFFF;
			break;
		case BufferedImage.TYPE_INT_BGR:
			if( DEBUG)
			System.out.println(sourceImage+" INT_BGR");
			break;
		case BufferedImage.TYPE_4BYTE_ABGR:
			if( DEBUG)
			System.out.println(sourceImage+" 4BYTE_ABGR");
			break;
		case BufferedImage.TYPE_3BYTE_BGR:
			if( DEBUG)
			System.out.println(sourceImage+" 3BYTE_BGR");
			byte[] datax = (byte[]) sourceImage.getData().getDataElements(0, 0, width, height, null);
			data = new int[width*height];
			int idata = 0;
			for(int i = 0; i < datax.length; i+=3) {
				data[idata++] = trichromat(datax[i+2],datax[i+1],datax[i]);//(int)datax[i+2] << 16 | (int)datax[i+1] << 8 | (int)datax[i];
			}
			break;
		case BufferedImage.TYPE_USHORT_GRAY:
			if( DEBUG)
			System.out.println(sourceImage+" USHORT_GRAY");
			break;
		case BufferedImage.TYPE_BYTE_GRAY:
			if( DEBUG)
			System.out.println(sourceImage+" BYTE_GRAY");
			break;
		case BufferedImage.TYPE_BYTE_BINARY:
			if( DEBUG)
			System.out.println(sourceImage+" BYTE_BINARY");
			break;
		case BufferedImage.TYPE_CUSTOM:
			if( DEBUG)
			System.out.println(sourceImage+" CUSTOM");
			break;
		default:
			if( DEBUG)
			System.out.println(sourceImage+" HellIfIKnow");
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
 * @return
 */
private int luminance(float r, float g, float b) {
	return Math.round(0.299f * r + 0.587f * g + 0.114f * b);
}
/**
 * These three particular coefficients represent the intensity (luminance) perception of typical 
 * trichromat humans to light of the precise Rec. 709 additive primary colors (chromaticities)
 * that are used in the definition of sRGB.
 * @param r
 * @param g
 * @param b
 * @return
 */
private int trichromat(float r, float g, float b) {
	return Math.round(0.2126f * r + 0.7152f * g + 0.0722f * b);
}

}
