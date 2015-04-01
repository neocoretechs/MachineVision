package com.neocoretechs.machinevision;
/*
 * The machine vision transform is based on a decreasing gradient edge detection algorithm 'Canny edge detection', 
then into a variation of the Hough transform that uses a 256 theta parametric space and then a variation of the inverse discrete cosine 
transform that renders a linear array in 'greatest features' order. The optimal distribution characteristic of cosine transforms is apparent when 
a graph of the data is viewed. The original approximately 180000 elements rendered are truncated to 65536 as that seems to be the point at which 'noise' 
renders the values convergent, though additional research may prove these additional data have merit, at this point the storage demands are high and
the additional resolution seems unnecessary.

Each image comprises 65536 object morphisms from cosine transformed image data, rendering representations that are
scale and orientation independent, one of the characteristics of the transform. 

Once the transformation has been completed, the data is stored using the principle of category theory to create morphisms with mappings 
'array element->cosine->category' where 'category' is one provided by the user as image training exemplar. The morphisms are retrieved as posets 
by monotonically increasing 'array element' and an RMS error on each increasing element of the retrieved transformation is performed using its 
derived cosine and the cosine of image under test. When a confidence factor reaches threshold the retrieved 'categories' are analyzed for best match.

The system uses category theoretic from The Relatrix. The machine cognition functions are 
based on algebraic theory and the formation of relationships through 'universal mapping properties' of category theoretic constructs. By forming 
morphisms stored in the Relatrix as serialized Java objects into mathematical constructs such as spans, cospans, then on to adjunctions and 
natural transformations we build higher order relationships from freeform data. The Relatrix allows the user to derive posets from categories 
to use common programming constructs.
 */

import java.io.IOException;
import java.nio.file.DirectoryStream;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.ParseException;
import java.util.Iterator;

import org.jtransforms.utils.IOUtils;

import com.neocoretechs.relatrix.Relatrix;

public class ImgRecognizer {
	private static float[] b = new float[65536]; // currently we work on a hardwired number of maximum elements from transform
	
	public static void getFiles(String dir) throws IOException, ParseException, IllegalAccessException {
		int totalRecords = 0;
		Path path = FileSystems.getDefault().getPath(dir);
		DirectoryStream<Path> files = Files.newDirectoryStream(path); 
		Iterator<Path> it = files.iterator();
		while(it.hasNext()) {
			Path targDir = it.next();
			if( !targDir.toFile().isDirectory() )
				continue;
			DirectoryStream<Path> images = Files.newDirectoryStream(targDir); 
			Iterator<Path> dirit = images.iterator();
			while(dirit.hasNext()) {
				Path targImage = dirit.next();
				long tim = System.currentTimeMillis();
				System.out.println("Processing "+targImage.toAbsolutePath().toString());
				// processfile2 uses static array of floats to reduce object creation overhead
				// since we scale the image to 512x512 the 'doubleheight', which represents the max range of the
				//'radius' values in the transform is constant, and the other value, theta, is preset at 256
				// for this model. So 256 increments of arc for one sinus cycle of hough transform covering
				// the interval from zero to pi
				float[] a = ImgProcessor.processFile2(targImage.toAbsolutePath().toString());
				String category = targImage.getParent().getFileName().toString();
				processPayload(a, category);
				System.out.println("Processed payload in "+(System.currentTimeMillis()-tim)+" ms.");
				++totalRecords;
				//return;
			}
		}
		System.out.println("FINISHED! with "+totalRecords+" processed");
	}
	/**
	 * Map the floating point coefficient of the inverse DCT on the hough of the canny to the category
	 * through the map of the index of the coefficient. For each image, we store these relationships. After all
	 * array elements are stored do a commit, so we have a commit per image
	 * @param a
	 * @param category
	 * @throws IOException
	 * @throws ParseException
	 * @throws IllegalAccessException 
	 */
	public static void processPayload(float[] a, String category) throws IOException, ParseException, IllegalAccessException {
		System.out.println("Attempting to recognize image from category: "+category);
				for(int i = 0; i < 65535; i++) {
					Integer map = new Integer(i);
					Integer mape = new Integer(i+1);
					// and the range is category
					//Comparable rel = 
					try {
						Iterator<?> it = Relatrix.findSubSet("?", map, "?", mape);
						while(it.hasNext()) {
							Comparable[] res =(Comparable[]) it.next();
							//for(int j=0; j < res.length;j++) {
							//	System.out.println("Res:"+j+" "+res[j]);
							//}
							Float cosn = (Float)res[0];
							String categoryStored = (String)res[1];
							b[i] = cosn;
							double rms = IOUtils.computeRMSE(a, b, 0, i+1);
							System.out.println("Index:"+i+" RMS:"+rms+"  Retrieved:"+categoryStored+" Target:"+category+" cos:"+cosn);
						}
					} catch (IllegalArgumentException | ClassNotFoundException e) {
						e.printStackTrace();
					}//(domain, map, category);
					System.out.println("****************Processed "+category+" "+i+"\r");
				}
				System.out.println();
	}
	
	public static void main(String[] args) throws Exception {
		System.out.println("Image recognizer coming up.. source image dir "+args[0] != null ? args[0] : "DEFUALT");
		if(args.length == 2) {
			Relatrix.setTablespaceDirectory(args[1]);
		} else {
			if( args.length == 3 ) {
				Relatrix.setRemoteDirectory(args[2]);
				Relatrix.setTablespaceDirectory(args[1]);
				System.out.println("Tablespace dir "+Relatrix.getTableSpaceDirectory()+" remote dir "+Relatrix.getRemoteDirectory());
			} else {
				System.out.println("Usage: java ImgRecognizer <dir with image files> <db log dir> [remote tablespace dir]");
				return;
			}
		}
		/*
		System.out.println("The following categories are present in the dataset:");
		Iterator<?> it = Relatrix.findSet("*", "*", "?");
		String tcat="";
		while(it.hasNext()) {
			Comparable<?>[] res = (Comparable[]) it.next();
			if( tcat.equals(res[0]))  continue;
			tcat = (String) res[0];
			System.out.println("Category:"+res[0]+" len"+res.length);
		}
		*/
		getFiles(args[0]);
	}
}
