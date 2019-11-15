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

import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.file.DirectoryStream;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.ParseException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map.Entry;
import java.util.Set;

import org.jtransforms.utils.IOUtils;

import com.neocoretechs.relatrix.Relatrix;
import com.neocoretechs.relatrix.client.RelatrixClient;
import com.neocoretechs.relatrix.client.RemoteSubsetIterator;

public class ImgRecognizer {
	static RelatrixClient rc = null;
	
	private static float[] b = new float[256]; // currently we work on a hardwired number of maximum elements from transform
	private static int catCount = 0;
	private static HashMap<String, ArrayList<Double>> catToRMS = new HashMap<String, ArrayList<Double>>();
	
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
				float[] a = ImgProcessor.processFile(targImage.toAbsolutePath().toString());
				String category = targImage.getParent().getFileName().toString();
				processPayload(a, category);
				//computeClosest(a);
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
		//System.out.println("Attempting to recognize image from category: "+category);
		int elementsToProcess = 255;
		//ArrayList<Comparable[]> al = new ArrayList<Comparable[]>(256);
		int num = 0;

				for(int i = 0; i < elementsToProcess; i++) {
					num = 0;
					Integer map = new Integer(i);
					Integer mape = new Integer(i+1);
					boolean found = false;
					// and the range is category
					//Comparable rel = 
					String lowCat = null;
					ArrayList<Double> ad = null;
					try {
						//Iterator<?> it = Relatrix.findSubSet("?", map, "?", mape);
						RemoteSubsetIterator it = rc.findSubSet("?", map, "?", mape);
						lowCat = null;
						double rmsLow = Double.MAX_VALUE;
						float lowCo = Float.MAX_VALUE;
						while(rc.hasNext(it)/*it.hasNext()*/) {
							Comparable[] res =(Comparable[]) rc.next(it);//it.next();
							++num;
							//for(int j=0; j < res.length;j++) {
							//	System.out.println("Res #:"+num+" component:"+j+"="+res[j]);
							//}
							Float cosn = (Float)res[0];
							//String categoryStored = (String)res[1];
							// scale it based on significance
							b[i] = (float) ( (((float)i/(float)elementsToProcess) * (a[i]-cosn)) + cosn);
							//ad.add(cosn);
							//b[i] = cosn;
							double rms = IOUtils.computeRMSE(a, b, i+1);//IOUtils.computeMSE(a, b, 0, i+1);
							//double rms = Math.abs(a[i]-b[i]);
							if( rms < rmsLow ) {
								rmsLow = rms;
								lowCat = (String) res[1];
								lowCo = (Float)res[0];
							} else
								found = true;
							//System.out.println("Index:"+i+" element:"+num+" cat:"+res[1]+" cos:"+a[i]+" "+b[i]+" RMS:"+rms);
						}
						// remove remote object
						rc.close(it);
						
						if( lowCat != null ) {
							//al.add(lastLow);
							// tally rms vals by cat
							ad = catToRMS.get(lowCat);
							if( ad == null) {
								ad = new ArrayList<Double>();
								catToRMS.put(lowCat,  ad);
							}
							ad.add((Double)rmsLow);
							b[i] = lowCo;
							System.out.println("Cat:"+lowCat+" best for "+category+" "+i+" detected "+num+" entries, RMS:"+rmsLow);
						} else {
							if( found )
								throw new IOException("At index "+map+" there are no entries BELOW MAX in category:"+category);
							else
								throw new IOException("At index "+map+" there are no entries retrieved for category:"+category);
							
						}
						//System.out.println("Processed "+num+" entries for iteration "+i);
						
					} catch (IllegalArgumentException | ClassNotFoundException e) {
						e.printStackTrace();
					}//(domain, map, category);
	
				}
				System.out.println("Occurance by category, total categories processed:"+catToRMS.size());
				Set<Entry<String, ArrayList<Double>>> es = catToRMS.entrySet();
				Iterator<Entry<String, ArrayList<Double>>> it = es.iterator();
				int ic = 0;
				while(it.hasNext()) {
					Entry<String, ArrayList<Double>> cata = it.next();
					ArrayList<Double> cv = cata.getValue();
					System.out.println(++ic+"="+cata.getKey()+" num:"+cv.size());
				}
				/*
				String fileName = "C:/users/jg/relatrix/"+category+"."+String.valueOf(++catCount);
				FileOutputStream fos = new FileOutputStream(fileName+".csv");
				for(int i = 0; i < al.size(); i++) { // significant coefficients for 256 theta by 512 (+1/2 height) high hough
					fos.write( (((String)al.get(i)[1])+","+(String.valueOf(al.get(i)[0])+"\r\n")).getBytes() );
				}
				fos.flush();fos.close();
				*/
				
	}
	
	/*
	private static void computeClosest(float[] a) {
		double rmsLow = Double.MAX_VALUE;
		Set<Entry<String, ArrayList<Float>>> es = catToRMS.entrySet();
		Iterator<Entry<String, ArrayList<Float>>> it = es.iterator();
		String cat = null;
		while(it.hasNext()) {
			Entry<String, ArrayList<Float>> cata = it.next();
			ArrayList<Float> cv = cata.getValue();
			for(int i = 0; i < 128; i++) b[i] = cv.get(i);
			double rms = IOUtils.computeRMSE(a, b, 0, 128);
			System.out.println(cata.getKey()+" RMS:"+rms);
			if( rms < rmsLow ) {
				rmsLow = rms;
				cat = cata.getKey();
			}
		}
		System.out.println("Closest match is:"+cat+" with RMS:"+rmsLow);
	}
	*/
	
	public static void main(String[] args) throws Exception {
		System.out.println("Image recognizer coming up.. source image dir "+args[0] != null ? args[0] : "DEFAULT");
		if(args.length == 2) {
			//Relatrix.setTablespaceDirectory(args[1]);
			rc = new RelatrixClient(/*"C:/Users/jg/Relatrix/AMI"*/ args[1], "localhost", 9000);
		} else {
			if( args.length == 3 ) {
				//Relatrix.setRemoteDirectory(args[2]);
				//Relatrix.setTablespaceDirectory(args[1]);
				rc = new RelatrixClient(args[1], args[2],"localhost", 9000);
				//System.out.println("Tablespace dir "+Relatrix.getTableSpaceDirectory()+" remote dir "+Relatrix.getRemoteDirectory());
				//System.out.println("Tablespace dir "+Relatrix.getTableSpaceDirectory()+" remote dir "+Relatrix.getRemoteDirectory());
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
		//processPayload();
		getFiles(args[0]);
		rc.close(); // close client
	}
}
