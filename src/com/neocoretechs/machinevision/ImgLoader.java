package com.neocoretechs.machinevision;

import java.io.IOException;

import java.nio.file.DirectoryStream;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.ParseException;
import java.util.Iterator;

import com.neocoretechs.bigsack.DBPhysicalConstants;
import com.neocoretechs.relatrix.DuplicateKeyException;
import com.neocoretechs.relatrix.Relatrix;
import com.neocoretechs.relatrix.client.RelatrixClient;

public class ImgLoader {
	private static RelatrixClient rc;
	
	public static void getFiles(String dir) throws IOException, ParseException, IllegalAccessException, DuplicateKeyException {
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
				System.out.println("Processed payload in "+(System.currentTimeMillis()-tim)+" ms.");
				++totalRecords;
			}
		}
		rc.close();
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
	 * @throws DuplicateKeyException 
	 */
	public static void processPayload(float[] a, String category) throws IOException, ParseException, IllegalAccessException, DuplicateKeyException {
				for(int i = 0; i < 256; i++) {
					Float domain = new Float(a[i]);
					Integer map = new Integer(i);
					// and the range is category
					//Comparable rel = 
					//Relatrix.transactionalStore(domain, map, category);
					rc.store(domain,  map,  category);
					//System.out.println("Going to store "+category+" "+i);
					//Relatrix.store(domain, map, category);
					System.out.print("****************STORED "+category+" "+i+"\r");
				}
				//Relatrix.transactionCommit();
				//rc.transactionCommit();
				System.out.println();
	}
	
	public static void main(String[] args) throws Exception {
		System.out.println("Image loader coming up..blocksize:"+DBPhysicalConstants.DBLOCKSIZ+" args "+args.length);
		if(args.length == 2) {
			rc = new RelatrixClient(/*"C:/Users/jg/Relatrix/AMI"*/ args[1], "localhost", 9000);
			getFiles(args[0]);
		} else {
			if( args.length == 3 ) {
				rc = new RelatrixClient(args[1], args[2], 9000);
				System.out.println("Tablespace dir "+Relatrix.getTableSpaceDirectory());
				getFiles(args[0]);
			} else {
				System.out.println("Usage: java ImgLoader <dir with image files> <db log dir> [remote tablespace dir]");
			}
		}

	}
}
