package com.neocoretechs.machinevision;

import java.io.BufferedReader;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.file.DirectoryStream;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.ParseException;
import java.util.Iterator;

import com.neocoretechs.bigsack.DBPhysicalConstants;
import com.neocoretechs.relatrix.Relatrix;

public class ImgLoader {
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
				float[] a = ImgProcessor.processFile(targImage.toAbsolutePath().toString());
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
				for(int i = 0; i < a.length; i++) {
					Float domain = new Float(a[i]);
					Integer map = new Integer(i);
					// and the range is category
					//Comparable rel = 
					Relatrix.transactionalStore(domain, map, category);
					//System.out.println("Going to store "+category+" "+i);
					//Relatrix.store(domain, map, category);
					System.out.print("****************STORED "+category+" "+i+"\r");
				}
				Relatrix.transactionCommit();
				System.out.println();
	}
	
	public static void main(String[] args) throws Exception {
		System.out.println(DBPhysicalConstants.DATASIZE+" "+DBPhysicalConstants.DBLOCKSIZ);
		Relatrix.setTablespaceDirectory(args[1]);
		getFiles(args[0]);

	}
}
