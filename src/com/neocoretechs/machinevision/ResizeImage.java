package com.neocoretechs.machinevision;

import java.awt.AlphaComposite;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.RenderingHints;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.file.DirectoryStream;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.ParseException;
import java.util.Iterator;
import java.util.Scanner;

import javax.imageio.ImageIO;

import com.neocoretechs.relatrix.DuplicateKeyException;

public class ResizeImage {
	static int width, height;
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
				String category = targImage.getParent().getFileName().toString();
				processPayload(path, category);
				System.out.println("Processed payload in "+(System.currentTimeMillis()-tim)+" ms.");
				++totalRecords;
			}
		}
		System.out.println("FINISHED! with "+totalRecords+" processed");
	}
	
	private static void processPayload(Path path, String category) {
		try {
			Image imageLx = ImageIO.read(path.toFile());
			BufferedImage bi = resizeImage(imageLx, width, height);
			File outputfile = path.toFile();
			ImageIO.write(bi, "jpg", outputfile);
		} catch (IOException e) {
			System.out.println(e);
		}
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
	public static void main(String[] args) throws Exception {
	    //FileInputStream fin = null;
	    Image imageLx;
	    File f = new File(args[0]);
	    if(f.isDirectory()) {
	      	width = Integer.parseInt(args[1]);
	    	height = Integer.parseInt(args[2]);
	    	 try (Scanner keyboard = new Scanner(System.in)) {
	    		 System.out.println("Will overwtrite directory JPG files, type OK to continue...");
	    		 String input = keyboard.nextLine(); 
	    		 if(input.equals("OK"))
	    			 getFiles(args[0]);
	    	 }
	    } else {
	      	width = Integer.parseInt(args[2]);
	    	height = Integer.parseInt(args[3]);
	    	//try(FileInputStream fin = new FileInputStream(f)) {
	    	imageLx = ImageIO.read(f);
	    	BufferedImage bi = resizeImage(imageLx, width, height);
	    	try {
	    		File outputfile = new File(args[1]);
	    		ImageIO.write(bi, "jpg", outputfile);
	    	} catch (IOException e) {
	    		System.out.println(e);
	    	}
		}
	}
}
