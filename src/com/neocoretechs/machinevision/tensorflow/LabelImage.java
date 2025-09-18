package com.neocoretechs.machinevision.tensorflow;

import java.io.IOException;
import java.io.PrintStream;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import org.tensorflow.DataType;
import org.tensorflow.Graph;
import org.tensorflow.Operation;
import org.tensorflow.Output;
import org.tensorflow.Session;
import org.tensorflow.Session.Run;
import org.tensorflow.Tensor;
import org.tensorflow.TensorFlow;

import org.tensorflow.types.UInt8;

/** Sample use of the TensorFlow Java API to label images using a pre-trained model. */
public class LabelImage {
	//static int model_width = 300; // 224 for inception5h
	//static int model_height = 300; // 224 for inception 5h;
	//static String model_name = "frozen_inference_graph.pb";// tensorflow_inception_graph.pb for inception5h
	//static String label_file = "coco80.txt";// imagenet_comp_graph_label_strings.txt for inception5h
	static int model_width = 224;// for inception5h
	static int model_height = 224;// for inception 5h;
	static String model_name = "tensorflow_inception_graph.pb";// for inception5h
	static String label_file = "imagenet_comp_graph_label_strings.txt";// for inception5h
	
  private static void printUsage(PrintStream s) {
    s.println("TensorFlow version: " + TensorFlow.version());
    s.println();
    s.println("Usage: java -cp /home/libtensorflow-1.14.0.jar;/home/LabelImage.jar -Djava.library.path=/jni/libtensorflow_jni-gpu-windows-x86 com.neocoretechs.machinevision.tensorflow.LabelImage <model dir> <image file>");
    s.println();
    s.println("Where:");
    s.println("<model dir> is a directory containing the unzipped contents of the inception model");
    s.println("<image file> is the path to a JPEG image file");
  }

  public static void main(String[] args) {
    if (args.length != 2) {
      printUsage(System.err);
      System.exit(1);
    }
    String modelDir = args[0];
    String imageFile = args[1];

    byte[] graphDef = readAllBytesOrExit(Paths.get(modelDir, model_name));//"tensorflow_inception_graph.pb"));
    List<String> labels =
        readAllLinesOrExit(Paths.get(modelDir, label_file));//"imagenet_comp_graph_label_strings.txt"));
    byte[] imageBytes = readAllBytesOrExit(Paths.get(imageFile));

    try (Tensor<Float> image = constructAndExecuteGraphToNormalizeImage(imageBytes)) {
      /*float[] labelProbabilities = executeInceptionGraph(graphDef, image);
      int bestLabelIdx = maxIndex(labelProbabilities);
      System.out.println(
          String.format("BEST MATCH: %s (%.2f%% likely)",
              labels.get(bestLabelIdx),
              labelProbabilities[bestLabelIdx] * 100f));
              */
    	List<Tensor<?>> result = executeInceptionGraphAll(graphDef, image);
    	for(Tensor<?> t: result) {
    		System.out.println(t);
    		float[] results = t.copyTo(new float[1][(int)t.shape()[1]])[0];
    		System.out.println(Arrays.toString(results));
    		System.out.println("---------------------");
    	}
    }
    	
  }

  private static Tensor<Float> constructAndExecuteGraphToNormalizeImage(byte[] imageBytes) {
    try (Graph g = new Graph()) {
      GraphBuilder b = new GraphBuilder(g);
      // Some constants specific to the pre-trained model at:
      // https://storage.googleapis.com/download.tensorflow.org/models/inception5h.zip
      //
      // - The model was trained with images scaled to 224x224 pixels.
      // - The colors, represented as R, G, B in 1-byte each were converted to
      //   float using (value - Mean)/Scale.
      final int H = model_width;
      final int W = model_height;
      final float mean = 117f;
      final float scale = 1f;

      // Since the graph is being constructed once per execution here, we can use a constant for the
      // input image. If the graph were to be re-used for multiple input images, a placeholder would
      // have been more appropriate.
      final Output<String> input = b.constant("input", imageBytes);
      final Output<Float> output =
          b.div(
              b.sub(
                  b.resizeBilinear(
                      b.expandDims(
                          b.cast(b.decodeJpeg(input, 3), Float.class),
                          b.constant("make_batch", 0)),
                      b.constant("size", new int[] {H, W})),
                  b.constant("mean", mean)),
              b.constant("scale", scale));
      try (Session s = new Session(g)) {
        // Generally, there may be multiple output tensors, all of them must be closed to prevent resource leaks.
        return s.runner().fetch(output.op().name()).run().get(0).expect(Float.class);
      }
    }
  }

  private static float[] executeInceptionGraph(byte[] graphDef, Tensor<Float> image) {
    try (Graph g = new Graph()) {
      g.importGraphDef(graphDef);
      try (Session s = new Session(g);
          // Generally, there may be multiple output tensors, all of them must be closed to prevent resource leaks.
          Tensor<Float> result =
              s.runner().feed("input", image).fetch("output").run().get(0).expect(Float.class)) {
        final long[] rshape = result.shape();
        if (result.numDimensions() != 2 || rshape[0] != 1) {
          throw new RuntimeException(
              String.format(
                  "Expected model to produce a [1 N] shaped tensor where N is the number of labels, instead it produced one with shape %s",
                  Arrays.toString(rshape)));
        }
        int nlabels = (int) rshape[1];
        return result.copyTo(new float[1][nlabels])[0];
      }
    }
  }

  private static List<Tensor<?>> executeInceptionGraphAll(byte[] graphDef, Tensor<Float> image) {
	  List<Tensor<?>> result;
	  	try (Graph g = new Graph()) {
	      g.importGraphDef(graphDef);
	      Session s = new Session(g);
	      Iterator<Operation> ito = g.operations();
	      int i = 0;
	      while(ito.hasNext()) {
	    	  System.out.println(++i+") "+ito.next());
	      }
	      // Generally, there may be multiple output tensors, all of them must be closed to prevent resource leaks.
	      result = s.runner().feed("input", image).fetch("output").run();//.get(0).expect(Float.class);
	      s.close();
	      return result;
	    }
	  }

  private static int maxIndex(float[] probabilities) {
    int best = 0;
    for (int i = 1; i < probabilities.length; ++i) {
      if (probabilities[i] > probabilities[best]) {
        best = i;
      }
    }
    return best;
  }

  private static byte[] readAllBytesOrExit(Path path) {
    try {
      return Files.readAllBytes(path);
    } catch (IOException e) {
      System.err.println("Failed to read [" + path + "]: " + e.getMessage());
      System.exit(1);
    }
    return null;
  }

  private static List<String> readAllLinesOrExit(Path path) {
    try {
      return Files.readAllLines(path, Charset.forName("UTF-8"));
    } catch (IOException e) {
      System.err.println("Failed to read [" + path + "]: " + e.getMessage());
      System.exit(0);
    }
    return null;
  }

}
