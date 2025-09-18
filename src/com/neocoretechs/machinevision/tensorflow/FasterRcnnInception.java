package com.neocoretechs.machinevision.tensorflow;


import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

/*

From the web page this is the output dictionary

num_detections: a tf.int tensor with only one value, the number of detections [N].
detection_boxes: a tf.float32 tensor of shape [N, 4] containing bounding box coordinates in the following order: [ymin, xmin, ymax, xmax].
detection_classes: a tf.int tensor of shape [N] containing detection class index from the label file.
detection_scores: a tf.float32 tensor of shape [N] containing detection scores.
raw_detection_boxes: a tf.float32 tensor of shape [1, M, 4] containing decoded detection boxes without Non-Max suppression. M is the number of raw detections.
raw_detection_scores: a tf.float32 tensor of shape [1, M, 90] and contains class score logits for raw detection boxes. M is the number of raw detections.
detection_anchor_indices: a tf.float32 tensor of shape [N] and contains the anchor indices of the detections after NMS.
detection_multiclass_scores: a tf.float32 tensor of shape [1, N, 90] and contains class score distribution (including background) for detection boxes in the image including background class.

However using
venv\Scripts\python.exe venv\Lib\site-packages\tensorflow\python\tools\saved_model_cli.py show --dir models\faster_rcnn_inception_resnet_v2_1024x1024 --all
2021-03-19 12:25:37.000143: I tensorflow/stream_executor/platform/default/dso_loader.cc:49] Successfully opened dynamic library cudart64_110.dll

MetaGraphDef with tag-set: 'serve' contains the following SignatureDefs:

signature_def['__saved_model_init_op']:
  The given SavedModel SignatureDef contains the following input(s):
  The given SavedModel SignatureDef contains the following output(s):
    outputs['__saved_model_init_op'] tensor_info:
        dtype: DT_INVALID
        shape: unknown_rank
        name: NoOp
  Method name is:

signature_def['serving_default']:
  The given SavedModel SignatureDef contains the following input(s):
    inputs['input_tensor'] tensor_info:
        dtype: DT_UINT8
        shape: (1, -1, -1, 3)
        name: serving_default_input_tensor:0
  The given SavedModel SignatureDef contains the following output(s):
    outputs['detection_anchor_indices'] tensor_info:
        dtype: DT_FLOAT
        shape: (1, 300)
        name: StatefulPartitionedCall:0
    outputs['detection_boxes'] tensor_info:
        dtype: DT_FLOAT
        shape: (1, 300, 4)
        name: StatefulPartitionedCall:1
    outputs['detection_classes'] tensor_info:
        dtype: DT_FLOAT
        shape: (1, 300)
        name: StatefulPartitionedCall:2
    outputs['detection_multiclass_scores'] tensor_info:
        dtype: DT_FLOAT
        shape: (1, 300, 91)
        name: StatefulPartitionedCall:3
    outputs['detection_scores'] tensor_info:
        dtype: DT_FLOAT
        shape: (1, 300)
        name: StatefulPartitionedCall:4
    outputs['num_detections'] tensor_info:
        dtype: DT_FLOAT
        shape: (1)
        name: StatefulPartitionedCall:5
    outputs['raw_detection_boxes'] tensor_info:
        dtype: DT_FLOAT
        shape: (1, 300, 4)
        name: StatefulPartitionedCall:6
    outputs['raw_detection_scores'] tensor_info:
        dtype: DT_FLOAT
        shape: (1, 300, 91)
        name: StatefulPartitionedCall:7
  Method name is: tensorflow/serving/predict

Defined Functions:
  Function Name: '__call__'
    Option #1
      Callable with:
        Argument #1
          input_tensor: TensorSpec(shape=(1, None, None, 3), dtype=tf.uint8, name='input_tensor')

So it appears there's a discrepancy between the web page and running saved_model_cli as
num_detections: a tf.int tensor with only one value, the number of detections [N].
but the actual tensor is DT_FLOAT according to saved_model_cli
also the web page states
detection_classes: a tf.int tensor of shape [N] containing detection class index from the label file.
but again the actual tensor is DT_FLOAT according to saved_model_cli.
*/


import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import org.tensorflow.Graph;
import org.tensorflow.Operand;
import org.tensorflow.Output;
import org.tensorflow.SavedModelBundle;
import org.tensorflow.Session;
import org.tensorflow.Tensor;
//import org.tensorflow.ndarray.FloatNdArray;
//import org.tensorflow.ndarray.Shape;
import org.tensorflow.op.Ops;
import org.tensorflow.op.core.Constant;
import org.tensorflow.op.core.Placeholder;
import org.tensorflow.op.core.Reshape;
import org.tensorflow.op.image.DecodeJpeg;
import org.tensorflow.op.image.EncodeJpeg;
import org.tensorflow.op.io.ReadFile;
import org.tensorflow.op.io.WriteFile;
//import org.tensorflow.types.TFloat32;
//import org.tensorflow.types.TString;
//import org.tensorflow.types.TUint8;


/**
 * Loads an image using ReadFile and DecodeJpeg and then uses the saved model
 * faster_rcnn/inception_resnet_v2_1024x1024/1 to detect objects with a detection score greater than 0.3
 * Uses the DrawBounding boxes
 */

public class FasterRcnnInception {
	static int model_width = 224;// for inception5h
	static int model_height = 224;// for inception 5h;
    private final static String[] cocoLabels = new String[]{
            "person",
            "bicycle",
            "car",
            "motorcycle",
            "airplane",
            "bus",
            "train",
            "truck",
            "boat",
            "traffic light",
            "fire hydrant",
            "street sign",
            "stop sign",
            "parking meter",
            "bench",
            "bird",
            "cat",
            "dog",
            "horse",
            "sheep",
            "cow",
            "elephant",
            "bear",
            "zebra",
            "giraffe",
            "hat",
            "backpack",
            "umbrella",
            "shoe",
            "eye glasses",
            "handbag",
            "tie",
            "suitcase",
            "frisbee",
            "skis",
            "snowboard",
            "sports ball",
            "kite",
            "baseball bat",
            "baseball glove",
            "skateboard",
            "surfboard",
            "tennis racket",
            "bottle",
            "plate",
            "wine glass",
            "cup",
            "fork",
            "knife",
            "spoon",
            "bowl",
            "banana",
            "apple",
            "sandwich",
            "orange",
            "broccoli",
            "carrot",
            "hot dog",
            "pizza",
            "donut",
            "cake",
            "chair",
            "couch",
            "potted plant",
            "bed",
            "mirror",
            "dining table",
            "window",
            "desk",
            "toilet",
            "door",
            "tv",
            "laptop",
            "mouse",
            "remote",
            "keyboard",
            "cell phone",
            "microwave",
            "oven",
            "toaster",
            "sink",
            "refrigerator",
            "blender",
            "book",
            "clock",
            "vase",
            "scissors",
            "teddy bear",
            "hair drier",
            "toothbrush",
            "hair brush"
    };

    public static void main(String[] args) {

        if (args.length != 2) {
            throw new IllegalArgumentException("Exactly 2 parameters required !");
        }
        //my output image
        String modelDir = args[0];
        String imageFile = args[1];
        String model_name = "models/faster_rcnn_inception_resnet_v2_1024x1024";
        byte[] graphDef = readAllBytesOrExit(Paths.get(modelDir, model_name));//"tensorflow_inception_graph.pb"));
        List<String> labels = Arrays.asList(cocoLabels);
           // readAllLinesOrExit(Paths.get(modelDir, label_file));//"imagenet_comp_graph_label_strings.txt"));
        byte[] imageBytes = readAllBytesOrExit(Paths.get(imageFile));

        try (Tensor<Float> image = constructAndExecuteGraphToNormalizeImage(imageBytes)) {
        // get path to model folder

        // load saved model
        SavedModelBundle model = SavedModelBundle.load(model_name, "serve");
        //create a map of the COCO 2017 labels
        TreeMap<Float, String> cocoTreeMap = new TreeMap<>();
        
        float cocoCount = 0;
        for (String cocoLabel : cocoLabels) {
            cocoTreeMap.put(cocoCount, cocoLabel);
            cocoCount++;
        }
        try (Graph g = new Graph(); Session s = new Session(g)) {
            Ops tf = Ops.create(g);
            GraphBuilder b = new GraphBuilder(g);
            Session.Runner runner = s.runner();
/*
            try (TUint8 reshapeTensor = (TUint8) s.runner().fetch(reshape).run().get(0)) {
                Map<String, Tensor> feedDict = new HashMap<>();
                //The given SavedModel SignatureDef input
                feedDict.put("input_tensor", reshapeTensor);
                //The given SavedModel MetaGraphDef key
                Map<String, Tensor> outputTensorMap = model.function("serving_default").call(feedDict);
                //detection_classes, detectionBoxes etc. are model output names
                try (TFloat32 detectionClasses = (TFloat32) outputTensorMap.get("detection_classes");
                     TFloat32 detectionBoxes = (TFloat32) outputTensorMap.get("detection_boxes");
                     TFloat32 rawDetectionBoxes = (TFloat32) outputTensorMap.get("raw_detection_boxes");
                     TFloat32 numDetections = (TFloat32) outputTensorMap.get("num_detections");
                     TFloat32 detectionScores = (TFloat32) outputTensorMap.get("detection_scores");
                     TFloat32 rawDetectionScores = (TFloat32) outputTensorMap.get("raw_detection_scores");
                     TFloat32 detectionAnchorIndices = (TFloat32) outputTensorMap.get("detection_anchor_indices");
                     TFloat32 detectionMulticlassScores = (TFloat32) outputTensorMap.get("detection_multiclass_scores")) {
                    int numDetects = (int) numDetections.getFloat(0);
                    if (numDetects > 0) {
                        ArrayList<FloatNdArray> boxArray = new ArrayList<>();
                        //TODO tf.image.combinedNonMaxSuppression
                        for (int n = 0; n < numDetects; n++) {
                            //put probability and position in outputMap
                            float detectionScore = detectionScores.getFloat(0, n);
                            //only include those classes with detection score greater than 0.3f
                            if (detectionScore > 0.3f) {
                                boxArray.add(detectionBoxes.get(0, n));
                            }
                        }
                        //2-D. A list of RGBA colors to cycle through for the boxes.
                        Operand<TFloat32> colors = tf.constant(new float[][]{
                                {0.9f, 0.3f, 0.3f, 0.0f},
                                {0.3f, 0.3f, 0.9f, 0.0f},
                                {0.3f, 0.9f, 0.3f, 0.0f}
                        });
                        Shape boxesShape = Shape.of(1, boxArray.size(), 4);
                        int boxCount = 0;
                        //3-D with shape `[batch, num_bounding_boxes, 4]` containing bounding boxes
                        try (TFloat32 boxes = TFloat32.tensorOf(boxesShape)) {
                            //batch size of 1
                            boxes.setFloat(1, 0, 0, 0);
                            for (FloatNdArray floatNdArray : boxArray) {
                                boxes.set(floatNdArray, 0, boxCount);
                                boxCount++;
                            }
                            //Placeholders for boxes and path to outputimage
                            Placeholder<TFloat32> boxesPlaceHolder = tf.placeholder(TFloat32.class, Placeholder.shape(boxesShape));
                            Placeholder<TString> outImagePathPlaceholder = tf.placeholder(TString.class);
                            //Create JPEG from the Tensor with quality of 100%
                            EncodeJpeg.Options jpgOptions = EncodeJpeg.quality(100L);
                            //convert the 4D input image to normalised 0.0f - 1.0f
                            //Draw bounding boxes using boxes tensor and list of colors
                            //multiply by 255 then reshape and recast to TUint8 3D tensor
                            WriteFile writeFile = tf.io.writeFile(outImagePathPlaceholder,
                                    tf.image.encodeJpeg(
                                            tf.dtypes.cast(tf.reshape(
                                                    tf.math.mul(
                                                            tf.image.drawBoundingBoxes(tf.math.div(
                                                                    tf.dtypes.cast(tf.constant(reshapeTensor),
                                                                            TFloat32.class),
                                                                    tf.constant(255.0f)
                                                                    ),
                                                                    boxesPlaceHolder, colors),
                                                            tf.constant(255.0f)
                                                    ),
                                                    tf.array(
                                                            imageShape.asArray()[0],
                                                            imageShape.asArray()[1],
                                                            imageShape.asArray()[2]
                                                    )
                                            ), TUint8.class),
                                            jpgOptions));
                            //output the JPEG to file
                            s.runner().feed(outImagePathPlaceholder, TString.scalarOf(outputImagePath))
                                    .feed(boxesPlaceHolder, boxes)
                                    .addTarget(writeFile).run();
                        }
                    }
                }
            }
            */
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
