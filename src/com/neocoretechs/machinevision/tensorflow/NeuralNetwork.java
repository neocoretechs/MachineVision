package com.neocoretechs.machinevision.tensorflow;

import org.tensorflow.Tensors;
import org.tensorflow.Graph;
import org.tensorflow.GraphOperation;
import org.tensorflow.GraphOperationBuilder;
import org.tensorflow.Operation;
import org.tensorflow.Output;
import org.tensorflow.op.Op;
import org.tensorflow.Session;
import org.tensorflow.Tensor;
import org.tensorflow.TensorFlow;
import org.tensorflow.DataType;
import java.io.IOException;
import java.nio.FloatBuffer;
import java.util.Arrays;
import java.util.List;

public class NeuralNetwork {

    private Graph graph;
    //private GraphOperationBuilder graphBuilder;
    private Session session;
    private Tensor<?> input;
    private Output<?> output1;
    private GraphOperation graphOpIn;
    private GraphOperation graphOpHidden;
    private GraphOperation graphOpOutput;

    public NeuralNetwork() throws IOException {
      	graph = new Graph();
    }

    public Graph getGraph() {
    	return graph;
    }
    
    public Object[] run(float[] inputData) throws IOException {
    	
        FloatBuffer inputBuffer = FloatBuffer.wrap(inputData);
        // batch size 1, length elements
        input = Tensor.create(new long[]{1, inputData.length}, inputBuffer);
        
        GraphBuilder b = new GraphBuilder(graph);
            // Construct the computation graph with a single operation, a constant
            // named "MyConst" with a value "value".
            //try (Tensor t = Tensor.create(value.getBytes("UTF-8"))) {
              // The Java API doesn't yet include convenience functions for adding operations.
             // graph.opBuilder("Const", "MyConst").setAttr("dtype", t.dataType()).setAttr("value", t).build();
            //}

            // Execute the "MyConst" operation in a Session.
            //try (Session s = new Session(graph);
                // Generally, there may be multiple output tensors,
                // all of them must be closed to prevent resource leaks.
               // Tensor output = s.runner().fetch("MyConst").run().get(0)) {
              //System.out.println(new String(output.bytesValue(), "UTF-8"));
            //}
        output1 = b.constant("input",input);
        
        System.out.println(output1+" output from input layer");
        
        // Create hidden layer with ReLU activation
        
        graphOpHidden = graph.opBuilder("Const", "hidden")
                .addInput(output1)
                .setAttr("units", 300) // 300 nodes in hidden layer
                .setAttr("activation", "RELU")
                .build();
        
        System.out.println(graphOpHidden.numOutputs()+" outputs from hidden layer");
        // Create output layer with softmax activation

        graphOpOutput = graph.opBuilder("Const", "output")
                .addInput(graphOpHidden.output(0))
                .setAttr("units", 6) // 6 classification categories
                .setAttr("activation", "SOFTMAX")
                .build();
        
        System.out.println(graphOpOutput.numOutputs()+" outputs from output layer");
        // Create a session 
        // Update the input tensor with the new data

        // Update the session runner with the new input
        session = new Session(graph);
        List<Tensor<?>> result = session.runner()
                .feed("input", input)
                .fetch("output")
                .run();

        // Get the output
        return result.toArray();
    }

    public static void main(String[] args) throws IOException {

        NeuralNetwork neuralNetwork = new NeuralNetwork();

        // Run with new input data

        float[] inputData = new float[16000]; // initialize input data

        // return array of tensors
        Object[] output = neuralNetwork.run(inputData);

        // Print the output

        System.out.println(Arrays.toString(output));

    }

}