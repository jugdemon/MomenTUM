/*******************************************************************************
 * Welcome to the pedestrian simulation framework MomenTUM. 
 * This file belongs to the MomenTUM version 2.0.2.
 * 
 * This software was developed under the lead of Dr. Peter M. Kielar at the
 * Chair of Computational Modeling and Simulation at the Technical University Munich.
 * 
 * All rights reserved. Copyright (C) 2017.
 * 
 * Contact: peter.kielar@tum.de, https://www.cms.bgu.tum.de/en/
 * 
 * Permission is hereby granted, free of charge, to use and/or copy this software
 * for non-commercial research and education purposes if the authors of this
 * software and their research papers are properly cited.
 * For citation information visit:
 * https://www.cms.bgu.tum.de/en/31-forschung/projekte/456-momentum
 * 
 * However, further rights are not granted.
 * If you need another license or specific rights, contact us!
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

package tum.cms.sim.momentum.utility.neuralNetwork;

/**
 * This factory is used to created neural networks and tensors based on Tensorflow.
 * The wrapper approach reduces dependencies by providing classes that use the Tensorflow api. 
 * 
 * If a developer needs another mechanism from tensorflow he/she have to implement the code in
 * the neuralNetwork package and provide that via the factory.
 * 
 * See {@link tum.cms.sim.momentum.utility.probability.distrubution.DistributionFactory} for a similar approach.
 *  
 * @author Peter M. Kielar
 *
 */
public class NeuralNetworkFactory {
	
	/**
	 * Private constructor to avoid new objects of {@link NeuralNetworkFactory}.
	 */
	private NeuralNetworkFactory() { }
	
	/**
	 * This method creates a new NeuralNetwork object based on the a previously saved
	 * Tensorflow session. The model have to be stored via the saved_model.builder
	 * api of tensorflow using the tag SERVING.
	 * 
	 * @param pathToSavedNetworkFolder, a path that points to a folder where the model is.
	 * @return {@link NeuralNetwork}
	 */
	public static NeuralNetwork createNeuralNetwork(String pathToSavedNetworkFolder) {
		
		return new NeuralNetwork(pathToSavedNetworkFolder);
	}
	
	/**
	 * Create a tensor object.
	 * 
	 * In order to use the tensor in a NeuralNetwork object:
	 * 
	 * The name of the tensor have to be the name of the corresponding tensor object
	 * in the graph that has been loaded.
	 * 
	 * The dimension has to match the dimension of the tensor in the graph that
	 * was restored.
	 * 
	 * @param name
	 * @param dimension
	 * @return {@link NeuralTensor}
	 */
	public static NeuralTensor createNeuralTensor(String name, long[] dimension) {
		
		return new NeuralTensor(name, dimension);
	}
}
