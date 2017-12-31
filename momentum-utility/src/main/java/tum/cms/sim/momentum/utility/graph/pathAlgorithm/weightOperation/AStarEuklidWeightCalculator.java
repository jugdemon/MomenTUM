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

package tum.cms.sim.momentum.utility.graph.pathAlgorithm.weightOperation;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;

import tum.cms.sim.momentum.utility.geometry.Vector2D;
import tum.cms.sim.momentum.utility.graph.Graph;
import tum.cms.sim.momentum.utility.graph.Vertex;

public class AStarEuklidWeightCalculator extends DirectWeightCalculatur {

		protected String weightName = null;
		protected String weightNameExtension = null;
		
		private boolean cognitive = false;
		private HashMap<Integer, Vector2D> vertexMap = null;
		
		protected Comparator<Vertex> weightComparator = new Comparator<Vertex>() {

			@Override
			public int compare(Vertex o1, Vertex o2) {
				
				return Double.compare(o2.getWeight(AStarEuklidWeightCalculator.this.getVertexWeightName()), 
						o1.getWeight(AStarEuklidWeightCalculator.this.getVertexWeightName()));
			}
		};
		
		protected String getVertexWeightName() {
			return weightName + weightNameExtension;
		}
		
		public AStarEuklidWeightCalculator(String weightName, String weightNameExtension) {

			this.weightName = weightName == null ? "" : weightName;
			this.weightNameExtension = weightNameExtension == null ? "" : weightNameExtension;
		}
		
		public AStarEuklidWeightCalculator(String weightName, String weightNameExtension, boolean cognitive) {

			this.weightName = weightName == null ? "" : weightName;
			this.weightNameExtension = weightNameExtension == null ? "" : weightNameExtension;
			this.cognitive = cognitive;
		}
		
		public void SetVertexDistortion(HashMap<Integer, Vector2D> vertexMap) {
			this.vertexMap = vertexMap;
		}

		@Override
		public double calculateWeight(Graph graph, Vertex previousVisit, Vertex target, Vertex current, Vertex successor) {

			if (cognitive) {
				Vector2D curPos = vertexMap.get(current.getId());
				Vector2D sucPos = vertexMap.get(successor.getId());
				Vector2D tarPos = vertexMap.get(target.getId());
				double euklidianDistance = curPos.distance(sucPos);
			    double beelineDistance = sucPos.distance(tarPos);
			    double currentWeight = current.getWeight(this.getVertexWeightName());
			   
			    return currentWeight + euklidianDistance + beelineDistance;
			} else {
				double euklidianDistance = current.euklidDistanceBetweenVertex(successor);
			    double beelineDistance = successor.euklidDistanceBetweenVertex(target);
			    double currentWeight = current.getWeight(this.getVertexWeightName());
			   
			    return currentWeight + euklidianDistance + beelineDistance;
			}
	    }
		
		@Override
		public void updateWeight(Graph graph, double calculatedWeight, Vertex previousVisit, Vertex target, Vertex current, Vertex successor) {
			double euklidianDistance = 0;
			if (cognitive) {
				euklidianDistance += vertexMap.get(current.getId()).distance(vertexMap.get(successor.getId()));
			} else {
				euklidianDistance += current.euklidDistanceBetweenVertex(successor);
			}
		    double currentWeight = current.getWeight(this.getVertexWeightName());
			successor.setWeight(this.getVertexWeightName(), euklidianDistance + currentWeight);
		}
		
		@Override
		public void initializeWeightsForStart(Graph graph, Vertex startVertex) {

	    	graph.getVertex(startVertex.getId()).setWeight(this.getVertexWeightName(), 0.0);
		}
		
		public void initalizeWeights(Graph graph) {

			graph.getVertices().stream().forEach(vertex -> vertex.setWeight(this.getVertexWeightName(), Double.MAX_VALUE));	
		}

		public void removeWeights(Graph graph) {
			
			graph.getVertices().stream().forEach(vertex -> vertex.removeWeight(this.getVertexWeightName()));	
		}
		
		@Override
		public boolean compareWeight(Vertex successor, double weight) {
		
			return successor.getWeight(this.getVertexWeightName()) > weight;
		}
		
		@Override
		public ArrayList<Vertex> sort(ArrayList<Vertex> candidates) {
			
			candidates.sort(weightComparator);
			return candidates;
		}
}
