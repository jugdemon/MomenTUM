package tum.cms.sim.momentum.model.tactical.routing.cognitiveModel;

import tum.cms.sim.momentum.data.agent.pedestrian.types.IPedestrianExtension;
import tum.cms.sim.momentum.utility.graph.Graph;
import tum.cms.sim.momentum.utility.graph.Path;
import tum.cms.sim.momentum.utility.graph.Vertex;
import tum.cms.sim.momentum.utility.graph.pathAlgorithm.ShortestPathAlgorithm;
import tum.cms.sim.momentum.utility.graph.pathAlgorithm.weightOperation.AStarEuklidWeightCalculator;

public class CognitiveExtension implements IPedestrianExtension {

	private ShortestPathAlgorithm shortestPathAlgorithm = null;
	private AStarEuklidWeightCalculator calculator = null;
	private String vertexWeightName = null;
	
	public CognitiveExtension(String weightNameEdge, String weightForPedestrian) {
		
		this.vertexWeightName = weightNameEdge + weightForPedestrian;
		this.calculator = new AStarEuklidWeightCalculator(weightNameEdge, weightForPedestrian);
		this.shortestPathAlgorithm = new ShortestPathAlgorithm(this.calculator);
	}
	
	public void removeWeightsForPedestrian(Graph graph) {
		
		graph.getVertices().stream().forEach(vertex -> vertex.removeWeight(this.vertexWeightName));	
	}
	
	public Path route(Graph graph, Vertex start, Vertex target) {

		graph.getVertices().stream().forEach(vertex -> vertex.setWeight(this.vertexWeightName, Double.MAX_VALUE));	
		
		return this.shortestPathAlgorithm.calculateShortestPath(graph, start, target);
	}
}
