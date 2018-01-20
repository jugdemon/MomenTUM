package tum.cms.sim.momentum.model.tactical.routing.cognitiveModel;

import java.util.HashMap;
import java.util.concurrent.ThreadLocalRandom;

import tum.cms.sim.momentum.data.agent.pedestrian.types.IPedestrianExtension;
import tum.cms.sim.momentum.utility.geometry.Vector2D;
import tum.cms.sim.momentum.utility.graph.Graph;
import tum.cms.sim.momentum.utility.graph.Path;
import tum.cms.sim.momentum.utility.graph.Vertex;
import tum.cms.sim.momentum.utility.graph.pathAlgorithm.ShortestPathAlgorithm;
import tum.cms.sim.momentum.utility.graph.pathAlgorithm.weightOperation.AStarEuklidWeightCalculator;

public class CognitiveExtension implements IPedestrianExtension {

	private ShortestPathAlgorithm shortestPathAlgorithm = null;
	private AStarEuklidWeightCalculator calculator = null;
	private String vertexWeightName = null;
    private HashMap<Integer, Vector2D> vertexMap = null; 
    private double familiarity;
    private double concentration;
    private double[] vonMisesDist = null;
	private double upperBound;
	private double lowerBound;
	private int vonMisesDistSize = 720;
	
	public CognitiveExtension(String weightNameEdge, String weightForPedestrian, double familiarity, double concentration) {
		
		this.vertexWeightName = weightNameEdge + weightForPedestrian;
		this.calculator = new AStarEuklidWeightCalculator(weightNameEdge, weightForPedestrian,true);
		this.shortestPathAlgorithm = new ShortestPathAlgorithm(this.calculator);
		this.familiarity = familiarity;
		this.concentration = concentration;
	}
	
	public void removeWeightsForPedestrian(Graph graph) {
		
		graph.getVertices().stream().forEach(vertex -> vertex.removeWeight(this.vertexWeightName));	
	}
	
	public Path route(Graph graph, Vertex start, Vertex target) {

		graph.getVertices().stream().forEach(vertex -> vertex.setWeight(this.vertexWeightName, Double.MAX_VALUE));	
		
		return this.shortestPathAlgorithm.calculateShortestPath(graph, start, target);
	}
	
	public void UpdateCognitiveDistortion(HashMap<Integer, Vector2D> vertexMap) {
		this.vertexMap = vertexMap;
		this.calculator.SetVertexDistortion(vertexMap);
	}
	
	public HashMap<Integer, Vector2D> GetCognitiveDistortion(){
		return this.vertexMap;
	}
	
	public double getCognitiveDistanceDistortion() {
		return ThreadLocalRandom.current().nextDouble(0.95 - 0.2*(1-familiarity), 0.95 + 0.2*(1-familiarity));
	}
	
	/**
	 * @return distortion from the correct direction in radians.
	 * 
	 *
	 */
	public double getCognitiveDirectionDistortion() {
		if (vonMisesDist == null ) {
			vonMisesDist = cdf();
			lowerBound = vonMisesDist[0];
			upperBound = vonMisesDist[vonMisesDistSize-1];
		}
		double rand = ThreadLocalRandom.current().nextDouble(0.0, upperBound);
		if (rand < lowerBound) {
			return -Math.PI;
		}
		for (int i = 0; i < vonMisesDistSize-1;i++) {
			if (vonMisesDist[i]< rand && vonMisesDist[i+1]> rand) {
				return 2*Math.PI*((double)i/(double)vonMisesDistSize)-Math.PI;
			}
		}
		return Math.PI;
	}
	
	private double[] cdf() {
		double[] uniformRep = new double[vonMisesDistSize];
		uniformRep[0]= vonMisesDistribution(-Math.PI);
		for (int i = 1; i < uniformRep.length; i++) {
			uniformRep[i]= uniformRep[i-1] + vonMisesDistribution(2*Math.PI*((double)i/(double)vonMisesDistSize)-Math.PI);
		}
		return uniformRep;		
	}
	
	private double vonMisesDistribution(double x) {
		return Math.exp(concentration * Math.cos(x))/(2*Math.PI*bessI0(concentration));
	}

	//Found at https://stackoverflow.com/questions/1193061/bessel-library-function-in-java
	//Ported from https://www.astro.rug.nl/~gipsy/sub/bessel.c
	/*------------------------------------------------------------*/
	/* PURPOSE: Evaluate modified Bessel function In(x) and n=0.  */
	/*------------------------------------------------------------*/
	private double bessI0( double x )
	{
	   double ax,ans;
	   double y;


	   if ((ax=Math.abs(x)) < 3.75) {
	      y=x/3.75;
	      y=y*y;
	      ans=1.0+y*(3.5156229+y*(3.0899424+y*(1.2067492
	         +y*(0.2659732+y*(0.360768e-1+y*0.45813e-2)))));
	   } else {
	      y=3.75/ax;
	      ans=(Math.exp(ax)/Math.sqrt(ax))*(0.39894228+y*(0.1328592e-1
	         +y*(0.225319e-2+y*(-0.157565e-2+y*(0.916281e-2
	         +y*(-0.2057706e-1+y*(0.2635537e-1+y*(-0.1647633e-1
	         +y*0.392377e-2))))))));
	   }
	   return ans;
	}
}
