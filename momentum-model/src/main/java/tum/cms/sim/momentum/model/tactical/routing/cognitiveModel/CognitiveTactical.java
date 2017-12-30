package tum.cms.sim.momentum.model.tactical.routing.cognitiveModel;

import java.util.Collection;
import java.util.HashMap;
import java.util.concurrent.ThreadLocalRandom;

import tum.cms.sim.momentum.data.agent.pedestrian.state.tactical.RoutingState;
import tum.cms.sim.momentum.data.agent.pedestrian.types.IPedestrianExtension;
import tum.cms.sim.momentum.data.agent.pedestrian.types.IRichPedestrian;
import tum.cms.sim.momentum.data.agent.pedestrian.types.ITacticalPedestrian;
import tum.cms.sim.momentum.infrastructure.execute.SimulationState;
import tum.cms.sim.momentum.model.tactical.routing.RoutingModel;
import tum.cms.sim.momentum.utility.geometry.Vector2D;
import tum.cms.sim.momentum.utility.graph.Edge;
import tum.cms.sim.momentum.utility.graph.Graph;
import tum.cms.sim.momentum.utility.graph.Path;
import tum.cms.sim.momentum.utility.graph.Vertex;

public class CognitiveTactical extends RoutingModel {

	private static String weightName = "cognitiveWeight";

	private Graph visibilityGraph = null;

	@Override
	public void callPreProcessing(SimulationState simulationState) {
		visibilityGraph = this.scenarioManager.getGraph();
	}
	
	@Override
	public IPedestrianExtension onPedestrianGeneration(IRichPedestrian pedestrian) {

		this.updateEdgeWeights(this.visibilityGraph, CognitiveTactical.weightName, pedestrian);
		return new CognitiveExtension(CognitiveTactical.weightName, Integer.toString(pedestrian.getId()), 1.0);

	}

	@Override
	public void onPedestrianRemoval(IRichPedestrian pedestrian) {
		((CognitiveExtension)pedestrian.getExtensionState(this)).removeWeightsForPedestrian(this.visibilityGraph);
	}

	@Override
	public void callBeforeBehavior(SimulationState simulationState, Collection<IRichPedestrian> pedestrians) {
		// TODO Auto-generated method stub

	}

	@Override
	public void callAfterBehavior(SimulationState simulationState, Collection<IRichPedestrian> pedestrians) {
		// TODO Auto-generated method stub

	}

	@Override
	public void callPedestrianBehavior(ITacticalPedestrian pedestrian, SimulationState simulationState) {
		
		Vertex start = this.findNavigationStartPoint(pedestrian, this.perception, this.scenarioManager);
		Vertex end = this.visibilityGraph.getGeometryVertex(pedestrian.getNextNavigationTarget().getGeometry());

		CognitiveExtension router = (CognitiveExtension)pedestrian.getExtensionState(this);		
		Path route = router.route(this.visibilityGraph, start, end);
		
		RoutingState routingState = this.updateRouteState(this.perception, pedestrian, route);
		pedestrian.setRoutingState(routingState);
	}


	@Override
	public void callPostProcessing(SimulationState simulationState) {
		// TODO Auto-generated method stub

	}
	
	private void updateEdgeWeights(Graph graph, String edgeWeightName, IRichPedestrian pedestrian) {

		CognitiveExtension cogState = (CognitiveExtension)pedestrian.getExtensionState(this);	
		
		HashMap<Integer, Vector2D> vertexMap = new HashMap<Integer, Vector2D>(); 
		Vector2D pedPos = pedestrian.getPosition();

		Edge edge = null;
		for(Vertex current : graph.getVertices()) {  
			
			Vector2D curPos = current.getGeometry().getCenter();
			double realDist = pedPos.distance(curPos);
			
			double cogDistort =  cogState.getCognitiveDistanceDistortion();
			double cogDist = (0.5 + cogDistort * 0.5) * realDist;
			
			
			
			vertexMap.put(current.getId(), curPos);
			 
			for(Vertex successor : graph.getSuccessorVertices(current)) {
				
				if(current == successor) {
					
					continue;
				}
  
				edge = graph.getEdge(current, successor);
				edge.setWeight(edgeWeightName, current.euklidDistanceBetweenVertex(successor));
			}
		}
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
