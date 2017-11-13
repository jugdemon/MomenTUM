package tum.cms.sim.momentum.model.tactical.routing.cognitiveModel;

import java.util.Collection;

import tum.cms.sim.momentum.data.agent.pedestrian.state.tactical.RoutingState;
import tum.cms.sim.momentum.data.agent.pedestrian.types.IPedestrianExtension;
import tum.cms.sim.momentum.data.agent.pedestrian.types.IRichPedestrian;
import tum.cms.sim.momentum.data.agent.pedestrian.types.ITacticalPedestrian;
import tum.cms.sim.momentum.infrastructure.execute.SimulationState;
import tum.cms.sim.momentum.model.tactical.routing.RoutingModel;
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

		this.initializeEdgeWeights(this.visibilityGraph, CognitiveTactical.weightName);

	}
	
	@Override
	public IPedestrianExtension onPedestrianGeneration(IRichPedestrian pedestrian) {

		return new CognitiveExtension(CognitiveTactical.weightName, Integer.toString(pedestrian.getId()));
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
	
	private void initializeEdgeWeights(Graph graph, String edgeWeightName) {

		Edge edge = null;
		
		for(Vertex current : graph.getVertices()) {  
			 
			for(Vertex successor : graph.getSuccessorVertices(current)) {
				
				if(current == successor) {
					
					continue;
				}
				
				edge = graph.getEdge(current, successor);
				edge.setWeight(edgeWeightName, current.euklidDistanceBetweenVertex(successor));
			}
		}
	}

}
