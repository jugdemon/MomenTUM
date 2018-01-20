package tum.cms.sim.momentum.model.tactical.routing.cognitiveModel;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Collection;
import java.util.HashMap;
import java.util.concurrent.ThreadLocalRandom;

import org.apache.commons.collections4.IterableMap;

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
	
	private int printID = -1;
	private int printCounterMax = 10;
	private int printCounter = printCounterMax;
	private int printsCounted = 0;

	@Override
	public void callPreProcessing(SimulationState simulationState) {
		visibilityGraph = this.scenarioManager.getGraph();
	}
	
	@Override
	public IPedestrianExtension onPedestrianGeneration(IRichPedestrian pedestrian) {
		
		return new CognitiveExtension(CognitiveTactical.weightName, Integer.toString(pedestrian.getId()), 0.9, 30.0);

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
		updateEdgeWeights(this.visibilityGraph, CognitiveTactical.weightName, pedestrian);
		
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
	
	private void updateEdgeWeights(Graph graph, String edgeWeightName, ITacticalPedestrian pedestrian) {

		CognitiveExtension cogState = (CognitiveExtension)pedestrian.getExtensionState(this);	

		HashMap<Integer, Vector2D> vertexMap = new HashMap<Integer, Vector2D>(); 
		HashMap<Integer, Vector2D> oldVertexMap = cogState.GetCognitiveDistortion();
		HashMap<Integer, Vector2D> origVertexMap = new HashMap<Integer, Vector2D>(); 

		Edge edge = null;
		for(Vertex current : graph.getVertices()) {  
			int id = current.getId();
			if (!vertexMap.containsKey(id)) {
				Vector2D newPos = generateDistortion(current.getGeometry().getCenter(),
						pedestrian.getPosition(),
						cogState.getCognitiveDistanceDistortion(),
						cogState.getCognitiveDirectionDistortion());
				
				if (oldVertexMap != null) {
					double percentNew = 0.2;
					Vector2D oldPos = oldVertexMap.get(id);
					vertexMap.put(id, newPos.multiply(percentNew).sum(oldPos.multiply(1.0-percentNew)));
				} else {
					vertexMap.put(id, newPos);
				}
				origVertexMap.put(id, current.getGeometry().getCenter());
			}
			 
			for(Vertex successor : graph.getSuccessorVertices(current)) {
				if(current == successor) {
					
					continue;
				}
				int succId = successor.getId();
				if (!vertexMap.containsKey(succId)) {
					Vector2D newPosSuc = generateDistortion(successor.getGeometry().getCenter(),
							pedestrian.getPosition(),
							cogState.getCognitiveDistanceDistortion(),
							cogState.getCognitiveDirectionDistortion());
					
					if (oldVertexMap != null) {
						double percentNew = 0.2;
						Vector2D oldPosSuc = oldVertexMap.get(succId);
						vertexMap.put(succId, newPosSuc.multiply(percentNew).sum(oldPosSuc.multiply(1.0-percentNew)));
					} else {
						vertexMap.put(succId, newPosSuc);
					}
					origVertexMap.put(succId, successor.getGeometry().getCenter());
				}
  
				edge = graph.getEdge(current, successor);
				Vector2D currentDistorted = vertexMap.get(id);
				Vector2D successorDistorted = vertexMap.get(succId);
				edge.setWeight(edgeWeightName, currentDistorted.distance(successorDistorted));
			}
		}
		cogState.UpdateCognitiveDistortion(vertexMap);
		//if (printID == -1) trySetPrintId(pedestrian.getId());
		//if (printID == pedestrian.getId()) writeToFile(origVertexMap,vertexMap,pedestrian.getId());
	}
	
	private synchronized void trySetPrintId(int id) {
		if (printID == -1 && id != 1) {
			printID = id;
		}
	}
	
	private void writeToFile(HashMap<Integer, Vector2D> origVertexMap, HashMap<Integer, Vector2D>vertexMap, int id) {
		if (printCounter == printCounterMax) {
			java.nio.file.Path path = Paths.get("C:\\Users\\Jascha\\Documents\\ETH\\STP_Master\\Master Thesis\\momentum_data\\output\\distortedGraph"+id+"_"+printsCounted+".csv");
			printsCounted++;
			//Use try-with-resource to get auto-closeable writer instance
			try (BufferedWriter writer = Files.newBufferedWriter(path))
			{
			    for (  int key : vertexMap.keySet()) {
			    	Vector2D oldVec = origVertexMap.get(key);
			    	Vector2D newVec = vertexMap.get(key);
			    	double oldX = oldVec.getXComponent();
			    	double oldY = oldVec.getYComponent();
			    	double newX = newVec.getXComponent();
			    	double newY = newVec.getYComponent();
			    	writer.write(key +","+oldX +","+oldY + "," + newX + "," + newY +"\n");
				}
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			printCounter=0;
		} else {
			printCounter++;
		}
	}
	
	private Vector2D generateDistortion(Vector2D nodePosition, Vector2D pedestrianPosition, double distanceDistortion, double directionDistortion) {
		if (nodePosition.equals(pedestrianPosition)) return nodePosition;
		double realDist = pedestrianPosition.distance(nodePosition);
		double cogDist = Math.pow(realDist,distanceDistortion);
		return nodePosition.
				subtract(pedestrianPosition).
				rotate(directionDistortion).
				multiply(cogDist/realDist).
				sum(pedestrianPosition);
			 
	}
	
}
