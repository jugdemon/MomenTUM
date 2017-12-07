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

package tum.cms.sim.momentum.model.tactical;

import java.util.Collection;
import java.util.Set;

import tum.cms.sim.momentum.configuration.ModelTypConstants.ModelType;
import tum.cms.sim.momentum.data.agent.pedestrian.state.tactical.RoutingState;
import tum.cms.sim.momentum.data.agent.pedestrian.state.tactical.TacticalState;
import tum.cms.sim.momentum.data.agent.pedestrian.state.tactical.TacticalState.Behavior;
import tum.cms.sim.momentum.data.agent.pedestrian.state.tactical.TacticalState.Motoric;
import tum.cms.sim.momentum.data.agent.pedestrian.types.IPedestrianExtension;
import tum.cms.sim.momentum.data.agent.pedestrian.types.IRichPedestrian;
import tum.cms.sim.momentum.infrastructure.execute.SimulationState;
import tum.cms.sim.momentum.infrastructure.logging.LoggingManager;
import tum.cms.sim.momentum.model.MessageStrings;
import tum.cms.sim.momentum.model.PedestrianBehaviorModel;
import tum.cms.sim.momentum.model.tactical.participating.StayingModel;
import tum.cms.sim.momentum.model.tactical.queuing.QueuingModel;
import tum.cms.sim.momentum.model.tactical.routing.RoutingModel;
import tum.cms.sim.momentum.model.tactical.searching.SearchingModel;
import tum.cms.sim.momentum.utility.geometry.Vector2D;
import tum.cms.sim.momentum.utility.graph.Vertex;

public class TacticalModel extends PedestrianBehaviorModel {
	
	protected final static String strategicCommandName = "strategicCommand";
	protected final static String goalDistanceRadiusName = "goalDistanceRadius";
	protected final static String navigationDistanceRadiusName = "navigationDistanceRadius";
	protected final static String tacticalControlName = "tacticalControl";
	protected final static String deepNodeSelectionName = "deepNodeSelection";
	protected final static String routeMemoryName = "routeMemory";
	
	protected Behavior strategicFixedCommand = null;
	protected double goalDistanceRadius = 0.15;
	protected double navigationDistanceRadius = 0.15;
	protected boolean tacticalControl = true;
	protected boolean routeMemory = true;
	protected int deepNodeSelection = 0;

	private RoutingModel routingModel = null;
	
	public RoutingModel getRoutingModel() {
		return routingModel;
	}

	public void setRoutingModel(RoutingModel routingModel) {
		this.routingModel = routingModel;
	}

	private SearchingModel searchingModel = null;
	
	public SearchingModel getSearchingModel() {
		return searchingModel;
	}

	public void setSearchingModel(SearchingModel searchingModel) {
		this.searchingModel = searchingModel;
	}
	
	private QueuingModel queuingModel = null;
	
	public QueuingModel getQueuingModel() {
		return queuingModel;
	}

	public void setQueuingModel(QueuingModel queuingModel) {
		this.queuingModel = queuingModel;
	}
	
	private StayingModel stayingModel = null;

	public StayingModel getStayingModel() {
		return stayingModel;
	}

	public void setParticipatingModel(StayingModel stayingModel) {
		this.stayingModel = stayingModel;
	}
	
	@Override
	public ModelType getModelType() {
		
		return ModelType.Tactical;
	}
	
	@Override
	public IPedestrianExtension onPedestrianGeneration(IRichPedestrian pedestrian) {
		
		return null; 
	}

	@Override
	public void onPedestrianRemoval(IRichPedestrian pedestrian) {
		
		// Nothing to do
	}

	@Override
	public void callPreProcessing(SimulationState simulationState) {
		
		if(this.properties.getStringProperty(strategicCommandName) != null) {
			
			strategicFixedCommand = Behavior.valueOf(this.properties.getStringProperty(strategicCommandName));
		}

		if(this.properties.getDoubleProperty(goalDistanceRadiusName) != null) {
			
			goalDistanceRadius = this.properties.getDoubleProperty(goalDistanceRadiusName);
		}
		
		if(this.properties.getDoubleProperty(navigationDistanceRadiusName) != null) {
			
			navigationDistanceRadius = this.properties.getDoubleProperty(navigationDistanceRadiusName);
		}
		
		if(this.properties.getBooleanProperty(tacticalControlName) != null) {
			
			tacticalControl = this.properties.getBooleanProperty(tacticalControlName);
		}
		
		if(this.properties.getIntegerProperty(deepNodeSelectionName) != null) {
			
			deepNodeSelection = this.properties.getIntegerProperty(deepNodeSelectionName);
		}
		
		if(this.properties.getBooleanProperty(routeMemoryName) != null) {
			
			routeMemory = this.properties.getBooleanProperty(routeMemoryName);
		}
		
		LoggingManager.logDebug(MessageStrings.propertySetTo,
				navigationDistanceRadiusName,
				String.valueOf(navigationDistanceRadius),
				this.getClass().getSimpleName());
		
		LoggingManager.logDebug(MessageStrings.propertySetTo,
				goalDistanceRadiusName,
				String.valueOf(goalDistanceRadius),
				this.getClass().getSimpleName());
		
		LoggingManager.logDebug(MessageStrings.propertySetTo,
				tacticalControlName,
				String.valueOf(tacticalControl),
				this.getClass().getSimpleName());
		
		LoggingManager.logDebug(MessageStrings.propertySetTo,
				deepNodeSelectionName,
				String.valueOf(deepNodeSelection),
				this.getClass().getSimpleName());
		
		LoggingManager.logDebug(MessageStrings.propertySetTo,
				routeMemoryName,
				String.valueOf(routeMemory),
				this.getClass().getSimpleName());
		
		LoggingManager.logDebug(MessageStrings.propertySetTo,
				strategicFixedCommand,
				String.valueOf(strategicFixedCommand),
				this.getClass().getSimpleName());
	}

	@Override
	public void callPostProcessing(SimulationState simulationState) {
		
		// Nothing to do
	}

	@Override
	public void callBeforeBehavior(SimulationState simulationState, Collection<IRichPedestrian> pedestrians) {
		
		pedestrians.parallelStream().forEach(pedestrian -> {		
			
			boolean tacticalControlRouting = false;
			
			// Initial check: is goal visible and does the pedestrian has active behavior?
			// If not, due to some circumstances, start routing to the goal until it can be seen.
			if(this.tacticalControl && this.checkCommandExecutable(pedestrian)) {
				
				// The strategic command cannot be executed -> route until the the walking goal is visible
				// This describes a bottom up control "fighting" the strategic command.
				tacticalControlRouting = true;
				pedestrian.getTacticalState().setOverrideBehaviorTask(Behavior.Routing);
			}

			if(tacticalControlRouting || !this.getBehavior(pedestrian).equals(Behavior.Staying)) {
				
				pedestrian.setStayingState(null);
			}

			if(tacticalControlRouting || !this.getBehavior(pedestrian).equals(Behavior.Queuing)) {
				
				pedestrian.setQueuingState(null);
			}
			
			if(tacticalControlRouting || !this.getBehavior(pedestrian).equals(Behavior.Searching)) {
							
				pedestrian.setSearchingState(null);
			}
			
			if(!tacticalControlRouting && !this.getBehavior(pedestrian).equals(Behavior.Routing)) {
						
				pedestrian.setRoutingState(null);
			}
		});
		
		if(this.stayingModel != null) {
		
			this.stayingModel.callBeforeBehavior(simulationState, pedestrians);
		}
		
		if(this.queuingModel != null) {
			
			this.queuingModel.callBeforeBehavior(simulationState, pedestrians);
		}
		
		if(this.routingModel != null) {
			
			this.routingModel.callBeforeBehavior(simulationState, pedestrians);
		}
		
		if(this.searchingModel != null) {
			
			this.searchingModel.callBeforeBehavior(simulationState, pedestrians);
		}
	}
	
	private boolean checkCommandExecutable(IRichPedestrian pedestrian) { 

		if(this.getBehavior(pedestrian) == null) {
			
			return false;
		}
		
		boolean noCommandGiven = this.getBehavior(pedestrian).equals(Behavior.None);
		
		if(noCommandGiven) {
			
			return false;
		}
		
		boolean routingCommandGiven = this.getBehavior(pedestrian).equals(Behavior.Routing);
		
		if(routingCommandGiven) {
			
			return false;
		}
		
		boolean queuingActive = pedestrian.getQueuingState() != null;
		boolean stayingActive = pedestrian.getStandingState() != null;
		
		if(queuingActive || stayingActive) {
			
			return false;
		}
		
		boolean goalIsVisible = this.isGoalTargetVisible(pedestrian);
		
		return !goalIsVisible; // if not visible the check failed!
	}

	private Behavior getBehavior(IRichPedestrian pedestrian) {
		
		Behavior behavior = Behavior.None;
		
		if(this.strategicFixedCommand != null) {
			
			behavior = this.strategicFixedCommand;
		}
		else {
			
			behavior = pedestrian.getBehaviorTask();
		}
		
		return behavior;
	}
	
	@Override
	public void callPedestrianBehavior(IRichPedestrian pedestrian, SimulationState simulationState) {

		Behavior command = this.getBehavior(pedestrian);
		
		// In case the strategic command cannot be executed, it is overridden with Routing
		// If this is the case, and tacticalControl is activated, execute routing behavior.
		if(this.tacticalControl && 
		   pedestrian.getTacticalState() != null && 
		   pedestrian.getTacticalState().getOverrideBehaviorTask() != null) {
				
			command = pedestrian.getTacticalState().getOverrideBehaviorTask();
		}
	
  		switch(command) {
  		
		case Staying:
			
			this.callTacticStayingBehavior(pedestrian, simulationState);
			break;
			
		case Queuing:
			
			this.callTactiQueuingBehavior(pedestrian, simulationState);
			break;
			
		case Searching:
			
			this.callTacticSearchBehavior(pedestrian, simulationState);
			break;
			
		case Routing:
			
			this.callTacticRouteBehavior(pedestrian, simulationState);
			break;
			
		case None:
		default:
			
			break;
  		}
	}
	
	@Override
	public void callAfterBehavior(SimulationState simulationState, Collection<IRichPedestrian> pedestrians) {
	
		if(this.stayingModel != null) {
			
			this.stayingModel.callAfterBehavior(simulationState, pedestrians);
		}
		
		if(this.queuingModel != null) {
			
			this.queuingModel.callAfterBehavior(simulationState, pedestrians);
		}
		
		if(this.routingModel != null) {
			
			this.routingModel.callAfterBehavior(simulationState, pedestrians);
		}
		
		if(this.searchingModel != null) {
			
			this.searchingModel.callAfterBehavior(simulationState, pedestrians);
		}
		
		//this.pedestrianNearNavigationNode = new HashMap<>();
		
		pedestrians.parallelStream().forEach(pedestrian -> {			
	
			boolean isSmallScaleBehavior = false;
			Vector2D nextWalkingTarget = pedestrian.getNextWalkingTarget();
			
			if(pedestrian.getStayingState() != null ||
			   pedestrian.getQueuingState() != null) {
				
				isSmallScaleBehavior = true;
			}
			else if(pedestrian.getRoutingState() != null ||
					pedestrian.getSearchingState() != null) {
			
				isSmallScaleBehavior = false;
			}
			else { // no target, nothing to do

				isSmallScaleBehavior = true;
			}
			
			Motoric motoricTask = null;

			if(isSmallScaleBehavior && (pedestrian.getStandingState() != null
					|| this.isGoalPositionReached(nextWalkingTarget, pedestrian.getPosition()))) {

				motoricTask = Motoric.Standing;		
			}
			else {
				
				motoricTask = Motoric.Walking;
			}
			
			pedestrian.setTacticalState(new TacticalState(motoricTask));
		});
	}

	/**
	 * Checks if the pedestrian can see the goal area or the point of interest.
	 * @param pedestrian which for checking
	 * @return true if visible, otherwise false
	 */
	private boolean isGoalTargetVisible(IRichPedestrian pedestrian) {
		
		return perception.isVisible(pedestrian, pedestrian.getNextNavigationTarget().getPointOfInterest());
	}
	
	/**
	 * Calculates if the pedestrian is close to the given position
	 * @param pedestrian which for checking
	 * @return true if visible, otherwise false
	 */	
	private boolean isGoalPositionReached(Vector2D pedestrianPosition, Vector2D targetPosition) {
		
		return targetPosition != null && targetPosition.distance(pedestrianPosition) <= this.goalDistanceRadius;
	}
	
	private void callTacticRouteBehavior(IRichPedestrian pedestrian, SimulationState simulationState) {
			
		if(tacticalControl) {
			
			// correct goal / point of interest is visible, just go there!
			boolean normalRouting  = !this.routingModel.shortCutRoute(this.perception, pedestrian);
		
			// re-routing is a process that only needs to be activated if
			// the next navigation is visible 
			// the current navigation node is not visible!
			if(normalRouting && this.routingModel.reRoutingNecessary(pedestrian, this.tacticalControl, this.deepNodeSelection > 0)) {
				
				this.routingModel.callPedestrianBehavior(pedestrian, simulationState);
				
				// TODO create an abstract method in RoutingModel put this into
				if(this.deepNodeSelection > 0) {
					
					int currentDepth = this.deepNodeSelection;
					
					Vertex nextTolast = pedestrian.getRoutingState().getLastVisit();
					Vertex last = pedestrian.getRoutingState().getLastVisit();
					Vertex start = last;
					Vertex next = pedestrian.getRoutingState().getNextVisit();
					Vertex nextToNext = pedestrian.getRoutingState().getNextToCurrentVisit();
					
					Set<Vertex> visited = pedestrian.getRoutingState().getVisited();
					Vertex end = null;
					
					if(pedestrian.getNextNavigationTarget() != null) {
						end = scenarioManager.getGraph().getGeometryVertex(pedestrian.getNextNavigationTarget().getGeometry());
					}
					
					while(currentDepth > 0) {
						
						this.routingModel.callPedestrianBehavior(pedestrian, simulationState);
					
						RoutingState newRoutingState = pedestrian.getRoutingState();
						
						// TODO check if circles are solved next / last
						if(!perception.isVisible(pedestrian, newRoutingState.getNextVisit()) ||
						   (end != null && newRoutingState.getNextVisit().equals(end)) ||
						   (start != null && end != null && start.equals(end))) {
							
							nextToNext = newRoutingState.getNextVisit();
							break;
						}

						currentDepth--;
						
						if(currentDepth == 0) { 
							
							nextToNext = newRoutingState.getNextVisit();
							break;
						}
						else {
							
							nextTolast = last;
						}
						
						visited.add(next);
						
						last = newRoutingState.getLastVisit();
						next = newRoutingState.getNextVisit();
					}
					
					RoutingState finalRoutingState = new RoutingState(visited, nextTolast, last, next);
					finalRoutingState.setNextToCurrentVisit(nextToNext);
					pedestrian.setRoutingState(finalRoutingState);
				}
			}			
		}
		else if(pedestrian.getRoutingState() == null || 
			   pedestrian.getRoutingState().getNextVisit() == null ||
			   pedestrian.getPosition().distance(pedestrian.getRoutingState().getNextVisit().getGeometry().getCenter()) < navigationDistanceRadius) {
			
			this.routingModel.callPedestrianBehavior(pedestrian, simulationState);
		}
		
		// Is the route memory activated, if not delete it
		if(!routeMemory && pedestrian.getRoutingState() != null) {
			
			pedestrian.getRoutingState().getVisited().clear();
		}
	}

	private void callTacticSearchBehavior(IRichPedestrian pedestrian, SimulationState simulationState) {
		
		this.searchingModel.callPedestrianBehavior(pedestrian, simulationState);
	}
	
	private void callTacticStayingBehavior(IRichPedestrian pedestrian, SimulationState simulationState) {
		
		this.stayingModel.callPedestrianBehavior(pedestrian, simulationState);
	}
	
	private void callTactiQueuingBehavior(IRichPedestrian pedestrian, SimulationState simulationState) {
		
		this.queuingModel.callPedestrianBehavior(pedestrian, simulationState);
	}
	

//	pedestrians.stream().forEach(pedestrian -> {	
//
//		// If the navigation node reached calculation is dynamic
//		// count the number of pedestrian for each vertex here
//		if(dynamicNodeReached) {
//			
//			if(pedestrian.getRoutingState() != null &&
//			   pedestrian.getRoutingState().getNextVisit() != null &&
//			   pedestrian.getPosition().distance(pedestrian.getRoutingState().getNextVisit().getGeometry().getCenter()) < dynamicNodeDistance) {
//				
//				if(!this.pedestrianNearNavigationNode.containsKey(pedestrian.getRoutingState().getNextVisit())) {
//				
//					this.pedestrianNearNavigationNode.put(pedestrian.getRoutingState().getNextVisit(), 1);
//				}
//				else {
//					
//					int currentNumber = this.pedestrianNearNavigationNode.get(pedestrian.getRoutingState().getNextVisit());
//					this.pedestrianNearNavigationNode.put(pedestrian.getRoutingState().getNextVisit(), currentNumber + 1);
//				}
//			}
//		}
//	});
//	this.getRoutingModel().set
//	if(dynamicNodeReached) { // Update Walking position based on the crowdness at the Vertex
//		
//		if(pedestrian.getRoutingState() != null &&
//		   pedestrian.getRoutingState().getNextVisit() != null &&
//		   pedestrian.getRoutingState().getLastVisit() != null) {
//		   //pedestrian.getPosition().distance(pedestrian.getRoutingState().getNextVisit().getGeometry().getCenter()) < dynamicNodeDistance) {
//
//			// compute next vertex geometric structure
//			pedestrian.getRoutingState().setNextVertexGeoemetry(
//				this.createVisitingTriangle(
//					pedestrian.getRoutingState().getNextVisit().getGeometry().getCenter(),
//					pedestrian.getPosition(),
//					pedestrian.getRoutingState().getLastVisit().getGeometry().getCenter(),
//					this.computeVertexExpansion(pedestrian)));
//		}
//	}
	
//	if(this.isNavigationPointReached(pedestrian)) {
//	
//	// Is the next walking target after the current walking target visible?
//	if(pedestrian.getRoutingState().getNextToNextVisit() == null ||
//	   perception.isVisible(pedestrian.getPosition(), pedestrian.getRoutingState().getNextToNextVisit())) {
//		
//		// If the next walking target (after the current) is visible after point reached, reroute
//		return true; 
//	}
//	else {
//		
//		// the next walking target is not visible after point reached? wait with reroute.
//		// move closer
//		return false;
//	}
//}
//}
//else if(this.isNavigationPointReached(pedestrian)) {
//
//return true;
//}

//	private double computeVertexExpansion(IRichPedestrian pedestrian) {
//		
//		Integer numberOfVisitorsToNode = this.pedestrianNearNavigationNode.get(pedestrian.getRoutingState().getNextVisit());
//		
//			if(numberOfVisitorsToNode != null && numberOfVisitorsToNode > 1) {
//			
//			double navigationSizePolygon = numberOfVisitorsToNode.intValue() * navigationDistanceRadius;
//			 
//			Polygon2D visitTriangle = this.createVisitingTriangle(
//					 pedestrian.getRoutingState().getNextVisit().getGeometry().getCenter(),
//					 pedestrian.getPosition(),
//					 pedestrian.getRoutingState().getLastVisit().getGeometry().getCenter(),
//					 navigationSizePolygon);
//			
//			nextWalkingTarget = visitTriangle.getPointClosestToVector(pedestrian.getPosition());
//		}
//	}
	
//	private Polygon2D createVisitingTriangle(Vector2D nextVisitPosition,
//			Vector2D pedestrianPosition,
//			Vector2D lastVisitPosition,
//			Double navigationSizePolygon) {
//		
//		double rotation = 90.0;
//
//		Vector2D fromPreviousToCurrent = nextVisitPosition.subtract(lastVisitPosition); 
//
//		ArrayList<Vector2D> triangleCorners = new ArrayList<Vector2D>();
//	
//		triangleCorners.add(nextVisitPosition.sum(
//				fromPreviousToCurrent.scale(navigationSizePolygon)
//				.rotate(GeometryAdditionals.translateToRadiant(-1.0 * rotation))));
//		
//		triangleCorners.add(nextVisitPosition.sum(fromPreviousToCurrent.scale(navigationSizePolygon)));
//	
//		triangleCorners.add(nextVisitPosition.sum(
//				fromPreviousToCurrent.scale(navigationSizePolygon)
//					.rotate(GeometryAdditionals.translateToRadiant(rotation))));
//		
//		return GeometryFactory.createPolygon(triangleCorners);
//	}
	
//	/**
//	 * Calculates if a pedestrian did reached the navigation point for routing.
//	 * If the pedestrian is at the navigation point, the vertex will be set as
//	 * visited.
//	 * If the tactical parameter dynamicNodeReached true, the computation will
//	 * change and increase the visited area with the number of pedestrians approaching
//	 * the node.
//	 */
//	private boolean isNavigationPointReached(ITacticalPedestrian pedestrian) {
//		
//		boolean hasReached = false;
//
//		double distance = pedestrian.getRoutingState().getNextVisit().getGeometry().getCenter().distance(
//				pedestrian.getPosition());
//		boolean inside = pedestrian.getRoutingState().getNextVisit().getGeometry().contains(pedestrian.getPosition());
//		
//		double navigationSizePolygon = 3 * navigationDistanceRadius;
//		
////		if(dynamicNodeReached && pedestrian.getRoutingState().getNextVisit() != null) {
////			
////			Integer numberOfVisitorsToNode = this.pedestrianNearNavigationNode.get(pedestrian.getRoutingState().getNextVisit());
////			
////			if(numberOfVisitorsToNode != null) {
////				
////				 navigationSizePolygon = numberOfVisitorsToNode.intValue() * navigationDistanceRadius;
////			}
////		}
//		
//		if(inside || distance < pedestrian.getBodyRadius() || (tacticalControl && distance < navigationSizePolygon)) {
//					
//			hasReached = true;	
//		}
//		else if(tacticalControl && pedestrian.getRoutingState().getLastVisit() != null) {
//
//			Polygon2D visitTriangle = this.createVisitingTriangle(
//					 pedestrian.getRoutingState().getNextVisit().getGeometry().getCenter(),
//					 pedestrian.getPosition(),
//					 pedestrian.getRoutingState().getLastVisit().getGeometry().getCenter(),
//					 navigationSizePolygon);
//			
//			if(visitTriangle.contains(pedestrian.getPosition())) {
//
//				hasReached = true;
//			}
//		}
//		else if(distance < navigationDistanceRadius) {
//				
//			hasReached = true;
//		}
//
//		return hasReached;
//	}
	
}
