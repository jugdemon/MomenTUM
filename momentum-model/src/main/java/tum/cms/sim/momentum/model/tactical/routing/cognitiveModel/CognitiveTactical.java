package tum.cms.sim.momentum.model.tactical.routing.cognitiveModel;

import java.util.Collection;

import tum.cms.sim.momentum.data.agent.pedestrian.types.IPedestrianExtension;
import tum.cms.sim.momentum.data.agent.pedestrian.types.IRichPedestrian;
import tum.cms.sim.momentum.data.agent.pedestrian.types.ITacticalPedestrian;
import tum.cms.sim.momentum.infrastructure.execute.SimulationState;
import tum.cms.sim.momentum.model.tactical.routing.RoutingModel;

public class CognitiveTactical extends RoutingModel {

	@Override
	public IPedestrianExtension onPedestrianGeneration(IRichPedestrian pedestrian) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void onPedestrianRemoval(IRichPedestrian pedestrian) {
		// TODO Auto-generated method stub

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
		// TODO Auto-generated method stub

	}

	@Override
	public void callPreProcessing(SimulationState simulationState) {
		// TODO Auto-generated method stub

	}

	@Override
	public void callPostProcessing(SimulationState simulationState) {
		// TODO Auto-generated method stub

	}

}
