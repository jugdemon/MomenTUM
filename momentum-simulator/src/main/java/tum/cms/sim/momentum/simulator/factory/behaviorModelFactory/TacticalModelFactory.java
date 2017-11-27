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

package tum.cms.sim.momentum.simulator.factory.behaviorModelFactory;

import tum.cms.sim.momentum.configuration.model.tactical.TacticalModelConfiguration;
import tum.cms.sim.momentum.model.perceptional.PerceptionalModel;
import tum.cms.sim.momentum.model.tactical.TacticalModel;
import tum.cms.sim.momentum.model.tactical.participating.StayingModel;
import tum.cms.sim.momentum.model.tactical.queuing.QueuingModel;
import tum.cms.sim.momentum.model.tactical.routing.RoutingModel;
import tum.cms.sim.momentum.model.tactical.searching.SearchingModel;
import tum.cms.sim.momentum.simulator.component.ComponentManager;
import tum.cms.sim.momentum.simulator.factory.ModelFactory;
import tum.cms.sim.momentum.utility.generic.PropertyBackPackFactory;
import tum.cms.sim.momentum.utility.generic.Unique;

public class TacticalModelFactory extends ModelFactory<TacticalModelConfiguration, TacticalModel>{

	@Override
	public TacticalModel createModel(TacticalModelConfiguration configuration, ComponentManager componentManager) {
		
		TacticalModel tacticalModel = new TacticalModel();
		
		Unique.generateUnique(tacticalModel, configuration);
		tacticalModel.setPropertyBackPack(PropertyBackPackFactory.fillProperties(configuration));
		tacticalModel.setExeuctionId(tacticalModel.getId());
		
		PerceptionalModel perceptualModel = componentManager.getPerceptionalModel(configuration.getPerceptualModel());
		
		this.fillComposition(tacticalModel, perceptualModel, componentManager);
		
		if(configuration.getStayingReference() != null) {
			
			StayingModel participatingModel = componentManager.getStayingModel(configuration.getStayingReference().getModelId());
			this.fillComposition(participatingModel, perceptualModel, componentManager);	
			tacticalModel.setParticipatingModel(participatingModel);
		}

		if(configuration.getQueuingReference() != null) {
			
			QueuingModel queuingModel = componentManager.getQueuingModel(configuration.getQueuingReference().getModelId());
			this.fillComposition(queuingModel, perceptualModel, componentManager);	
			tacticalModel.setQueuingModel(queuingModel);
		}
		
		if(configuration.getRoutingReference() != null) {
			
			RoutingModel routingModel = componentManager.getRoutingModel(configuration.getRoutingReference().getModelId());
			this.fillComposition(routingModel, perceptualModel, componentManager);	
			tacticalModel.setRoutingModel(routingModel);
		}
		
		if(configuration.getSerachingReference() != null) {
			
			SearchingModel searchingModel = componentManager.getSearchingModel(configuration.getSerachingReference().getModelId());
			this.fillComposition(searchingModel, perceptualModel, componentManager);
			tacticalModel.setSearchingModel(searchingModel);
		}
		
		return tacticalModel;
	}
}
