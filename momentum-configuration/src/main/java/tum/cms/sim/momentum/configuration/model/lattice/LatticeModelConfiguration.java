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

package tum.cms.sim.momentum.configuration.model.lattice;

import java.util.HashMap;

import com.thoughtworks.xstream.annotations.XStreamAlias;
import com.thoughtworks.xstream.annotations.XStreamAsAttribute;
import com.thoughtworks.xstream.converters.enums.EnumToStringConverter;

import tum.cms.sim.momentum.configuration.generic.PropertyContainerNode;

@XStreamAlias("lattice")
public class LatticeModelConfiguration extends PropertyContainerNode {

	public enum LatticeType {
		
		Quadratic,
		Hexagon
	}
	
	@SuppressWarnings("rawtypes")
	public static EnumToStringConverter getLatticeTypeConverter() {
		
		HashMap<String, LatticeType> map = new HashMap<>();
		map.put(LatticeType.Quadratic.toString(), LatticeType.Quadratic);
		map.put(LatticeType.Hexagon.toString(), LatticeType.Hexagon);
		
		return new EnumToStringConverter<>(LatticeType.class, map);
	}
	
	public enum NeighborhoodType {
		
		Touching,
		Edge
	}
	
	@SuppressWarnings("rawtypes")
	public static EnumToStringConverter getNeighbourhoodTypeConverter() {
		
		HashMap<String, NeighborhoodType> map = new HashMap<>();
		map.put(NeighborhoodType.Edge.toString(), NeighborhoodType.Edge);
		map.put(NeighborhoodType.Touching.toString(), NeighborhoodType.Touching);
		
		return new EnumToStringConverter<>(NeighborhoodType.class, map);
	}

	
	@XStreamAsAttribute
	private Integer scenarioId = null;
	
	public Integer getScenarioId() {
		return scenarioId;
	}

	public void setScenarioId(Integer scenarioId) {
		this.scenarioId = scenarioId;
	}

	@XStreamAsAttribute
	private LatticeType latticeType;

	public LatticeType getLatticeType() {
		return latticeType;
	}

	public void setLatticeType(LatticeType latticeType) {
		this.latticeType = latticeType;
	}

	@XStreamAsAttribute
	private NeighborhoodType neighborhoodType;

	public NeighborhoodType getNeigborhoodType() {
		return neighborhoodType;
	}

	public void setNeigboorhoodType(NeighborhoodType neigborhoodType) {
		this.neighborhoodType = neigborhoodType;
	}
	
	@XStreamAsAttribute
	private double cellEdgeSize;

	public double getCellEdgeSize() {
		return this.cellEdgeSize;
	}
	
	public void setCellEdgeSize(double cellEdgeSize) {
		this.cellEdgeSize = cellEdgeSize;
	}
	
	public enum BehaviorType {
		Synchronized,
		Unsynchronized
	}
	
	@SuppressWarnings("rawtypes")
	public static EnumToStringConverter getBehaviorTypeConverter() {
		
		HashMap<String, BehaviorType> map = new HashMap<>();
		map.put(BehaviorType.Synchronized.toString(), BehaviorType.Synchronized);
		map.put(BehaviorType.Unsynchronized.toString(), BehaviorType.Unsynchronized);
		
		return new EnumToStringConverter<>(BehaviorType.class, map);
	}
	
	@XStreamAsAttribute
	private BehaviorType behaviorType;

	public BehaviorType getBehaviorType() {
		return behaviorType;
	}

	public void setBehaviorType(BehaviorType behavior) {
		this.behaviorType = behavior;
	}
}
