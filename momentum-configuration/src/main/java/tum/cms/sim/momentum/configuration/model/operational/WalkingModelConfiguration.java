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

package tum.cms.sim.momentum.configuration.model.operational;

import java.util.HashMap;

import com.thoughtworks.xstream.annotations.XStreamAlias;
import com.thoughtworks.xstream.annotations.XStreamAsAttribute;
import com.thoughtworks.xstream.converters.enums.EnumToStringConverter;

import tum.cms.sim.momentum.configuration.generic.PropertyContainerNode;

@XStreamAlias("walking")
public class WalkingModelConfiguration extends PropertyContainerNode {
	
	public enum WalkingModelType {
		
		NoInteraction,
		SocialForce,
		Bonneaud,
		StockCellular,
		ParallelStockCellular,
		Moussaid, 
		DynamicFloorfield,
		MoussaidHeuristic,
		ClassicLWR,
		ParallelSocialForce,
		SharedSpaceForce,
		CsvPlayback
	}
	
	@SuppressWarnings("rawtypes")
	public static EnumToStringConverter getTypeConverter() {

		HashMap<String, WalkingModelType> map = new HashMap<>();
		map.put(WalkingModelType.ParallelSocialForce.toString(), WalkingModelType.ParallelSocialForce);
		map.put(WalkingModelType.SocialForce.toString(), WalkingModelType.SocialForce);
		map.put(WalkingModelType.Bonneaud.toString(), WalkingModelType.Bonneaud);	
		map.put(WalkingModelType.NoInteraction.toString(), WalkingModelType.NoInteraction);
		map.put(WalkingModelType.StockCellular.toString(), WalkingModelType.StockCellular);
		map.put(WalkingModelType.ParallelStockCellular.toString(), WalkingModelType.ParallelStockCellular);
		map.put(WalkingModelType.Moussaid.toString(), WalkingModelType.Moussaid);
		map.put(WalkingModelType.DynamicFloorfield.toString(), WalkingModelType.DynamicFloorfield);
		map.put(WalkingModelType.MoussaidHeuristic.toString(), WalkingModelType.MoussaidHeuristic);
		map.put(WalkingModelType.ClassicLWR.toString(), WalkingModelType.ClassicLWR);
		map.put(WalkingModelType.SharedSpaceForce.toString(), WalkingModelType.SharedSpaceForce);
		map.put(WalkingModelType.CsvPlayback.toString(), WalkingModelType.CsvPlayback);
		
		return new EnumToStringConverter<>(WalkingModelType.class, map);
	}
	
	@XStreamAsAttribute
	private WalkingModelType type;

	public WalkingModelType getType() {
		return type;
	}

	public void setType(WalkingModelType type) {
		this.type = type;
	}
}
