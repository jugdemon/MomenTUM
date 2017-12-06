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

package tum.cms.sim.momentum.utility.csvData;

import tum.cms.sim.momentum.utility.csvData.reader.BufferingStrategy;

public enum CsvType {
	
	TransitZones,
	MacroscopicNetwork,
	xtDensity,
	Pedestrian,
	Car;
	
	public String getIdHeader() {
		
		switch (this) {
		
		case TransitZones:
			return "transitid";
		case MacroscopicNetwork:
			return "macroid";
		case xtDensity:
			return "id";
		case Pedestrian:
			return "id";
        case Car:
            return "id";
		default:
			return "id";
		}
	}
	
	public BufferingStrategy getBufferingStrategy() {
		
		switch (this) {
		
		case TransitZones:
			return BufferingStrategy.FileBuffer;
		case MacroscopicNetwork:
			return BufferingStrategy.AllBuffer;
		case xtDensity:
			return BufferingStrategy.FileBuffer;
		case Pedestrian:
			return BufferingStrategy.FileBuffer;
        case Car:
            return BufferingStrategy.FileBuffer;
		default:
			return BufferingStrategy.NoBuffer;
		}
	}
	
	/**
	 * 
	 * @return if this {@link CsvFile} has a custom type.
	 */
	public static boolean isCustomType(CsvType csvType) {
		if(csvType.equals(CsvType.Pedestrian)) {
			return false;
		} else {
			return true;
		}
	}

}
