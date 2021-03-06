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

package tum.cms.sim.momentum.utility.lattice;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Set;

import org.apache.commons.lang3.tuple.Pair;

import tum.cms.sim.momentum.configuration.model.lattice.LatticeModelConfiguration.LatticeType;
import tum.cms.sim.momentum.configuration.model.lattice.LatticeModelConfiguration.NeighborhoodType;
import tum.cms.sim.momentum.utility.generic.IUnique;
import tum.cms.sim.momentum.utility.generic.PropertyBackPack;
import tum.cms.sim.momentum.utility.geometry.Cycle2D;
import tum.cms.sim.momentum.utility.geometry.Polygon2D;
import tum.cms.sim.momentum.utility.geometry.Segment2D;
import tum.cms.sim.momentum.utility.geometry.Vector2D;
import tum.cms.sim.momentum.utility.lattice.Lattice.Occupation;

public interface ILattice extends IUnique {

	PropertyBackPack getPropertyBackPack();

	void setPropertyBackPack(PropertyBackPack propertyContainer);

	List<CellIndex> getCellsInOrder();

	NeighborhoodType getNeighborhoodType();

	void setNeighborhoodType(NeighborhoodType type);

	LatticeType getType();

	double getCellEdgeSize();
	
	double getCellArea();

	int getNumberOfRows();

	int getNumberOfColumns();

	Vector2D getStartCellPosition();

	Vector2D getMinPositionBoundingBox();

	Vector2D getMaxPositionBoundingBox();

	CellIndex getCellIndexFromPosition(Double xValue, Double yValue);

	CellIndex getCellIndexFromPosition(Vector2D position);

	Vector2D getCenterPosition(CellIndex index);

	Vector2D getCenterPosition(int row, int column);

	ArrayList<CellIndex> getAllNeighborIndices(CellIndex cellIndex);

	Polygon2D getCellPolygon(CellIndex cellIndex);

	Boolean isCellFree(CellIndex cellIndex);

	void increaseCellNumberValue(CellIndex cellIndex, double addToNumberValue);

	void setCellNumberValue(CellIndex cellIndex, double numberValue);

	void setCellNumberValue(int row, int column, double numberValue);

	void setCellNumberValue(Vector2D position, double numberValue);

	double getCellNumberValue(CellIndex cellIndex);

	double getCellNumberValue(int row, int column);

	Occupation getCellValue(CellIndex cellIndex);

	Occupation getCellValue(int row, int column);

	Boolean isCellFree(int row, int column);

	void setCellTo(CellIndex cellIndex, Occupation occupation);

	boolean occupyCell(CellIndex cellIndex, Occupation occupation);

	Boolean occupyCellIfFree(CellIndex cellIndex, Occupation occupation);

	Boolean occupyCell(int row, int column, Occupation occupation);

	boolean isInLatticeBounds(CellIndex index);

	boolean setCellIfFree(CellIndex cellIndex);

	boolean freeCell(CellIndex cellIndex);

	Boolean freeCell(int row, int column);

	ArrayList<CellIndex> getAllNeighborIndices(Vector2D position);

	void occupyCells(Collection<CellIndex> cells, Occupation occupation);

	HashMap<CellIndex, Vector2D> getBorderPositionForCells(Segment2D segment);

	HashMap<CellIndex, Vector2D> getInsidePositionForCells(Polygon2D polygon);

	HashMap<CellIndex, Vector2D> getBorderPositionForCells(Polygon2D polygon);

	/**
	 * Computes the minimum and maximum Double value respectively for each Area.
	 * 
	 * @param connectedAreas
	 * @return a List which contains the minimum and maximum paired to the
	 *         respective connected area.
	 */
	Double[] getMinMaxValuesForIndices(Set<CellIndex> connectedIndices);

	List<CellIndex> getAllPolygonCells(Polygon2D polygon);

	List<CellIndex> occupyAllPolygonCells(Polygon2D polygon, Occupation occupation);

	List<CellIndex> occupyAllSegmentCells(List<Segment2D> segments, Occupation occupation);

	List<CellIndex> occupyAllCellsInRadius(Vector2D position, double radius, Occupation occupation);
	
	List<CellIndex> occupyAllPolygonCells(Polygon2D polygon, double value);

	List<CellIndex> occupyAllSegmentCells(List<Segment2D> segments, double value);
	
	void setAllCells(Occupation occupation);

	void setAllCells(Double value);

	void setAllCellIfFree(double maxValue);

	void setAllCells(ILattice otherLattice);

	void setAllFreeTo(Occupation occupancy);

	void setCells(List<CellIndex> cellsToOccupy, double value);
	
	ArrayList<Vector2D> getCornerPoints(CellIndex cell);

	int getNumberOfOccupiedNeighbors(CellIndex cellIndex);

	void paintLattice();

	void paintLatticeWithValues();

	int computeDistanceMap(List<CellIndex> occupiedCells);
	
	List<List<CellIndex>> findLocalMinimal(int globalMaximal, int globalMinimal);
	
	/**
	 * This method attempts to casts a line from the start to the target CellIndex. 
	 * When the line hits the grid boundaries or an obstacle, it terminates.
	 * @param start
	 * @param target
	 * @return true if the target CellIndex is reached, false otherwise.
	 */
	
	boolean breshamLineCast(CellIndex start, CellIndex target);
	
	/**
	 * from http://playtechs.blogspot.de/2007/03/raytracing-on-grid.html
	 * 
	 * 
	 * @param from
	 * @param towards
	 * @param lattice
	 * @return zero (0.0) if towards cell was hit without intersection else the value of a hit cell
	 */
	double breshamLineCast(CellIndex from, CellIndex towards, int cellDistance);

	/**
	 * From cell start
	 * to cell end
	 * stop value can be null, if not null stop only if the ray finds a value
	 * if null stop at any non 0.0 value.
	 * ignore value will be ignored if exists
	 * @return returns a list of traversed cells and their values excluding the stop value
	 */
	List<Pair<Double, CellIndex>> breshamLineCastTrace(CellIndex from, CellIndex towards, Double stopValue, Double ignorevalue, boolean storeTrace);

	
	void flood(List<CellIndex> startingCells);

	/**
	 * Floods the lattice from the given starting CellIndex without altering the values
	 * and returns an index structure of all cells that are reachable from the starting cell.
	 * @param startCell from where the flooding starts
	 * @return a set of cell indices that are connected
	 */
	Set<CellIndex> flood(CellIndex startCell);

	List<CellIndex> getAllCellsWith(Occupation occupation);

	List<CellIndex> getAllCellsWithValue(Double value);

	List<CellIndex> getAllCircleCells(Cycle2D circle);
	
	List<CellIndex> getAllCircleCells(double radius, Vector2D position);
	
	List<CellIndex> getAllOnCircleBorder(double radius, Vector2D center);
}