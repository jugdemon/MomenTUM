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
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Set;
import java.util.Map.Entry;

import org.apache.commons.math3.util.FastMath;

import tum.cms.sim.momentum.configuration.model.lattice.LatticeModelConfiguration.LatticeType;
import tum.cms.sim.momentum.configuration.model.lattice.LatticeModelConfiguration.NeighbourhoodType;
import tum.cms.sim.momentum.utility.generic.IHasProperties;
import tum.cms.sim.momentum.utility.generic.PropertyBackPack;
import tum.cms.sim.momentum.utility.generic.Unique;
import tum.cms.sim.momentum.utility.geometry.Cycle2D;
import tum.cms.sim.momentum.utility.geometry.GeometryFactory;
import tum.cms.sim.momentum.utility.geometry.Polygon2D;
import tum.cms.sim.momentum.utility.geometry.Segment2D;
import tum.cms.sim.momentum.utility.geometry.Vector2D;
import tum.cms.sim.momentum.utility.lattice.Lattice.Occupation;
import tum.cms.sim.momentum.utility.lattice.operation.QuadraticLatticCalculation;

public class Lattice extends Unique implements IHasProperties, ILattice {
	protected List<CellIndex> cellsInOrder = null;
	
	protected CellGrid grid = null;
	protected double cellEdgeSize;  // size of cell: length of one bounding line
	protected Vector2D startCellPosition;
	
	protected LatticeType latticeType = null;
	protected NeighbourhoodType neighborhoodType = null;
	private Double cellArea = null;

	public enum Occupation {
		Fixed,
		Dynamic,
		Empty
	};
	
	private Double doubleForFixedObject = Double.NaN; 
	private Double doubleForDynamicObject = Double.MAX_VALUE;
	private Double doubleForNoObject = 0.0;	
	
	protected PropertyBackPack properties = null;
	protected Vector2D minPositionBoundingBox = null;
	protected Vector2D maxPositionBoundingBox = null;


	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getPropertyBackPack()
	 */
	@Override
	public PropertyBackPack getPropertyBackPack() {
		return properties;
	}

	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#setPropertyBackPack(tum.cms.sim.momentum.utility.generic.PropertyBackPack)
	 */
	@Override
	public void setPropertyBackPack(PropertyBackPack propertyContainer) {

		this.properties = propertyContainer;
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getCellsInOrder()
	 */
	@Override
	public List<CellIndex> getCellsInOrder() {
	
		if(cellsInOrder == null) {
				
			int numberOfCells = this.grid.getNumberOfColumns() * this.grid.getNumberOfRows();
			ArrayList<CellIndex> cells = new ArrayList<CellIndex>(numberOfCells);
			
			for(int rowIter = 0; rowIter < this.grid.getNumberOfRows(); rowIter++) {
			
				for(int columnIter = 0; columnIter < this.grid.getNumberOfColumns(); columnIter++) {
					
					cells.add(LatticeTheoryFactory.createCellIndex(rowIter, columnIter));
				}
			}
			
			cellsInOrder = cells;
		}
		
		return cellsInOrder;
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getNeighborhoodType()
	 */
	@Override
	public NeighbourhoodType getNeighborhoodType() {
		return neighborhoodType;
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#setNeighborhoodType(tum.cms.sim.momentum.configuration.model.lattice.LatticeModelConfiguration.NeighbourhoodType)
	 */
	@Override
	public void setNeighborhoodType(NeighbourhoodType type) {
		this.neighborhoodType = type;
	}

	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getType()
	 */
	@Override
	public LatticeType getType() {
		return latticeType;
	}	
	

	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getCellEdgeSize()
	 */
	@Override
	public double getCellEdgeSize() {
		return cellEdgeSize;
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getCellArea()
	 */
	@Override
	public double getCellArea() {
		
		return cellArea;
	}
	
	private void setCellArea(Double cellArea) {
		
		this.cellArea = cellArea;
	}
	
	private double calculateCellArea() {
		
		Double cellArea = null;
		
		switch (this.getType()) {
		
		case Hexagon:
			
			cellArea = cellEdgeSize * cellEdgeSize * 1.5 * Math.sqrt(3);
			break;
		
		case Quadratic:
			
			cellArea = cellEdgeSize * cellEdgeSize;
			break;
		
			default:
				break;				
		}
		return cellArea;
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getNumberOfRows()
	 */
	@Override
	public int getNumberOfRows() {
		return grid.getNumberOfRows();
	}

	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getNumberOfColumns()
	 */
	@Override
	public int getNumberOfColumns() {
		return grid.getNumberOfColumns();
	}

	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getStartCellPosition()
	 */
	@Override
	public Vector2D getStartCellPosition() {
		return startCellPosition.copy();
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getMinPositionBoundingBox()
	 */
	@Override
	public Vector2D getMinPositionBoundingBox() {
		return minPositionBoundingBox;
	}

	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getMaxPositionBoundingBox()
	 */
	@Override
	public Vector2D getMaxPositionBoundingBox() {
		return maxPositionBoundingBox;
	}
	/**
	 * 
	 * @param latticeType
	 * @param neigborhoodType
	 * @param fillingType
	 * @param cellEdgeSize
	 * @param maxX
	 * @param minX
	 * @param maxY
	 * @param minY
	 */
	protected Lattice(LatticeType latticeType, 
			NeighbourhoodType neigborhoodType,
			double cellEdgeSize, 
			double maxX,
			double minX,
			double maxY,
			double minY) {
	
		this.latticeType = latticeType;
		this.neighborhoodType = neigborhoodType;


		int maxRowIndex = 0;
		int maxColumnIndex = 0;
		
		switch(this.latticeType) {
		
		case Hexagon:
			break;
			
		case Quadratic:
		default:
			maxRowIndex = QuadraticLatticCalculation.calculateMaxIndex(maxY, minY, cellEdgeSize);
			maxColumnIndex = QuadraticLatticCalculation.calculateMaxIndex(maxX, minX, cellEdgeSize);
			this.startCellPosition = GeometryFactory.createVector(minX, minY); // the cell (0|0) has the center position (minX,minY)
			break;	
		}		
		
		this.maxPositionBoundingBox = GeometryFactory.createVector(maxX, maxY);
		this.minPositionBoundingBox = GeometryFactory.createVector(minX, minY);
		this.grid = new CellGrid(maxRowIndex, maxColumnIndex);	
		this.cellEdgeSize = cellEdgeSize;
		this.setCellArea(this.calculateCellArea());
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getCellIndexFromPosition(java.lang.Double, java.lang.Double)
	 */
	@Override
	public CellIndex getCellIndexFromPosition(Double xValue, Double yValue) {
		
		return getCellIndexFromPosition(GeometryFactory.createVector(xValue, yValue));
	}

	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getCellIndexFromPosition(tum.cms.sim.momentum.utility.geometry.Vector2D)
	 */
	@Override
	public CellIndex getCellIndexFromPosition(Vector2D position) {
		
		CellIndex cellIndex = null;
		switch(this.latticeType)  {
		
		case Hexagon:
			break;
			
		case Quadratic:
		default:
			cellIndex = LatticeTheoryFactory.createCellIndex(
					QuadraticLatticCalculation.getCellIndexFromPosition(position,
						this.cellEdgeSize, 
						this.startCellPosition));
		}
		return cellIndex;
	}

	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getCenterPosition(tum.cms.sim.momentum.utility.lattice.CellIndex)
	 */
	@Override
	public Vector2D getCenterPosition(CellIndex index) {
		
		int row = index.getRow();
		int column = index.getColumn();
		return this.getCenterPosition(row, column);
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getCenterPosition(int, int)
	 */
	@Override
	public Vector2D getCenterPosition(int row, int column) {
		
		Vector2D position = null;
		
		switch(this.latticeType)  {
		
		case Hexagon:
			break;
			
		case Quadratic:
		default:
			position = QuadraticLatticCalculation.getCenterPositionFromCellIndex(row, column, this.cellEdgeSize, this.startCellPosition);
		}
		return position;
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getAllNeighborIndices(tum.cms.sim.momentum.utility.lattice.CellIndex)
	 */
	@Override
	public ArrayList<CellIndex> getAllNeighborIndices(CellIndex cellIndex) {
		
		ArrayList<CellIndex> cellIndizes = null;

		switch(this.latticeType)  {
		case Hexagon:
			break;
			
		case Quadratic:
		default:
			
			switch(this.neighborhoodType) {
			
			case Edge:
				cellIndizes = QuadraticLatticCalculation.getAllNeumannNeighborIndices(cellIndex);
				break;
				
			case Touching:
			default:
				cellIndizes = QuadraticLatticCalculation.getAllMooreNeighborIndices(cellIndex);
				break;
			}				
			break;
		}		
		return cellIndizes;
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getCellPolygon(tum.cms.sim.momentum.utility.lattice.CellIndex)
	 */
	@Override
	public Polygon2D getCellPolygon(CellIndex cellIndex) {
		
		Vector2D cellCenter = this.getCenterPosition(cellIndex);
		Polygon2D cellPolygon = null;
		
		switch(this.latticeType)  {
		
		case Hexagon:
			break;
			
		case Quadratic:
		default:
			cellPolygon = QuadraticLatticCalculation.calculateCellPolygon(cellCenter, this.cellEdgeSize);
		}
		return cellPolygon;
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#isCellFree(tum.cms.sim.momentum.utility.lattice.CellIndex)
	 */
	@Override
	public synchronized Boolean isCellFree(CellIndex cellIndex) {
		
		Double cellValue = grid.getCell(cellIndex);
		
		if (cellValue.equals(doubleForFixedObject) || cellValue.equals(doubleForDynamicObject)) {		
			return false;
		}
		return true;		
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#increaseCellNumberValue(tum.cms.sim.momentum.utility.lattice.CellIndex, double)
	 */
	@Override
	public synchronized void increaseCellNumberValue(CellIndex cellIndex, double addToNumberValue) {
		
		this.grid.setCell(cellIndex, this.grid.getCell(cellIndex).doubleValue() + addToNumberValue);
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#setCellNumberValue(tum.cms.sim.momentum.utility.lattice.CellIndex, double)
	 */
	@Override
	public void setCellNumberValue(CellIndex cellIndex, double numberValue) {
		
		this.grid.setCell(cellIndex, numberValue);
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#setCellNumberValue(int, int, double)
	 */
	@Override
	public void setCellNumberValue(int row, int column, double numberValue) {
		
		this.grid.setCell(row, column, numberValue);
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getCellNumberValue(tum.cms.sim.momentum.utility.lattice.CellIndex)
	 */
	@Override
	public double getCellNumberValue(CellIndex cellIndex) {
	
		return this.grid.getCell(cellIndex).doubleValue();
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getCellNumberValue(int, int)
	 */
	@Override
	public double getCellNumberValue(int row, int column) {
		
		return this.grid.getCell(row, column).doubleValue();
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getCellValue(tum.cms.sim.momentum.utility.lattice.CellIndex)
	 */
	@Override
	public Occupation getCellValue(CellIndex cellIndex) {
		
		Double value = this.grid.getCell(cellIndex).doubleValue();
		
		if(value.isNaN()) {
			
			return Occupation.Fixed;
		}
		
		if(value.equals(Double.MAX_VALUE)) {
			
			return Occupation.Dynamic;
		}

		return Occupation.Empty;
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getCellValue(int, int)
	 */
	@Override
	public Occupation getCellValue(int row, int column) {
		
		Double cellValue = grid.getCell(row, column);

		if (cellValue.equals(doubleForFixedObject)) {
			return Occupation.Fixed;
		}
		
		if (cellValue.equals(doubleForDynamicObject)) {
			return Occupation.Dynamic;
		}
		
		return Occupation.Empty;
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#isCellFree(int, int)
	 */
	@Override
	public Boolean isCellFree(int row, int column) {
		
		Double cellValue = grid.getCell(row, column);
		
		if (cellValue.equals(doubleForFixedObject) || cellValue.equals(doubleForDynamicObject)) {		
			return false;
		}
		return true;	
	}

	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#setCellTo(tum.cms.sim.momentum.utility.lattice.CellIndex, tum.cms.sim.momentum.utility.lattice.Lattice.Occupation)
	 */
	@Override
	public synchronized void setCellTo(CellIndex cellIndex, Occupation occupation) {
		
		this.grid.setCell(cellIndex, this.convertOccupationToDouble(occupation));
	}
	
	@Override
	public synchronized void setCellTo(int row, int column, Occupation occupation) {
		
		this.setCellTo(new CellIndex(row, column), occupation);
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#occupyCell(tum.cms.sim.momentum.utility.lattice.CellIndex, tum.cms.sim.momentum.utility.lattice.Lattice.Occupation)
	 */
	
	@Override
	public synchronized boolean occupyCell(CellIndex cellIndex, Occupation occupation) {
		
		boolean success = false;
		
		if (occupation == Occupation.Empty) {
			return success;
		}
		
		if (this.isCellFree(cellIndex)) {
			
			this.grid.setCell(cellIndex, this.convertOccupationToDouble(occupation));	
			success = true;
		}	
		
		return success;
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#occupyCellIfFree(tum.cms.sim.momentum.utility.lattice.CellIndex, tum.cms.sim.momentum.utility.lattice.Lattice.Occupation)
	 */
	@Override
	public synchronized Boolean occupyCellIfFree(CellIndex cellIndex, Occupation occupation) {
		
		boolean success = false;
		
		if (occupation == Occupation.Empty) {
			return success;
		}
		
		if (this.isCellFree(cellIndex)) {
			
			this.grid.setCell(cellIndex, this.convertOccupationToDouble(occupation));	
			success = true;
		}	
		
		return success;
	}

	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#occupyCell(int, int, tum.cms.sim.momentum.utility.lattice.Lattice.Occupation)
	 */
	@Override
	public synchronized Boolean occupyCell(int row, int column, Occupation occupation) {
		
		CellIndex cellIndex = new CellIndex(row, column);
		
		return this.occupyCell(cellIndex, occupation);
	}
	
	private Double convertOccupationToDouble(Occupation occupation) {
		
		if (occupation == Occupation.Fixed) {
			return doubleForFixedObject;						
		}
		if (occupation == Occupation.Dynamic) {
			return doubleForDynamicObject;						
		}
		return doubleForNoObject;
	}

	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#isInLatticeBounds(tum.cms.sim.momentum.utility.lattice.CellIndex)
	 */
	@Override
	public boolean isInLatticeBounds(CellIndex index) {
		
		boolean isInBounds = false;
		
		if(index.getRow() >= 0 && 
		   index.getColumn() >= 0 &&
		   index.getRow() < this.getNumberOfRows() &&
		   index.getColumn() < this.getNumberOfColumns()) {
				
			isInBounds = true;
		}
		
		return isInBounds;
	}

	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#setCellIfFree(tum.cms.sim.momentum.utility.lattice.CellIndex)
	 */
	@Override
	public synchronized boolean setCellIfFree(CellIndex cellIndex){
		
		boolean success = false;
		
		if (this.isCellFree(cellIndex)) {
			
			this.grid.setCell(cellIndex, doubleForNoObject);	
			success = true;
		}		
		
		return success;		
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#freeCell(tum.cms.sim.momentum.utility.lattice.CellIndex)
	 */
	@Override
	public synchronized boolean freeCell(CellIndex cellIndex){
		
		boolean success = false;
		
		if (!this.isCellFree(cellIndex)) {
			
			this.grid.setCell(cellIndex, doubleForNoObject);	
			success = true;
		}		
		return success;		
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#freeCell(int, int)
	 */
	@Override
	public synchronized Boolean freeCell(int row, int column) {
		
		CellIndex cellIndex = new CellIndex(row, column);
		
		return this.freeCell(cellIndex);
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getAllNeighborIndices(tum.cms.sim.momentum.utility.geometry.Vector2D)
	 */
	@Override
	public ArrayList<CellIndex> getAllNeighborIndices(Vector2D position) {
		
		CellIndex cellIndex = this.getCellIndexFromPosition(position);
		return getAllNeighborIndices(cellIndex);
	}

	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#occupyInsideCells(java.util.Collection, tum.cms.sim.momentum.utility.lattice.Lattice.Occupation)
	 */
	@Override
	public synchronized void occupyInsideCells(Collection<CellIndex> cells, Occupation occupation) {
	
		if(cells != null) {
			
			cells.forEach(cellIndex -> this.occupyCell(cellIndex, occupation));
		}
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getBorderPositionForCells(tum.cms.sim.momentum.utility.geometry.Segment2D)
	 */
	@Override
	public HashMap<CellIndex, Vector2D> getBorderPositionForCells(Segment2D segment) {
		
		HashMap<CellIndex, Vector2D> borderPositions = new HashMap<CellIndex, Vector2D>();
		
		switch(this.latticeType) {
		
		case Hexagon:
			break;
			
		case Quadratic:
		default:
			
			double polygonMinX = segment.getSmallestVertixValueOfX();
			double polygonMinY = segment.getSmallestVertixValueOfY();	
			double polygonMaxX = segment.getLargestVertixValueOfX();
			double polygonMaxY =segment.getLargestVertixValueOfY();	
			
			CellIndex cellWithMinValues = this.getCellIndexFromPosition(polygonMinX, polygonMinY);
			CellIndex cellWithMaxValues = this.getCellIndexFromPosition(polygonMaxX, polygonMaxY);
			
			int rowStart = cellWithMinValues.getRow();
			int columnStart = cellWithMinValues.getColumn();
			int rowEnd = cellWithMaxValues.getRow();
			int columnEnd = cellWithMaxValues.getColumn();
			
			for(int rowIter = rowStart; rowIter <= rowEnd ; rowIter++) {
							
				for(int columnIter = columnStart; columnIter <= columnEnd; columnIter++)  {
					
					CellIndex currentCellIndex = LatticeTheoryFactory.createCellIndex(rowIter, columnIter);
					Polygon2D currentCellPolygon = this.getCellPolygon(currentCellIndex);
					
					if (!segment.getIntersection(currentCellPolygon).isEmpty()) {
						
						borderPositions.put(currentCellIndex, this.getCenterPosition(currentCellIndex));
					}					
				}
			}
		}
		return borderPositions;
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getInsidePositionForCells(tum.cms.sim.momentum.utility.geometry.Polygon2D)
	 */
	@Override
	public HashMap<CellIndex, Vector2D> getInsidePositionForCells(Polygon2D polygon) {

		HashMap<CellIndex, Vector2D> insideCellPositions = new HashMap<CellIndex, Vector2D>();
		
		switch(this.latticeType)  {
		
		case Hexagon:
			break;
			
		case Quadratic:
		default:
			insideCellPositions = QuadraticLatticCalculation.calculateInsideCellPositions(polygon, this);
			break;
		}
		return insideCellPositions;		
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getBorderPositionForCells(tum.cms.sim.momentum.utility.geometry.Polygon2D)
	 */
	@Override
	public HashMap<CellIndex, Vector2D> getBorderPositionForCells(Polygon2D polygon) {

		HashMap<CellIndex, Vector2D> borderCellPositions = new HashMap<CellIndex, Vector2D>();
		
		switch(this.latticeType)  {
		
		case Hexagon:
			break;
			
		case Quadratic:
		default:
			
			borderCellPositions = QuadraticLatticCalculation.calculateBorderCellPositions(polygon, this);

			break;
		}
		return borderCellPositions;		
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getAllCircleCells(tum.cms.sim.momentum.utility.geometry.Cycle2D)
	 */
	@Override
	public List<CellIndex> getAllCircleCells(Cycle2D circle) {
		
		ArrayList<CellIndex> cells = new ArrayList<CellIndex>();
		Double radius = circle.getRadius();
		Vector2D center = circle.getCenter();
		
		Vector2D circlePointNorth = center.sum(GeometryFactory.createVector(0.0, -radius));
		Vector2D circlePointEast = center.sum(GeometryFactory.createVector(radius, 0.0));
		Vector2D circlePointSouth = center.sum(GeometryFactory.createVector(0.0, +radius));
		Vector2D circlePointWest = center.sum(GeometryFactory.createVector(-radius, 0.0));
		
		Vector2D circlePointNW = GeometryFactory.createVector(circlePointWest.getXComponent(), circlePointNorth.getYComponent());
		Vector2D circlePointSE = GeometryFactory.createVector(circlePointEast.getXComponent(), circlePointSouth.getYComponent());
		
		Integer startIndexRow = this.getCellIndexFromPosition(circlePointNW).getRow();
		Integer startIndexColumn = this.getCellIndexFromPosition(circlePointNW).getColumn();
		Integer endIndexRow = this.getCellIndexFromPosition(circlePointSE).getRow();
		Integer endIndexColumn = this.getCellIndexFromPosition(circlePointSE).getColumn();
		
		for (int row = startIndexRow; row <= endIndexRow; row++) {
			
			for (int column = startIndexColumn; column <= endIndexColumn; column++) {
				
				CellIndex cell = LatticeTheoryFactory.createCellIndex(row, column);
				
				if (this.isInLatticeBounds(cell) && center.distance(this.getCenterPosition(cell)) <= radius) {
					
					cells.add(cell);
				}
			}			
		}
		return cells;
	}

	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getAllPolygonCells(tum.cms.sim.momentum.utility.geometry.Polygon2D)
	 */
	@Override
	public List<CellIndex> getAllPolygonCells(Polygon2D polygon) {
	
		HashMap<CellIndex, Vector2D> insideCellPositions = this.getInsidePositionForCells(polygon);
		HashMap<CellIndex, Vector2D> borderCellPositions = this.getBorderPositionForCells(polygon);
		ArrayList<CellIndex> cells = new ArrayList<CellIndex>();
		
		for(Entry<CellIndex, Vector2D> cellIndexPosition : insideCellPositions.entrySet()) {
			
			if(this.isInLatticeBounds(cellIndexPosition.getKey())) {
				
				cells.add(cellIndexPosition.getKey());
			}
		}
		
		for(Entry<CellIndex, Vector2D> cellIndexPosiion : borderCellPositions.entrySet()) {
			
			if(this.isInLatticeBounds(cellIndexPosiion.getKey())) {
				
				cells.add(cellIndexPosiion.getKey());
			}
		}
		
		return cells;
	}
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#occupyAllPolygonCells(tum.cms.sim.momentum.utility.geometry.Polygon2D, tum.cms.sim.momentum.utility.lattice.Lattice.Occupation)
	 */
	@Override
	public List<CellIndex> occupyAllPolygonCells(Polygon2D polygon, Occupation occupation) {
		
		HashMap<CellIndex, Vector2D> insideCellPositions = this.getInsidePositionForCells(polygon);
		HashMap<CellIndex, Vector2D> borderCellPositions = this.getBorderPositionForCells(polygon);
		ArrayList<CellIndex> cellsToOccupy = new ArrayList<CellIndex>();
		
		for(Entry<CellIndex, Vector2D> cellIndexPosiion : insideCellPositions.entrySet()) {
			
			if(this.isInLatticeBounds(cellIndexPosiion.getKey())) {
				
				cellsToOccupy.add(cellIndexPosiion.getKey());
			}
		}
		
		for(Entry<CellIndex, Vector2D> cellIndexPosiion : borderCellPositions.entrySet()) {
			
			if(this.isInLatticeBounds(cellIndexPosiion.getKey())) {
				
				cellsToOccupy.add(cellIndexPosiion.getKey());
			}
		}
		
		cellsToOccupy.stream().forEach(cellIndex -> this.occupyCell(cellIndex, occupation));
		
		return cellsToOccupy;
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#occupyAllSegmentCells(java.util.List, tum.cms.sim.momentum.utility.lattice.Lattice.Occupation)
	 */
	@Override
	public List<CellIndex> occupyAllSegmentCells(List<Segment2D> segments, Occupation occupation) {
		
		ArrayList<CellIndex> cellsToOccupy = new ArrayList<CellIndex>();
		
		for (Segment2D segment : segments) {
			
			HashMap<CellIndex, Vector2D> cellPositions = this.getBorderPositionForCells(segment);
			
			for(Entry<CellIndex, Vector2D> cellIndexPosiion : cellPositions.entrySet()) {
				
				if(this.isInLatticeBounds(cellIndexPosiion.getKey())) {
					
					cellsToOccupy.add(cellIndexPosiion.getKey());
				}
			}
		}
		cellsToOccupy.stream().forEach(cellIndex -> this.occupyCell(cellIndex, occupation));
		
		return cellsToOccupy;
	}

	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#setAllCells(tum.cms.sim.momentum.utility.lattice.Lattice.Occupation)
	 */
	@Override
	public void setAllCells(Occupation occupation) {

		for (int row = 0; row < this.getNumberOfRows(); row++) {
			
			for (int column = 0; column < this.getNumberOfColumns(); column++){
				
				CellIndex cellIndex = LatticeTheoryFactory.createCellIndex(row, column);
				this.grid.setCell(cellIndex, this.convertOccupationToDouble(occupation));				
			}
		}
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#setAllCells(java.lang.Double)
	 */
	@Override
	public void setAllCells(Double value) {

		for (int row = 0; row < this.getNumberOfRows(); row++) {
			
			for (int column = 0; column < this.getNumberOfColumns(); column++){
				
				CellIndex cellIndex = LatticeTheoryFactory.createCellIndex(row, column);
				this.grid.setCell(cellIndex, value);				
			}
		}
	}

	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#setAllCellIfFree(double)
	 */
	@Override
	public void setAllCellIfFree(double maxValue) {
		
		for (int row = 0; row < this.getNumberOfRows(); row++) {
			
			for (int column = 0; column < this.getNumberOfColumns(); column++){
				
				CellIndex cellIndex = LatticeTheoryFactory.createCellIndex(row, column);
				
				if(this.isCellFree(cellIndex)) {
				
					this.grid.setCell(cellIndex, maxValue);	
				}
			}
		}
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#setAllCells(tum.cms.sim.momentum.utility.lattice.SynchronizedLattice)
	 */
	@Override
	public void setAllCells(ILattice otherLattice) {
		
		for (int row = 0; row < this.getNumberOfRows(); row++) {
			
			for (int column = 0; column < this.getNumberOfColumns(); column++){
				
				CellIndex cellIndex = LatticeTheoryFactory.createCellIndex(row, column);
				this.grid.setCell(cellIndex, this.convertOccupationToDouble(otherLattice.getCellValue(cellIndex)));				
			}
		}
	}

	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#setAllFreeTo(tum.cms.sim.momentum.utility.lattice.Lattice.Occupation)
	 */
	@Override
	public void setAllFreeTo(Occupation occupancy) {
		
		for (int row = 0; row < this.getNumberOfRows(); row++) {
			
			for (int column = 0; column < this.getNumberOfColumns(); column++){

				if(this.isCellFree(row, column)) {
					
					CellIndex cellIndex = LatticeTheoryFactory.createCellIndex(row, column);
					this.occupyCell(cellIndex, occupancy);
				}
			}
		}
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getCornerPoints(tum.cms.sim.momentum.utility.lattice.CellIndex)
	 */
	@Override
	public ArrayList<Vector2D> getCornerPoints(CellIndex cell) {
		
		ArrayList<Vector2D> cornerPoints = new ArrayList<Vector2D>(); 
		
		switch(this.latticeType)  {
		
			case Hexagon:
				break;
				
			case Quadratic:
				default:
				Vector2D centerPosition = this.getCenterPosition(cell);
				
				Vector2D LeftUpCorner = centerPosition.sum(-cellEdgeSize/2, cellEdgeSize/2);
				Vector2D LeftDownCorner = centerPosition.sum(-cellEdgeSize/2, -cellEdgeSize/2);
				Vector2D RightDownCorner = centerPosition.sum(cellEdgeSize/2, -cellEdgeSize/2);
				Vector2D RightUpCorner = centerPosition.sum(cellEdgeSize/2, cellEdgeSize/2);
				
				cornerPoints.add(LeftUpCorner);
				cornerPoints.add(LeftDownCorner);
				cornerPoints.add(RightDownCorner);
				cornerPoints.add(RightUpCorner);
			}
		return null;		
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getNumberOfOccupiedNeighbors(tum.cms.sim.momentum.utility.lattice.CellIndex)
	 */
	@Override
	public int getNumberOfOccupiedNeighbors(CellIndex cellIndex) {
		
		int counter = 0;
		List<CellIndex> neighbors = this.getAllNeighborIndices(cellIndex);

		
		for(CellIndex cell : neighbors) {
			if(!this.isInLatticeBounds(cell))
				continue;
			if(!this.isCellFree(cell))
				counter++;
		}
		
		return counter;
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#paintLattice()
	 */
	@Override
	public void paintLattice()  {
		
		int maxRows = this.getNumberOfRows();
		int maxColumns = this.getNumberOfColumns();
		
		for (int row = maxRows-1; row >= 0; row--) {
			
			System.out.print("\n");
			
			for (int column = 0; column < maxColumns; column++) {
				Double cellValue = this.grid.getCell(LatticeTheoryFactory.createCellIndex(row, column));
				if (cellValue.equals(doubleForFixedObject)) {		
					System.out.print("X");
				}
				
				else if (cellValue.equals(doubleForDynamicObject)) {		
					System.out.print("o");
				}

//				else {
				else if (this.isCellFree(row,column)) {
					System.out.print("_");
				}				
			}
		}
		
		System.out.print("\n");
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#paintLatticeWithValues()
	 */
	@Override
	public void paintLatticeWithValues()  {
		
		int maxRows = this.getNumberOfRows();
		int maxColumns = this.getNumberOfColumns();
		
		for (int row = maxRows-1; row >= 0; row--) {
			
			System.out.print("\n");
			
			for (int column = 0; column < maxColumns; column++) {
				Double cellValue = this.grid.getCell(LatticeTheoryFactory.createCellIndex(row, column));
				if (cellValue.equals(doubleForFixedObject)) {		
					System.out.print(" X ");
				}
				
				else if (cellValue.equals(doubleForDynamicObject)) {		
					System.out.print(" o ");
				}
				
				else {
					int val = cellValue.intValue();
					if (val < 10) {
						System.out.print(" " + cellValue.intValue() + " ");
					}
					else {
						System.out.print(cellValue.intValue() + " ");
					}
						
				}
				
				
			}
		}
		
		System.out.print("\n");
	}
    
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#breshamLineCast(tum.cms.sim.momentum.utility.lattice.CellIndex, tum.cms.sim.momentum.utility.lattice.CellIndex)
	 */
    @Override
	public boolean breshamLineCast(CellIndex from, CellIndex towards) {

    	boolean hitTarget = false;
    	
        int dx = FastMath.abs(from.getColumn() - towards.getColumn());
        int dy = FastMath.abs(from.getRow() - towards.getRow());
        
        if(dy == 0 && dx == 0) {
        	
        	return true;
        }
        
        int x = from.getColumn();
        int y = from.getRow();
        int targetX = towards.getColumn();
        int targetY = towards.getRow();
        
        int n = 1 + dx + dy;
        
        int x_inc = towards.getColumn() > from.getColumn() ? 1 : -1;
        int y_inc = towards.getRow() > from.getRow() ? 1 : -1;
        
        int error = dx - dy;
        dx *= 2;
        dy *= 2;

        for (; n > 0; --n) {
        	
        	if(!this.isCellFree(y, x)) {
        		
        		break;
        	}
        	
        	if(x == targetX && y == targetY) {
        		
        		hitTarget = true;
        		break;
        	}

            if (error > 0) {
            	
                x += x_inc;
                error -= dy;
            }
            else {
            	
                y += y_inc;
                error += dx;
            }
        }
        
    	return hitTarget;
    }
    
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#flood(java.util.List)
	 */
	@Override
	public void flood(List<CellIndex> startingCells) {
		
		Queue<CellIndex> toDoCells = new LinkedList<>();
		toDoCells.addAll(startingCells);
		HashSet<String> doneCells = new HashSet<String>();
		ArrayList<CellIndex> neighbours = null;
		
		while(toDoCells.size() > 0) {
			
			CellIndex current = toDoCells.poll();
			
			if(doneCells.contains(current.getString())) {
			
				continue;
			}
			
			doneCells.add(current.getString());
			this.occupyCell(current, Occupation.Dynamic);
			
			neighbours = this.getAllNeighborIndices(current);
			
			for(CellIndex neighbour : neighbours) {
				
				if(!doneCells.contains(neighbour.getString())) {
				
					if(this.isInLatticeBounds(neighbour) && this.isCellFree(neighbour)) {
						
						toDoCells.add(neighbour);
					}
				}
			}
		}
		
		for (int row = 0; row < this.getNumberOfRows(); row++) {
			
			for (int column = 0; column < this.getNumberOfColumns(); column++){
				
				CellIndex cellIndex = LatticeTheoryFactory.createCellIndex(row, column);
				
				if(this.getCellValue(cellIndex) == Occupation.Empty) {
					
					this.occupyCell(cellIndex, Occupation.Fixed);
				}				
			}
		}

		for (int row = 0; row < this.getNumberOfRows(); row++) {
			
			for (int column = 0; column < this.getNumberOfColumns(); column++){
				
				CellIndex cellIndex = LatticeTheoryFactory.createCellIndex(row, column);
				
				if(this.getCellValue(cellIndex) == Occupation.Dynamic) {
					
					this.freeCell(cellIndex);
				}				
			}
		}
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#flood(tum.cms.sim.momentum.utility.lattice.CellIndex)
	 */
	@Override
	public Set<CellIndex> flood(CellIndex startCell) {
		
		Queue<CellIndex> toDoCells = new LinkedList<>();
		toDoCells.add(startCell);
		
		HashSet<CellIndex> doneCells = new HashSet<CellIndex>();
		ArrayList<CellIndex> neighbours = null;
		
		while(toDoCells.size() > 0) {
			
			CellIndex current = toDoCells.poll();
			
			if(doneCells.add(current) == false) {
				continue;
			}
			
			neighbours = this.getAllNeighborIndices(current);
			
			for(CellIndex neighbour : neighbours) {
				if(!doneCells.contains(neighbour) && this.isCellFree(neighbour) && this.isInLatticeBounds(neighbour)) {
					toDoCells.add(neighbour);
				}
			}
		}
		
		return doneCells;
	}

	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getAllCellsWith(tum.cms.sim.momentum.utility.lattice.Lattice.Occupation)
	 */
	@Override
	public List<CellIndex> getAllCellsWith(Occupation occupation) {
		
		ArrayList<CellIndex> occupationCells = new ArrayList<>();
	
		for (int row = 0; row < this.getNumberOfRows(); row++) {
			
			for (int column = 0; column < this.getNumberOfColumns(); column++){
				
				CellIndex cellIndex = LatticeTheoryFactory.createCellIndex(row, column);
				
				if(this.getCellValue(cellIndex) == occupation) {
					
					occupationCells.add(cellIndex);
				}				
			}
		}
		
		return occupationCells;
	}
	
	/* (non-Javadoc)
	 * @see tum.cms.sim.momentum.utility.lattice.ILattice#getAllCellsWithValue(java.lang.Double)
	 */
	@Override
	public List<CellIndex> getAllCellsWithValue(Double value) {
		
		ArrayList<CellIndex> occupationCells = new ArrayList<>();
	
		for (int row = 0; row < this.getNumberOfRows(); row++) {
			
			for (int column = 0; column < this.getNumberOfColumns(); column++){
				
				CellIndex cellIndex = LatticeTheoryFactory.createCellIndex(row, column);
				
				if(this.getCellNumberValue(cellIndex) == value) {
					
					occupationCells.add(cellIndex);
				}				
			}
		}
		
		return occupationCells;
	}
}