/**
 * @author UCSD MOOC development team and Thomas Harrington
 *
 */

package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	// Initialize the MapEdge member variables. Variable names describe information being stored.
	private GeographicPoint startPoint;
	private GeographicPoint endPoint;
	private String edgeRoadName;
	private String edgeRoadType;
	private double edgeLength;
	
	static final double DEFAULT_LENGTH = 0.01;
	
	// Constructor Method for the MapEdge class
	MapEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length){
		this.startPoint = from;
		this.endPoint = to;
		this.edgeRoadName = roadName;
		this.edgeRoadType = roadType;
		this.edgeLength = length;
	}
	
	/**
	 * 
	 * Methods below are used to return or set the private member variables of the MapEdge class.
	 *
	 */
	
	public GeographicPoint getEndPoint(){
		return this.endPoint;
	}
	
	public GeographicPoint getStartPoint(){
		return this.startPoint;
	}
	
	public String getEdgeRoadName(){
		return this.edgeRoadName;
	}
	
	public String getEdgeRoadType(){
		return this.edgeRoadType;
	}
	
	public double getEdgeLength(){
		return this.edgeLength;
	}
}
