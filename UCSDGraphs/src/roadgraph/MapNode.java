/**
 * @author UCSD MOOC development team and Thomas Harrington
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */

package roadgraph;

import geography.GeographicPoint;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

public class MapNode implements Comparable<MapNode> {
	/* Initialize the MapNode member variables*/
	private GeographicPoint location;
	
	/**/
	//private MapNode parentNode; // Node to keep track of parent to reconstruct paths
	
	
	/* HashMap to keep track of shortest paths from this node to a destination node */
	private HashMap<MapNode,LinkedList<GeographicPoint>> shortestPaths;
	
	private double shortestDistance;
	//private Boolean visited; // Boolean to mark if it has been visited in a search method
	List<MapEdge> neighbors; // Edges starting at location and ending at another mapNode
	
	/*HashSet to keep a list of edges out of this node*/
	private HashSet<MapEdge> edges;
	
	// Constructor that creates a new MapNode with a specified location
	public MapNode(GeographicPoint addPoint){
		this.location = addPoint;
		edges = new HashSet<MapEdge>();
		shortestDistance = Double.MAX_VALUE;
		//this.visited = false;
		this.neighbors = new ArrayList<MapEdge>();
		shortestPaths = new HashMap<MapNode,LinkedList<GeographicPoint>>();
	}
	
	// Returns the distance between this MapNode and another GeographicPoint
	public double distance(GeographicPoint other){
		return this.location.distance(other);
	}
	
	// Method that adds a MapEdge to the list of neighbors
	public void addNeighbor(MapEdge newEdge){
		this.neighbors.add(newEdge);
		edges.add(newEdge);
	}
	
	// Method that returns a list of MapEdges starting from the MapNode
	public List<MapEdge> getNeighbors(){
		return this.neighbors;
	}
	
	// Method that returns a list of MapEdges starting from the MapNode
	public Set<MapEdge> getEdgeNeighbors(){
		return this.edges;
	}
	
	
	// Method that returns the MapNode location  
	public GeographicPoint getLocation(){
		 return this.location;
	}
	
	public int compareTo(MapNode other){
		if(other.shortestDistance < this.shortestDistance){
			return 1;
		}
		else if(other.shortestDistance == this.shortestDistance){
			return 0;
		}
		return -1;
	}
	
	/** Returns whether two nodes are equal.
	 * Nodes are considered equal if their locations are the same, 
	 * even if their street list is different.
	 * @param o the node to compare to
	 * @return true if these nodes are at the same location, false otherwise
	 */
	@Override
	public boolean equals(Object o)
	{
		if (!(o instanceof MapNode) || (o == null)) {
			return false;
		}
		MapNode node = (MapNode)o;
		return node.location.equals(this.location);
	}
	
	public void setShortestDistance(double value){
		shortestDistance = value;
	}
	
	public double getShortestDistance(){
		return shortestDistance;
	}
	
	/** ToString to print out a MapNode object
	 *  @return the string representation of a MapNode
	 */
	@Override
	public String toString()
	{
		String toReturn = "[NODE at location (" + location + ")";
		toReturn += " intersects streets: ";
		for (MapEdge e: edges) {
			toReturn += e.getEdgeRoadName() + ", ";
		}
		toReturn += "]";
		toReturn += "Actual: "+ this.shortestDistance;
		return toReturn;
	}
	
	/* Method to determine if path from this node to goal node has been computed */
	public boolean hasShortestPath(MapNode goal){
		return this.shortestPaths.containsKey(goal);
	}
	
	/* Method to add path from this node to goal node */
	public void addShortestPath(MapNode goal, LinkedList<GeographicPoint> path){
		if (!path.peekFirst().equals(this.location)){
			System.out.println("Path starting point is not equal to this node");
			System.out.println(this.location.toString());
			System.out.println(path.peekFirst().toString());
		}
		//shortestPaths.
		LinkedList<GeographicPoint> copy = (LinkedList<GeographicPoint>) path.clone();
		shortestPaths.put(goal, copy);
	}
	
	/* Method to get shortest path from this node to goal node */
	public LinkedList<GeographicPoint> getShortestPath(MapNode goal){
		if(this.hasShortestPath(goal)){
			return shortestPaths.get(goal);
		}
		else{
			System.out.println("There is no known path from " + this.toString() + " to " + goal.toString());
			LinkedList<GeographicPoint> emptyPath = new LinkedList<GeographicPoint>();
			return emptyPath; 
		}
	}


}
