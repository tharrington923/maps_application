/**
 * @author UCSD MOOC development team and Thomas Harrington
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


//import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
//import java.util.ListIterator;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
//import java.util.Stack;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;
//import week3example.classs;
//import week3example.start;
//import week3example.MazeNode;

/**
 * @author UCSD MOOC development team and Thomas Harrington
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	/**
	 * Create private variables to hold number of vertices,
	 * number of edges, and the vertices (intersections) of
	 * graph
	 */
	private int numVertices; 
	private int numEdges;
	private HashMap<GeographicPoint,MapNode> pointNodeMap;
	//public HashMap<GeographicPoint,MapNode> vertices;
	private HashSet<MapEdge> edges;
	
	/** 
	 * Create a new empty MapGraph by creating a constructor which
	 * initializes the member variables.
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		this.numEdges = 0;
		this.numVertices = 0;
		//this.vertices = new HashMap<GeographicPoint, MapNode>();
		pointNodeMap = new HashMap<GeographicPoint, MapNode>();
		edges = new HashSet<MapEdge>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 * 
	 * @author Thomas Harrington
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return this.numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 * @author Thomas Harrington
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		// I did not use this method in my implementation.
		//return this.vertices.keySet();
		return pointNodeMap.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 * @author Thomas Harrington
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return this.numEdges;
	}

	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 * @author Thomas Harrington
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3
		// Check for null location and see if node already exists
		if(location == null || pointNodeMap.containsKey(location)){
			return false;
		}
		// Node is not null and does not exist, so we create it, add to vertices list
		// and increment the number of vertices by one.
		else {
			MapNode mapNode = new MapNode(location);
			pointNodeMap.put(location, mapNode);
			this.numVertices++;
			return true;
		}
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 3
		// Check null precondition
		if(from == null || to == null){
			throw new IllegalArgumentException();
		}
		// Check the existence of vertices precondition
		else if(!pointNodeMap.containsKey(from) || !pointNodeMap.containsKey(to)){
			throw new IllegalArgumentException("");
		}
		// Check the distance precondition
		else if(pointNodeMap.get(from).distance(to)< 0){
			throw new IllegalArgumentException("Invalid distance specified");
		}
		// Begin the procedure to add the new edge.
		else {
			// Get the node of the starting point (from)
			MapNode fromNode = pointNodeMap.get(from);
			// Create the edge between the from and to nodes using the MapEdge constructor
			// See MapEdge.java for more details about the constructor.
			MapEdge newEdge = new MapEdge(from,to,roadName,roadType,length);
			edges.add(newEdge);
			/**
			 *  Use MapNode's addNeighbor method to add neighbor to the MapNode's list of edges
			 *  See MapNode.java for definition of this method and list.
			 */
			fromNode.addNeighbor(newEdge);
			this.numEdges++;
		}
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 * @author Thomas Harrington
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// If either start or goal location is null, return null
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			//return new LinkedList<GeographicPoint>();
			return null;
		}
		
		// MapNodes corresponding to the start and goal locations
		MapNode startNode = pointNodeMap.get(start);
		MapNode goalNode = pointNodeMap.get(goal);
		
		// Queue of MapNodes to explore (Queue makes it breadth first search!)
		Queue<MapNode> toExplore = new LinkedList<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		
		// Initialize the variables needed in this method. 
		// List to contain all GeographicPoints on the path from start to goal
		List<GeographicPoint> pathList = new LinkedList<GeographicPoint>();
		
		// Add the start location to the path list
		pathList.add(start);
		
		// Checking if start location is equal to goal location, if so return pathList
		if(start == goal){
			return pathList;
		}
		
		// Mark the startNode as visited and add to the list of MapNodes toExplore
		//startNode.wasVisited();
		visited.add(startNode);
		toExplore.add(startNode);
		
		// Boolean to keep track of if the goal location was reached.
		boolean found = false;
		
		MapNode curr = null;
		
		// Continuing until there are no more MapNodes to explore
		while (!toExplore.isEmpty()) {
			// Process next MapNode in the queue
			curr = toExplore.remove();
			GeographicPoint currLocation = curr.getLocation();
			
			// hook for visualization
			nodeSearched.accept(currLocation);
			
			if (pointNodeMap.get(currLocation) == goalNode) {
				found = true; // Goal location was reached
				break; // Can exit while loop
			}
			// Get list of neighbors using MapNode method getNeighbors(). See MapNode class for
			// implementation details.
			List<MapEdge> neighbors = curr.getNeighbors();
			
			// Method to process each neighbor in the list of neighbors. Implementation details below. 
			for(MapEdge m : neighbors){
				MapNode mNode = pointNodeMap.get(m.getEndPoint());
				if(!visited.contains(mNode)){
					visited.add(mNode);
					parentMap.put(mNode, curr);
					//mNode.setParent(curr);
					toExplore.add(mNode);
				}
			}
		}
		
		// If the goal location was never reached.
		if (!found) {
			System.out.println("No path exists from "+start+" to "+goal);
			return null;
		}
		// Construct Path from startNode to goalNode
		return reconstructPath(parentMap,startNode,goalNode);
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 * @author Thomas Harrington
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		int numNodesVisited = 0;
		
		// If either start or goal location is null, return null
		if(start == null || goal == null){
			System.out.println("Start or goal node is null!  No path exists.");
			return null;
		}
		
		MapNode startNode = pointNodeMap.get(start);
		MapNode goalNode = pointNodeMap.get(goal);
		
		// Initialize the variables needed in this method. 
		// List to contain all GeographicPoints on the path from start to goal
		List<GeographicPoint> pathList = new LinkedList<GeographicPoint>();
				
		// Add the start location to the path list
		pathList.add(start);
				
		// Checking if start location is equal to goal location, if so return pathList
		if(start == goal){
			return pathList;
		}
		
		
		/* Initialize variables needed */
		
		HashSet<MapNode> visited = new HashSet<MapNode>();
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		HashMap<MapNode,Double> distanceMap = new HashMap<MapNode,Double>();
		
		PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>(new Comparator<MapNode>(){
			public int compare(MapNode one, MapNode two){
				if(distanceMap.get(one) > distanceMap.get(two)){
					return 1;
				}
				else if(distanceMap.get(one) < distanceMap.get(two)){
					return -1;
				}
				return 0;
			}
		});
		
		distanceMap.put(startNode, 0.0);
		toExplore.offer(startNode);
		
		MapNode curr = null;
		while(!toExplore.isEmpty()){
			curr = toExplore.poll();
			numNodesVisited++;
			if(!visited.contains(curr)){
				if(!distanceMap.containsKey(curr)){
					distanceMap.put(curr, Double.MAX_VALUE);
				}
				nodeSearched.accept(curr.getLocation());
				visited.add(curr);
				if(pointNodeMap.get(curr.getLocation()) == goalNode){
					System.out.println("Dijkstra = " + numNodesVisited);
					return reconstructPath(parentMap, startNode, goalNode);
				}
				List<MapEdge> neighbors = curr.getNeighbors();
				
				// Method to process each neighbor in the list of neighbors
				// For each neighbor
				//Set<MapEdge> neighbors = curr.getEdgeNeighbors();
				for(MapEdge neighbor : neighbors){
					MapNode mNode = pointNodeMap.get(neighbor.getEndPoint());
					if(!distanceMap.containsKey(mNode)){
						distanceMap.put(mNode, Double.MAX_VALUE);
					}
					//System.out.println(mNode.getShortestDistance());
					// Not in visited set
					if(!visited.contains(mNode)){
						double newPathLength = distanceMap.get(curr) + neighbor.getEdgeLength();
						if (newPathLength < distanceMap.get(mNode)){
							distanceMap.replace(mNode, newPathLength);
							parentMap.put(mNode, curr);
							toExplore.offer(mNode);
						}
					}
				}
				
			}
		}
		System.out.println("No path found from " +start+ " to " + goal);
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearchM(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearchM(start, goal, temp);
	}
	
	/** Reconstruct a path from start to goal using the parentMap
	 *
	 * @param parentMap the HashNode map of children and their parents
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */
	private List<GeographicPoint>
	reconstructPath(HashMap<MapNode,MapNode> parentMap,
					MapNode start, MapNode goal)
	{
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode current = goal;

		while (!current.equals(start)) {
			path.addFirst(current.getLocation());
			current = parentMap.get(current);
		}

		// add start
		path.addFirst(start.getLocation());
		return path;
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 *   
	 * @author Thomas Harrington
	 */
	public List<GeographicPoint> aStarSearchM(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
		{
		// TODO: Implement this method in WEEK 4
		
		// If either start or goal location is null, return null
		if(start == null || goal == null){
			System.out.println("Start or goal node is null!  No path exists.");
			return null;
		}
		
		int numNodesVisited = 0;
		
		MapNode startNode = pointNodeMap.get(start);
		MapNode goalNode = pointNodeMap.get(goal);
		
		// Initialize the variables needed in this method. 
		// List to contain all GeographicPoints on the path from start to goal
		List<GeographicPoint> pathList = new LinkedList<GeographicPoint>();
				
		// Checking if start location is equal to goal location, if so return pathList
		if(start == goal){
			// Add the start location to the path list
			pathList.add(start);
			// Add the end location to the path list which equals start location
			pathList.add(start);
			System.out.println("Start location is equal to the goal location");
			return pathList;
		}
		
		/* Initialize variables needed */
		
		HashSet<MapNode> visited = new HashSet<MapNode>();
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		HashMap<MapNode,Double> distanceMap = new HashMap<MapNode,Double>();
		
		/* Note here in A* the priority queue priority accounts for the distance each node is from the goal */
		PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>(new Comparator<MapNode>(){
			public int compare(MapNode one, MapNode two){
				if(distanceMap.get(one) + one.distance(goal) > distanceMap.get(two) + two.distance(goal)){
					return 1;
				}
				else if(distanceMap.get(one) + one.distance(goal) < distanceMap.get(two) + two.distance(goal)){
					return -1;
				}
				return 0;
			}
		});
		
		distanceMap.put(startNode, 0.0);
		toExplore.offer(startNode);
		
		MapNode curr = null;
		while(!toExplore.isEmpty()){
			curr = toExplore.poll();
			numNodesVisited++;
			if(!visited.contains(curr)){
				if(!distanceMap.containsKey(curr)){
					distanceMap.put(curr, Double.MAX_VALUE);
				}
				nodeSearched.accept(curr.getLocation());
				visited.add(curr);
				if(pointNodeMap.get(curr.getLocation()) == goalNode){
					System.out.println("A* = " + numNodesVisited);
					return reconstructPartialPath(parentMap, startNode, goalNode, goalNode);
				}
				List<MapEdge> neighbors = curr.getNeighbors();
				
				// Method to process each neighbor in the list of neighbors
				// For each neighbor
				for(MapEdge neighbor : neighbors){
					MapNode mNode = pointNodeMap.get(neighbor.getEndPoint());
					if(!distanceMap.containsKey(mNode)){
						distanceMap.put(mNode, Double.MAX_VALUE);
					}
					// Not in visited set
					if(!visited.contains(mNode)){
						double newPathLength = distanceMap.get(curr) + neighbor.getEdgeLength();
						if (newPathLength < distanceMap.get(mNode)){
							distanceMap.replace(mNode, newPathLength);
							parentMap.put(mNode, curr);
							toExplore.offer(mNode);
						}
					}
				}
				
			}
		}
		System.out.println("No path found from " +start+ " to " + goal);
		return pathList;

	}
/**																											*/
	
	/** Reconstruct a partial path from start to goal using the parentMap and previously determined subpath
	 *
	 * @param parentMap the HashNode map of children and their parents
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 * 
	 * @author Thomas Harrington
	 */
	private List<GeographicPoint>
	reconstructPartialPath(HashMap<MapNode,MapNode> parentMap,
					MapNode start, MapNode curr, MapNode goal)
	{
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode current = curr;
		// If curr != goal, then path from curr -> goal exists
		if(!curr.equals(goal)){
			path = curr.getShortestPath(goal);
			// Now we set the current equal to the parent so we determine path from start -> curr.parent
			current = parentMap.get(current);
		}

		while (!current.equals(start)) {
			path.addFirst(current.getLocation());
			current.addShortestPath(goal, path);
			current = parentMap.get(current);
		}

		// Add starting node
		path.addFirst(start.getLocation());
		start.addShortestPath(goal, path);
		return path;
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}

	
	/** Find the path from start to goal using a modified A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 * 
	 * @author Thomas Harrington
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
		{
		// Project extension
		
		// If either start or goal location is null, return null
		if(start == null || goal == null){
			System.out.println("Start or goal node is null!  No path exists.");
			return null;
		}
		
		int numNodesVisited = 0;
		
		MapNode startNode = pointNodeMap.get(start);
		MapNode goalNode = pointNodeMap.get(goal);
		
		/* If the path was already computed */
		if(startNode.hasShortestPath(goalNode)){
			System.out.println("Path was previously computed");
			return startNode.getShortestPath(goalNode);
		}
		
		// Initialize the variables needed in this method. 
				
		// Checking if start location is equal to goal location, if so return pathList
		if(start == goal){
			System.out.println("Start goal is equal to end goal");
			return null;
		}
		
		/* Initialize variables needed */
		
		HashSet<MapNode> visited = new HashSet<MapNode>();
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		HashMap<MapNode,Double> distanceMap = new HashMap<MapNode,Double>();
		
		/* Note here in A* the priority queue priority accounts for the distance each node is from the goal */
		PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>(new Comparator<MapNode>(){
			public int compare(MapNode one, MapNode two){
				if(distanceMap.get(one) + one.distance(goal) > distanceMap.get(two) + two.distance(goal)){
					return 1;
				}
				else if(distanceMap.get(one) + one.distance(goal) < distanceMap.get(two) + two.distance(goal)){
					return -1;
				}
				return 0;
			}
		});
		
		distanceMap.put(startNode, 0.0);
		toExplore.offer(startNode);
		
		MapNode curr = null;
		while(!toExplore.isEmpty()){
			curr = toExplore.poll();
			numNodesVisited++;
			if(!visited.contains(curr)){
				if(!distanceMap.containsKey(curr)){
					distanceMap.put(curr, Double.MAX_VALUE);
				}
				nodeSearched.accept(curr.getLocation());
				visited.add(curr);
				if(curr.hasShortestPath(goalNode)){
					System.out.println("Path from " + curr + " to " + goalNode + " has been calculated");
					return reconstructPartialPath(parentMap,startNode, curr, goalNode);
				}
				if(pointNodeMap.get(curr.getLocation()) == goalNode){
					System.out.println("A* = " + numNodesVisited);
					return reconstructPartialPath(parentMap,startNode, curr, goalNode);
				}
				List<MapEdge> neighbors = curr.getNeighbors();
				
				// Method to process each neighbor in the list of neighbors
				// For each neighbor
				for(MapEdge neighbor : neighbors){
					MapNode mNode = pointNodeMap.get(neighbor.getEndPoint());
					if(!distanceMap.containsKey(mNode)){
						distanceMap.put(mNode, Double.MAX_VALUE);
					}
					// Not in visited set
					if(!visited.contains(mNode)){
						double newPathLength = distanceMap.get(curr) + neighbor.getEdgeLength();
						if (newPathLength < distanceMap.get(mNode)){
							distanceMap.replace(mNode, newPathLength);
							parentMap.put(mNode, curr);
							toExplore.offer(mNode);
						}
					}
				}
				
			}
		}
		System.out.println("No path found from " +start+ " to " + goal);
		return null;

	}
	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		//List<GeographicPoint> testroute3 = simpleTestMap.aStarSearchModified(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		
		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

	}
	
}
