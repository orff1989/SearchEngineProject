import java.util.*;
import java.util.stream.Collectors;

public class SearchAlgorithms {

    public static class BFS extends SearchAlgorithm {

        public BFS(Board board, boolean open, String order) {
            super(board, open, order);
        }

        @Override
        public SearchResult search(Node start, Node goal) {
            long startTime = System.nanoTime();
            int nodes_counter = 0;
            Queue<Node> openList = new LinkedList<>(); // The nodes that need to be explored
            start.setAction("");
            openList.add(start);
            Set<Node> closedList = new HashSet<>(); // The explored nodes

            while (!openList.isEmpty()) { // While there is nodes to be explored
                Node curr = openList.remove();
                nodes_counter++;

                if (curr.equals(goal)) { // Checking if we found the goal node
                    long endTime = System.nanoTime();
                    double duration = (endTime - startTime) / 1e9;
                    String path = curr.getPath();
                    int cost = curr.getCost();
                    return new SearchResult(path, nodes_counter, cost, duration);
                }
                closedList.add(curr); // Adding the node to the explored nodes list

                // Getting the valid neighbors of the node
                List<Node> neighbors = board.getValidNeighbors(curr.getX(), curr.getY(), order);
                for (Node neighbor : neighbors) {
                    if (!closedList.contains(neighbor) && !openList.contains(neighbor)) { // Check if we saw this node already
                        neighbor.setParent(curr);
                        neighbor.setCost(curr.getCost() + curr.costTo(neighbor,board));
                        neighbor.setAction(getAction(curr, neighbor)); // Set the action for the neighbor node
                        openList.add(neighbor);
                    }
                }
                if (this.open) {
                    System.out.println("Open list: " + openList);
                }
            }
            long endTime = System.nanoTime();
            double duration = (endTime - startTime) / 1e9;
            return new SearchResult("no path", nodes_counter, Integer.MAX_VALUE, duration); // the algorithm did not find any path
        }
    }

    public static class DFID extends SearchAlgorithm {
        private static final String CUTOFF = "CUTOFF";
        private static final String FAIL = "FAIL";

        public DFID(Board board, boolean open, String order) {
            super(board, open, order);
        }

        @Override
        public SearchResult search(Node start, Node goal) {
            long startTime = System.nanoTime();
            int nodes_counter = 0, depth = 1;

            // Loop until getting result
            while (true) {
                Map<Node, Boolean> visited = new HashMap<>();
                SearchResult result = limitedDFS(start, goal, depth, visited);

                if (this.open) {
                    System.out.println("Depth: " + depth + ", Open list: " + visited.keySet());
                }
                nodes_counter += visited.size();

                // Check if the search result is not a cutoff, and return the result accordingly
                if (!result.getPath().equals(CUTOFF)) {
                    long endTime = System.nanoTime();
                    double duration = (endTime - startTime) / 1e9;

                    if (result.getPath().equals(FAIL)) {
                        return new SearchResult("no path", nodes_counter, Integer.MAX_VALUE, duration);
                    } else {
                        return new SearchResult(result.getPath(), nodes_counter, result.getCost(), duration);
                    }
                }
                depth++; // Increasing the depth of the search
            }
        }

        // Recursive limited DFS function
        private SearchResult limitedDFS(Node node, Node goal, int limit, Map<Node, Boolean> visited) {
            // If the current node is the goal, return its path
            if (node.equals(goal)) {
                return new SearchResult(node.getPath(), 0, node.getCost(), 0);
            }
            // If the limit is reached, return a cutoff result
            else if (limit == 0) {
                return new SearchResult(CUTOFF, 0, Integer.MAX_VALUE, 0);
            } else {
                visited.put(node, true);
                boolean isCutoff = false;
                List<Node> neighbors = board.getValidNeighbors(node.getX(), node.getY(), order);

                // Iterate through the neighbors of the current node
                for (Node neighbor : neighbors) {
                    if (!visited.containsKey(neighbor)) {
                        neighbor.setParent(node);
                        neighbor.setCost(node.getCost() + node.costTo(neighbor, board));
                        neighbor.setAction(getAction(node, neighbor));

                        // Perform a recursive limited DFS on the neighbor
                        SearchResult result = limitedDFS(neighbor, goal, limit - 1, visited);

                        // Checking if the search result is cutoff or fail
                        if (result.getPath().equals(CUTOFF)) {
                            isCutoff = true;
                        } else if (!result.getPath().equals(FAIL)) {
                            return result;
                        }
                    }
                }
                // Remove the current node from the visited list
                visited.remove(node);

                // Return a cutoff or fail
                if (isCutoff) {
                    return new SearchResult(CUTOFF, 0, Integer.MAX_VALUE, 0);
                } else {
                    return new SearchResult(FAIL, 0, Integer.MAX_VALUE, 0);
                }
            }
        }
    }

    public static class AStar extends SearchAlgorithm {
        private String preference;

        public AStar(Board board, boolean open, String order, String preference) {
            super(board, open, order);
            this.preference = preference;
        }

        @Override
        public SearchResult search(Node start, Node goal) {
            long startTime = System.nanoTime();
            int nodes_counter = 0;

            // Create a priority queue that sort the nodes based on their Total Cost (f function)
            PriorityQueue<Node> openList = new PriorityQueue<>(createNodeComparator(goal));
            start.setAction("");
            openList.add(start);
            Set<Node> closedList = new HashSet<>();

            while (!openList.isEmpty()) {
                Node curr = openList.remove();
                nodes_counter++;

                // check if we reached to our goal node
                if (curr.equals(goal)) {
                    long endTime = System.nanoTime();
                    double duration = (endTime - startTime) / 1e9;
                    String path = curr.getPath();
                    int cost = curr.getCost();
                    return new SearchResult(path, nodes_counter, cost, duration);
                }
                closedList.add(curr);

                // Get the valid neighbors of the current node
                List<Node> neighbors = board.getValidNeighbors(curr.getX(), curr.getY(), order);
                for (Node neighbor : neighbors) {
                    // Check if we already explored this node
                    if (!closedList.contains(neighbor)) {
                        neighbor.setParent(curr);
                        neighbor.setCost(curr.getCost() + curr.costTo(neighbor, board));
                        neighbor.setAction(getAction(curr, neighbor));

                        if (!openList.contains(neighbor)) {
                            openList.add(neighbor);
                        }
                        else if (neighbor.getCost() > curr.getCost() + curr.costTo(neighbor, board)) {
                            // Update the cost and position of the neighbor in the open list if we found a better path
                            openList.remove(neighbor);
                            openList.add(neighbor);
                        }
                    }
                }
                if (this.open) {
                    System.out.println("Open list: " + openList);
                }
            }
            long endTime = System.nanoTime();
            double duration = (endTime - startTime) / 1e9;
            return new SearchResult("no path", nodes_counter, Integer.MAX_VALUE, duration);
        }

        // A comparator for the nodes based on their f-cost and the preference of exploring older or newer nodes
        private Comparator<Node> createNodeComparator(Node goal) {
            Comparator<Node> fScoreComparator = Comparator.comparingInt(node -> getTotalCost(node, goal, board));
            Comparator<Node> timestampComparator = preference.equals("old-first") ?
                    Comparator.comparingLong(Node::getId) :
                    Comparator.comparingLong(Node::getId).reversed();
            return fScoreComparator.thenComparing(timestampComparator);
        }
    }

    public static class IDAStar extends SearchAlgorithm {

        public IDAStar(Board board, boolean open, String order) {
            super(board, open, order);
        }

        @Override
        public SearchResult search(Node start, Node goal) {
            long startTime = System.nanoTime();
            int nodesCounter = 0;
            int threshold = heuristicFunction(start, goal);
            Map<Node, String> nodeMarkers = new HashMap<>();

            while (threshold != Integer.MAX_VALUE) {
                // Create every iterate of the while loop the L and H structures
                Stack<Node> L = new Stack<>();
                Set<Node> H = new HashSet<>();

                int minF = Integer.MAX_VALUE;

                L.push(start);
                H.add(start);

                // Running until L is empty
                while (!L.isEmpty()) {
                    nodesCounter++;
                    Node n = L.pop();
                    if (nodeMarkers.containsKey(n) && nodeMarkers.get(n).equals("out")) {
                        H.remove(n);
                        nodeMarkers.remove(n);
                    } else {
                        nodeMarkers.put(n, "out");
                        L.push(n);

                        // Getting the valid neighbors node from n
                        List<Node> neighbors = board.getValidNeighbors(n.getX(), n.getY(), order);
                        for (Node neighbor : neighbors) {
                            neighbor.setParent(n);
                            neighbor.setCost(n.getCost() + n.costTo(neighbor, board));
                            neighbor.setAction(getAction(n, neighbor));

                            // Getting the value of the f function
                            int f = getTotalCost(neighbor, goal, board);

                            if (f > threshold) {
                                minF = Math.min(minF, f);
                                continue;
                            }

                            if (H.contains(neighbor) && nodeMarkers.containsKey(neighbor) && nodeMarkers.get(neighbor).equals("out")) {
                                continue;
                            }

                            if (H.contains(neighbor) && !nodeMarkers.get(neighbor).equals("out")) {
                                Node existingNeighbor = getNodeFrom(neighbor,H);

                                if (existingNeighbor != null) {
                                    // Checking which f-cost is bigger
                                    if (getTotalCost(existingNeighbor, goal, board) > f) {
                                        L.remove(existingNeighbor);
                                        H.remove(existingNeighbor);
                                    } else {
                                        continue;
                                    }
                                }
                            }
                            // Check if we reached to the goal node
                            if (goal.equals(neighbor)) {
                                String path = neighbor.getPath();
                                int cost = neighbor.getCost();
                                long endTime = System.nanoTime();
                                double duration = (endTime - startTime) / 1e9;
                                return new SearchResult(path, nodesCounter, cost, duration);
                            }

                            L.push(neighbor);
                            H.add(neighbor);
                        }
                    }
                    if(open){
                        System.out.println("Open list: " + L);
                    }
                }
                threshold = minF;
            }

            long endTime = System.nanoTime();
            double duration = (endTime - startTime) / 1e9;
            return new SearchResult("no path", nodesCounter, Integer.MAX_VALUE, duration);
        }

    }

    public static class DFBnB extends SearchAlgorithm {
        private String preference;

        public DFBnB(Board board, boolean open, String order, String preference) {
            super(board, open, order);
            this.preference = preference;
        }

        @Override
        public SearchResult search(Node start, Node goal) {
            long startTime = System.nanoTime();

            Stack<Node> L = new Stack<>();
            Set<Node> H = new HashSet<>();
            Map<Node, String> nodeMarkers = new HashMap<>();

            L.push(start);
            H.add(start);

            int t = Integer.MAX_VALUE;
            int nodesCounter = 0;

            // Iterating while the L is not empty
            while (!L.isEmpty()) {
                Node n = L.pop();
                nodesCounter++;

                if (nodeMarkers.get(n) != null && nodeMarkers.get(n).equals("out")) {
                    H.remove(n);
                } else {
                    nodeMarkers.put(n, "out");
                    L.push(n);

                    // Getting the neighbors of n node
                    List<Node> N = board.getValidNeighbors(n.getX(), n.getY(), order);

                    // Sorting the neighbors Node list by the f function
                    Comparator<Node> comparator = Comparator.comparingInt(node -> getTotalCost(node, goal, board));
                    if ("new-first".equals(preference)) {
                        comparator = comparator.thenComparing((node1, node2) -> Math.toIntExact(node2.getId() - node1.getId()));
                    } else {
                        comparator = comparator.thenComparing((node1, node2) -> Math.toIntExact(node1.getId() - node2.getId()));
                    }
                    N.sort(comparator);

                    // Iterating over the neighbors nodes
                    for (Iterator<Node> it = N.iterator(); it.hasNext(); ) {
                        Node g = it.next();
                        g.setParent(n);
                        g.setCost(n.getCost() + n.costTo(g, board));
                        g.setAction(getAction(n, g));

                        int f_g = getTotalCost(g, goal, board);

                        // Checking the total cost
                        if (f_g >= t) {
                            it.remove();
                            while (it.hasNext()) {
                                it.next();
                                it.remove();
                            }
                        } else if (H.contains(g) && nodeMarkers.get(g) != null && nodeMarkers.get(g).equals("out")) {
                            it.remove();
                        } else if (H.contains(g) && (nodeMarkers.get(g) == null || !nodeMarkers.get(g).equals("out"))) {
                            Node gPrime = getNodeFrom(g, H);
                            if (gPrime != null && getTotalCost(gPrime, goal, board) <= f_g) {
                                it.remove();
                            } else {
                                L.remove(g);
                                H.remove(g);
                            }
                            // Checking if we reach to the goal node
                        } else if (g.equals(goal)) {
                            String path = g.getPath();
                            int cost = g.getCost();
                            long endTime = System.nanoTime();
                            double duration = (endTime - startTime) / 1e9;
                            return new SearchResult(path, nodesCounter, cost, duration);
                        }
                    }
                    // Inserting the N nodes members in reverse order to H and L
                    Collections.reverse(N);
                    L.addAll(N);
                    H.addAll(N);
                }
                if(open){
                    System.out.println("Open list: " + L);
                }
            }

            long endTime = System.nanoTime();
            double duration = (endTime - startTime) / 1e9;
            return new SearchResult("no path", nodesCounter, Integer.MAX_VALUE, duration);
        }
    }


    // The f function
    private static int getTotalCost(Node node, Node goal_node, Board board) {
        int gCost = node.getCost();
        int hCost = heuristicFunction(node, goal_node);
        return gCost + hCost;
    }

    // The heuristic function - h function
    private static int heuristicFunction(Node current, Node goal) {
        int dx = Math.abs(current.getX() - goal.getX());
        int dy = Math.abs(current.getY() - goal.getY());
        int minimumCost1 = 1; // Minimum cost of moving horizontally/vertically
        int minimumCost2 = 1; // Minimum cost of moving diagonally
        return minimumCost1 * (dx + dy) + (minimumCost2 - 2 * minimumCost1) * Math.min(dx, dy);
    }

    // This method returns the node in the set h that equal to given node n
    private static Node getNodeFrom(Node n, Set<Node> h) {
        for (Node node : h) {
            if (node.equals(n)) return node;
        }
        return null;
    }

    // This method returns the direction from the curr node to its neighbor
    private static String getAction(Node current, Node neighbor) {
        int dx = neighbor.getX() - current.getX();
        int dy = neighbor.getY() - current.getY();

        if (dx == 0 && dy == 1) {
            return "R";
        } else if (dx == 1 && dy == 1) {
            return "RD";
        } else if (dx == 1 && dy == 0) {
            return "D";
        } else if (dx == 1 && dy == -1) {
            return "LD";
        } else if (dx == 0 && dy == -1) {
            return "L";
        } else if (dx == -1 && dy == -1) {
            return "LU";
        } else if (dx == -1 && dy == 0) {
            return "U";
        } else if (dx == -1 && dy == 1) {
            return "RU";
        } else {
            return "";
        }
    }
}

