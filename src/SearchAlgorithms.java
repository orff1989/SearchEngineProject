import java.util.*;

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
        /*
        The Octile distance heuristic makes the A* algorithm optimal because it never overestimates the cost to reach the goal.
        This property is called admissibility, and it ensures that the A* algorithm will find the shortest path when using an admissible heuristic.
        The Octile distance heuristic is also consistent (or monotonic), which means that it satisfies the triangle inequality,
        and the cost of any node along the path is always less than or equal to the estimated cost of moving to an adjacent node
        plus the actual cost of moving to that adjacent node. Consistency ensures that A* is optimally efficient.
        */
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
            Comparator<Node> fScoreComparator = Comparator.comparingInt(node -> getTotalCost(node, goal));
            Comparator<Node> timestampComparator = preference.equals("old-first") ?
                    Comparator.comparingLong(Node::getId) :
                    Comparator.comparingLong(Node::getId).reversed();
            return fScoreComparator.thenComparing(timestampComparator);
        }

        // The f function
        private int getTotalCost(Node node, Node goal_node) {
            int gCost = node.getCost();
            int hCost = heuristicFunction(node, goal_node);
            return gCost + hCost;
        }

        // Calculates the heuristic cost of moving from one node to another
        private int heuristicFunction(Node a, Node b) {
            int dx = Math.abs(a.getX() - b.getX());
            int dy = Math.abs(a.getY() - b.getY());
            int diagonalSteps = Math.min(dx, dy);
            int straightSteps = Math.max(dx, dy) - diagonalSteps;
            int diagonalCost = 14; // Cost of diagonal movement (assuming cost of straight movement is 10)
            return diagonalCost * diagonalSteps + 10 * straightSteps;
        }
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

