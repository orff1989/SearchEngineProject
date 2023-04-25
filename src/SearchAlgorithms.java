import java.util.*;

public class SearchAlgorithms {

    public static class BFS extends SearchAlgorithm {
        /*
         It explores nodes in a breadth-wise fashion, starting from the root node and moving outward,
         expanding all neighboring nodes before moving to the next level of neighbors.
         admissible: BFS is optimal as it guarantees finding the shortest path by exploring nodes layer by layer.
         It ensures the solution with the fewest steps.
         consistent: BFS is complete because it systematically searches all nodes,
         ensuring a solution is found if one exists, as nodes are explored by depth.
        */
        private boolean open;

        public BFS(Board board, boolean open) {
            super(board);
            this.open = open;
        }

        @Override
        public SearchResult search(Board board, Node start, Node goal, String order, boolean time, boolean open) {
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
                    if (!closedList.contains(neighbor) && !openList.contains(neighbor)) {
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

        // This method returns the direction from the curr node to its neighbor
        private String getAction(Node current, Node neighbor) {
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

}

