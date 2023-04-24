import java.util.*;

public class SearchAlgorithms {

    public static class BFS extends SearchAlgorithm {
        private boolean open;

        public BFS(Board board, boolean open) {
            super(board);
            this.open = open;
        }

        @Override
        public SearchResult search(Board board, Node start, Node goal, String order, boolean time, boolean open) {
            long startTime = System.nanoTime();
            int num = 0;
            Queue<Node> queue = new LinkedList<>();
            start.setAction(""); // Set the action of the starting node to an empty string
            queue.add(start);
            Set<Node> closedList = new HashSet<>();

            while (!queue.isEmpty()) {
                Node curr = queue.remove();

                if (curr.equals(goal)) {
                    long endTime = System.nanoTime();
                    double duration = (endTime - startTime) / 1e9;
                    String path = curr.getPath();
                    int cost = curr.getCost();
                    return new SearchResult(path, num, cost, duration);
                }

                closedList.add(curr);

                List<Node> neighbors = board.getNeighbors(curr.getX(), curr.getY(), order);
                for (Node neighbor : neighbors) {
                    if (!closedList.contains(neighbor) && !queue.contains(neighbor)) {
                        neighbor.setParent(curr);
                        neighbor.setCost(curr.getCost() + curr.costTo(neighbor,board));
                        neighbor.setAction(getAction(curr, neighbor)); // Set the action for the neighbor node
                        queue.add(neighbor);
                        num++;
                    }
                }

                if (this.open) {
                    System.out.println("Open list: " + queue);
                }
            }

            return null; // no path found
        }

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

