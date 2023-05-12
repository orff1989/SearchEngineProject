import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.Set;

public class Board {
    private int size;
    private String[][] map;

    public Board(int size, String[][] map) {
        this.size = size;
        this.map = map;
    }


    public int getCellCost(int x, int y) {
        String cell = map[x - 1][y - 1];
        switch (cell) {
            case "D":
                return 1;
            case "R":
                return 3;
            case "H":
                return 5;
            case "X":
                return Integer.MAX_VALUE;
            case "S":
                return 0;
            case "G":
                return 5;
            default:
                return 0;
        }
    }

    public boolean isValidMove(int x, int y, Node parent) {
        if (x < 1 || x > size || y < 1 || y > size || (parent!=null && x == parent.getX() && y == parent.getY())) {
            return false;
        }
        return !map[x - 1][y - 1].equals("X");
    }

    public List<Node> getValidNeighbors(Node curr, String order, Queue<Node> openList) {
        int x= curr.getX(), y= curr.getY();
        List<Node> neighbors = new ArrayList<>();
        int[][] directions;

        if (order.equals("clockwise")) {
            directions = new int[][]{{0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};
        } else {
            directions = new int[][]{{0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}, {1, 0}, {1, 1}};
        }
        for (int[] direction : directions) {
            int newX = x + direction[0];
            int newY = y + direction[1];

            if (openList==null){
                if (isValidMove(newX, newY, curr.getParent())) {
                    neighbors.add(new Node(newX, newY, getCellCost(newX, newY), curr, ""));
                }
            }else {
                if (isValidMove(newX, newY, curr.getParent()) && !isVisited(newX, newY, openList)) {
                    neighbors.add(new Node(newX, newY, getCellCost(newX, newY), curr, ""));
                }
            }

        }
        return neighbors;
    }


    private boolean isVisited(int x, int y, Queue<Node> visited){
        for (Node n: visited) {
            if(n.getX()==x && n.getY()==y) return true;
        }
        return false;
    }

    public String[][] getMap() {
        return map;
    }

    public String getCellType(int x, int y) {
        return map[x][y];
    }
}
