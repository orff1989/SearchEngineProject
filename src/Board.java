import java.util.ArrayList;
import java.util.List;

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
                return 0;
            case "S":
                return 0;
            case "G":
                return 5;
            default:
                return 0;
        }
    }

    public boolean isValidMove(int x, int y) {
        if (x < 1 || x > size || y < 1 || y > size) {
            return false;
        }
        return !map[x - 1][y - 1].equals("X");
    }

    public List<Node> getValidNeighbors(int x, int y, String order) {
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

            if (isValidMove(newX, newY)) {
                neighbors.add(new Node(newX, newY, getCellCost(newX, newY), null, ""));
            }
        }
        return neighbors;
    }

    public String[][] getMap() {
        return map;
    }
}
