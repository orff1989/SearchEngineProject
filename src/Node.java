public class Node {
    int x; // Row
    int y; // Column
    int cost;
    Node parent;
    String action;


    public Node(int x, int y, int cost, Node parent, String action) {
        this.x = x;
        this.y = y;
        this.cost = cost;
        this.parent = parent;
        this.action = action;
    }

    public int getX() {
        return x;
    }

    public int getY() {
        return y;
    }

    public int getCost() {
        return cost;
    }

    public Node getParent() {
        return parent;
    }

    public String getAction() {
        return action;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null) return false;
        if (getClass() != obj.getClass()) return false;
        Node other = (Node) obj;
        return x == other.x && y == other.y;
    }

    public void setCost(int cost) {
        this.cost = cost;
    }

    public String getPath() {
        String path = "";
        Node currentNode = this;
        while (currentNode.parent != null) {
            path = "-" + currentNode.action + path;
            currentNode = currentNode.parent;
        }
        return path.length() > 0 ? path.substring(1) : path;
    }

    public void setAction(String s) {
        this.action=new String(s);
    }

    public void setParent(Node n) {
        this.parent=n;
    }

    public int costTo(Node neighbor, Board board) {
        int deltaX = Math.abs(neighbor.x - this.x);
        int deltaY = Math.abs(neighbor.y - this.y);
        int baseCost = board.getCellCost(neighbor.x, neighbor.y);
        if (deltaX == 1 && deltaY == 1 && "H".equals(board.getMap()[neighbor.x - 1][neighbor.y - 1])) {
            return baseCost + 5;
        }
        return baseCost;
    }

    @Override
    public String toString() {
        return "Node{" +
                "x=" + x +
                ", y=" + y +
                ", cost=" + cost +"}";
    }
}
