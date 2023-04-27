public class SearchResult {
    private String path;
    private int numNodes;
    private int cost;
    private double time;

    public SearchResult(String path, int numNodes, int cost, double time) {
        this.path = path;
        this.numNodes = numNodes;
        this.cost = cost;
        this.time = time;
    }

    public String getPath() {
        return path;
    }

    public int getNumNodes() {
        return numNodes;
    }

    public int getCost() {
        return cost;
    }

    public double getTime() {
        return time;
    }
}
