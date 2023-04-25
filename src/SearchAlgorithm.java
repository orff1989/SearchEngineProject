public abstract class SearchAlgorithm {
    protected Board board;
    protected boolean open;
    protected String order;

    public SearchAlgorithm(Board board, boolean open, String order) {
        this.board = board;
        this.open=open;
        this.order=order;
    }

    public abstract SearchResult search(Node start, Node goal);

}

