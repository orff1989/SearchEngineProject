public abstract class SearchAlgorithm {
    protected Board board;

    public SearchAlgorithm(Board board) {
        this.board = board;
    }

    public abstract SearchResult search(Board board, Node start, Node goal, String order, boolean time, boolean open);

}

