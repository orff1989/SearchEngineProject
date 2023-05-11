import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.StringTokenizer;

public class Ex1 {
    public static void main(String[] args) {
        try {
            // Reading the file
            BufferedReader reader = new BufferedReader(new FileReader("input.txt"));
            String algorithm = reader.readLine();
            String order = reader.readLine();
            boolean time = reader.readLine().equalsIgnoreCase("with time");
            boolean open = reader.readLine().equalsIgnoreCase("with open");
            int size = Integer.parseInt(reader.readLine());

            // Setting the files values in variables
            String line = reader.readLine();
            StringTokenizer st = new StringTokenizer(line, "(),");
            int startX = Integer.parseInt(st.nextToken());
            int startY = Integer.parseInt(st.nextToken());
            int goalX = Integer.parseInt(st.nextToken());
            int goalY = Integer.parseInt(st.nextToken());

            // Creating the map
            String[][] map = new String[size][size];
            for (int i = 0; i < size; i++) {
                line = reader.readLine();
                for (int j = 0; j < size; j++) {
                   map[i][j]=line.charAt(j)+"";
                }
            }

            Board board = new Board(size, map);
            Node start = new Node(startX, startY, 0, null, "");
            Node goal = new Node(goalX, goalY, 0, null, "");

            SearchAlgorithm searchAlgorithm;
            String preference = null;
            if (algorithm.equals("A*") || algorithm.equals("DFBnB")) {
                preference = order.split(" ")[1];
                order = order.split(" ")[0];
            }

            // Searching by the giving type of algorithm
            switch (algorithm) {
                case "BFS":
                    searchAlgorithm = new SearchAlgorithms.BFS(board, open, order);
                    break;
                case "DFID":
                    searchAlgorithm = new SearchAlgorithms.DFID(board, open, order);
                    break;
                case "A*":
                    searchAlgorithm = new SearchAlgorithms.AStar(board, open, order, preference);
                    break;
                case "IDA*":
                    searchAlgorithm = new SearchAlgorithms.IDAStar(board, open, order);
                    break;
                case "DFBnB":
                    searchAlgorithm = new SearchAlgorithms.DFBnB(board, open, order, preference);
                    break;
                default:
                    throw new IllegalArgumentException("Invalid search algorithm");
            }

            SearchResult result = searchAlgorithm.search(start, goal);

            // Writing the search result to the output file
            FileWriter writer = new FileWriter("output.txt");

            writer.write(result.getPath());
            writer.write("\nNum: " + result.getNumNodes());

            if (result.getCost()==Integer.MAX_VALUE) writer.write("\nCost: inf");
            else writer.write("\nCost: " + result.getCost());

            if (time) {
                writer.write("\n" + result.getTime() + " seconds");
            }

            writer.close();

        } catch (IOException e){
            e.printStackTrace();
        }
    }
}

