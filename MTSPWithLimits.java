import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.PriorityQueue;

/**
 * MTSPWithLimits 类实现了多个无人机巡航的 mTSP 问题，使用贪心算法，目的是总用时最短（即走最长路径的无人机的路径应该尽可能短）
 * 同时增加了每个无人机的最大飞行距离限制，并尽可能最大化利用无人机，以减少无人机的使用数量。
 * 使用方法：
 * int[][] paths = MTSPWithLimits.solveMTSP(data, numberOfDrones, limitDistance);
 * eg.  0 号无人机需要经过 0 3 4 2 6 -> path[0] = {0,3,4,2,6}
 */
public class MTSPWithLimits {
    /*
     *    * solveMTSP 方法实现了解决多个无人机巡航的 mTSP 问题
     *    * @param data 数据对象，包括距离矩阵和出发点
     *    * @param numberOfDrones 无人机数量
     *    * @param limitDistance 每个无人机的最大限制飞行距离
     *    * @return 返回一个包含各无人机路径的二维数组
     *    * @throws Exception 当出现错误时抛出异常
     */
    public static int[][] solveMTSP(Data data, int numberOfDrones, double limitDistance) {
        int[][] paths = new int[numberOfDrones][];
        double[] distances = new double[numberOfDrones];

        double[][] adjacencyMatrix = calculateDistances(data);

        PriorityQueue<Edge> edgeQueue = new PriorityQueue<>((a, b) -> {
            if (a.distance < b.distance) return -1;
            else if (a.distance > b.distance) return 1;
            return 0;
        });

        for (int i = 1; i < data.num; i++) {
            edgeQueue.offer(new Edge(0, i, adjacencyMatrix[0][i]));
        }

        int[] visitedCount = new int[data.num];
        visitedCount[0] = numberOfDrones;
        List<List<Integer>> dronePaths = new ArrayList<>();
        for (int i = 0; i < numberOfDrones; i++) {
            dronePaths.add(new ArrayList<>(Collections.singletonList(0)));
        }

        while (!edgeQueue.isEmpty()) {
            Edge edge = edgeQueue.poll();
            if (visitedCount[edge.to] == 0) {
                int droneIndex = findDroneWithShortestPath(adjacencyMatrix, dronePaths, distances, edge.to, limitDistance);
                if (droneIndex != -1) {
                    List<Integer> path = dronePaths.get(droneIndex);
                    int lastNode = path.get(path.size() - 1);
                    double currentDistance = adjacencyMatrix[lastNode][edge.to];
                    double returnDistance = adjacencyMatrix[edge.to][0];

                    if (distances[droneIndex] + currentDistance + returnDistance <= limitDistance) {
                        path.add(edge.to);
                        distances[droneIndex] += currentDistance;
                        visitedCount[edge.to] = 1;

                        for (int i = 1; i < data.num; i++) {
                            if (visitedCount[i] == 0) {
                                edgeQueue.offer(new Edge(edge.to, i, adjacencyMatrix[edge.to][i]));
                            }
                        }
                    }
                }
            }
        }

        // Check if all nodes have been visited
        for (int i = 1; i < data.num; i++) {
            if (visitedCount[i] == 0) {
                return new int[][]{}; // Return empty array if not all nodes can be visited
            }
        }

        for (int i = 0; i < numberOfDrones; i++) {
            List<Integer> pathList = dronePaths.get(i);
            pathList.add(0);
            paths[i] = pathList.stream().mapToInt(Integer::intValue).toArray();
            distances[i] += adjacencyMatrix[pathList.get(pathList.size() - 2)][0]; // Update the distance for the last leg (return to node 0)
            System.out.println("DIS " + i + ": " + distances[i]);
        }

        return paths;
    }

    private static double[][] calculateDistances(Data data) {
        double[][] distances = new double[data.num][data.num];
        for (int i = 0; i < data.num; i++) {
            for (int j = 0; j < data.num; j++) {
                distances[i][j] = Math.sqrt(Math.pow(data.x[i] - data.x[j], 2) + Math.pow(data.y[i] - data.y[j], 2));
            }
        }
        return distances;
    }

    private static int findDroneWithShortestPath(double[][] adjacencyMatrix, List<List<Integer>> dronePaths, double[] droneDistances, int destination, double limitDistance) {
        double minDistance = Double.MAX_VALUE;
        int minIndex = -1;
        for (int i = 0; i < dronePaths.size(); i++) {
            List<Integer> path = dronePaths.get(i);
            int lastNode = path.get(path.size() - 1);
            double currentDistance = adjacencyMatrix[lastNode][destination];
            double totalDistance = droneDistances[i] + currentDistance + adjacencyMatrix[destination][0];

            if (currentDistance < minDistance && totalDistance <= limitDistance) {
                minDistance = currentDistance;
                minIndex = i;
            }
        }
        return minIndex;
    }
}



