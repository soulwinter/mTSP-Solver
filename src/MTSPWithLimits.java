import java.util.*;

/**
 * MTSPWithLimits 类实现了多个无人机巡航的 mTSP 问题，使用贪心算法，目的是总用时最短（即走最长路径的无人机的路径应该尽可能短）
 * 同时增加了每个无人机的最大飞行距离限制，并尽可能最大化利用无人机，以减少无人机的使用数量。
 * 使用方法：
 * int[][] paths = MTSPWithLimits.solveMTSP(data, numberOfDrones, limitDistance, priorAllDrones);
 * eg.  0 号无人机需要经过 0 3 4 2 6 -> path[0] = {0,3,4,2,6}
 */
public class MTSPWithLimits {
    /*
     *    * solveMTSP 方法实现了解决多个无人机巡航的 mTSP 问题
     *    * @param data 数据对象，包括距离矩阵和出发点及禁飞区
     *    * @param numberOfDrones 无人机数量
     *    * @param limitDistance 每个无人机的最大限制飞行距离
     *    * @param priorAllDrones 是否优先启动所有无人机
     *    * @return 返回一个包含各无人机路径的二维数组
     */

    // 包含全部的节点及与禁飞区的交点、禁飞区本身的交点（initForbiddenZonesPoints）
    public static Data dataWithForbiddenZones;
    public static double[][] adjacencyMatrix;
    public static int[][] solveMTSP(Data data, int numberOfDrones, double limitDistance, boolean prioritizeAllDrones) {
        int[][] paths = new int[numberOfDrones][];
        double[] pathDistances = new double[numberOfDrones];

        // 最大尝试次数
        int maxAttempts = 10000;
        int attempts = 0;

        // 更新 dataWithForbiddenZones
        initForbiddenZonesPoints(data);

        adjacencyMatrix = calculateDistances(dataWithForbiddenZones);


        List<Integer>[][] forbiddenRounds = updateDistancesWithDijkstra(adjacencyMatrix, dataWithForbiddenZones);

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

        while (!edgeQueue.isEmpty() && attempts < maxAttempts) {

            Edge edge = edgeQueue.poll();
            if (visitedCount[edge.to] == 0) {
                int droneIndex = findDroneWithShortestPath(adjacencyMatrix, dronePaths, pathDistances, edge.to, limitDistance, prioritizeAllDrones);
                if (droneIndex != -1) {

                    List<Integer> path = dronePaths.get(droneIndex);
                    int lastNode = path.get(path.size() - 1);
                    double currentDistance = adjacencyMatrix[lastNode][edge.to];
                    double returnDistance = adjacencyMatrix[edge.to][0];

                    if (pathDistances[droneIndex] + currentDistance + returnDistance <= limitDistance) {

                        // 如果存在中间节点，就加上
                        if (forbiddenRounds[lastNode][edge.to] != null)
                        {
                            path.addAll(forbiddenRounds[lastNode][edge.to]);

                        }

                        path.add(edge.to);
                        System.out.println("ADD2: " + edge.to);
                        pathDistances[droneIndex] += currentDistance;

                        visitedCount[edge.to] = 1;

                        for (int i = 1; i < data.num; i++) {
                            if (visitedCount[i] == 0) {
                                edgeQueue.offer(new Edge(edge.to, i, adjacencyMatrix[edge.to][i]));
                            }
                        }
                    }
                } else {
                    // 如果找不到适合的无人机，将边重新放回队列，以便在后续尝试中重新评估
                    edgeQueue.offer(edge);
                }
            }
            attempts++;
        }


        if (attempts >= maxAttempts) {
            return new int[][]{}; // 如果达到最大尝试次数，返回空数组
        }

        // Check if all nodes have been visited
        for (int i = 1; i < data.num; i++) {
            if (visitedCount[i] == 0) {
                return new int[][]{}; // Return empty array if not all nodes can be visited
            }
        }

        for (int i = 0; i < numberOfDrones; i++) {

            List<Integer> pathList = dronePaths.get(i);
            // 如果存在中间节点，就加上
            double currentDistance = adjacencyMatrix[pathList.get(pathList.size()-1)][0];
            if (forbiddenRounds[pathList.get(pathList.size()-1)][0] != null)
            {
                pathList.addAll(forbiddenRounds[pathList.get(pathList.size()-1)][0]);

            }
            pathList.add(0);
            paths[i] = pathList.stream().mapToInt(Integer::intValue).toArray();
            pathDistances[i] += currentDistance;

            System.out.println("DIS " + i + ": " + pathDistances[i]);
            System.out.println(Arrays.toString(paths[i]));
        }

        return paths;
    }


    public static void initForbiddenZonesPoints(Data data)
    {

        //FIXME
//        dataWithForbiddenZones = data;

         dataWithForbiddenZones = new Data();

        ArrayList<Double> xList = new ArrayList<>();
        ArrayList<Double> yList = new ArrayList<>();
        // 加入全部节点
        for (double value : data.x) {
            xList.add(value);
        }
        for (double value : data.y) {
            yList.add(value);
        }
        if (data.forbiddenZones != null)
        {
            // 将禁飞区节点加入
            for (ForbiddenZone zone : data.forbiddenZones) {
                for (double value : zone.x) {
                    xList.add(value);
                    System.out.println(xList.size()-1);
                }
                for (double value : zone.y) {
                    yList.add(value);
                }
            }
        }



        dataWithForbiddenZones.x = xList.stream().mapToDouble(Double::doubleValue).toArray();
        dataWithForbiddenZones.y = yList.stream().mapToDouble(Double::doubleValue).toArray();
        dataWithForbiddenZones.num = dataWithForbiddenZones.x.length;

        dataWithForbiddenZones.forbiddenZones = data.forbiddenZones;

        // 此时，包含全部的节点及与禁飞区的交点、禁飞区本身的交点

    }

    // 计算距离，如果有交点，就把它们的距离设为最大值
    private static double[][] calculateDistances(Data data) {
        double[][] distances = new double[data.num][data.num];
        for (int i = 0; i < data.num - 1; i++) {
            for (int j = i + 1; j < data.num; j++) {
                // TODO: 判断两个点是否为禁区节点且连线穿过禁区，如果是则距离无限大
                if (data.isCrossingForbiddenZone(data.x[i], data.y[i], data.x[j], data.y[j]))
                {
                    distances[i][j] = Double.MAX_VALUE;
                    distances[j][i] = Double.MAX_VALUE;

                } else {
                    double distance = Math.sqrt(Math.pow(data.x[i] - data.x[j], 2) + Math.pow(data.y[i] - data.y[j], 2));
                    distances[i][j] = distance;
                    distances[j][i] = distance;
                }

            }
        }
        return distances;
    }

    // 更新路径距离，并给出经过点
    // NOTE: 这里的 distances 应该是对应 dataWithForbiddenZones 的
    public static List<Integer>[][] updateDistancesWithDijkstra(double[][] distances, Data dataWithForbiddenZones) {
        int size = distances.length;
        List<Integer>[][] paths = new ArrayList[size][size];

        for (int i = 0; i < size; i++) {
            for (int j = i + 1; j < size; j++) {
                if (distances[i][j] == Double.MAX_VALUE) {
                    // Calculate shortest path between nodes i and j using Dijkstra's algorithm
                    DijkstraResult result = dijkstra(dataWithForbiddenZones, distances, i, j);

                    // Update distances matrix
                    distances[i][j] = result.distance;
                    distances[j][i] = result.distance;

                    // Update paths matrix
                    paths[i][j] = result.path;
                    paths[j][i] = new ArrayList<>(result.path);
                    Collections.reverse(paths[j][i]);
                }
                if (distances[i][j] != Double.MAX_VALUE)
                {
                    System.out.println("Distance " + i + "->" + j + ": " + distances[i][j]);
                    // System.out.println("Distance " + j + "->" + i + ": " + distances[j][i]);
                }

            }
        }
        adjacencyMatrix = distances;
        return paths;
    }
    // 计算最短路径
    private static DijkstraResult dijkstra(Data data, double[][] distances, int source, int target) {
        int size = data.num;
        double[] minDistances = new double[size];
        Arrays.fill(minDistances, Double.MAX_VALUE);
        minDistances[source] = 0;

        int[] previous = new int[size];
        Arrays.fill(previous, -1);

        PriorityQueue<Integer> queue = new PriorityQueue<>(Comparator.comparingDouble(i -> minDistances[i]));
        queue.add(source);

        while (!queue.isEmpty()) {
            int current = queue.poll();

            if (current == target) {
                break;
            }

            for (int neighbor = 0; neighbor < size; neighbor++) {
                if (neighbor == current || distances[current][neighbor] == Double.MAX_VALUE) {
                    continue;
                }

                double newDistance = minDistances[current] + distances[current][neighbor];
                if (newDistance < minDistances[neighbor]) {
                    minDistances[neighbor] = newDistance;
                    previous[neighbor] = current;
                    queue.remove(neighbor);
                    queue.add(neighbor);
                }
            }
        }

        List<Integer> path = new ArrayList<>();
        for (int node = target; node != -1; node = previous[node]) {
            path.add(node);
        }
        Collections.reverse(path);
        // Remove source node and target node if the path is not empty
        if (!path.isEmpty()) {
            path.remove(0); // Remove source node
            if (path.size() > 0) {
                path.remove(path.size() - 1); // Remove target node
            }
        }

        return new DijkstraResult(minDistances[target], path);
    }



    private static int findDroneWithShortestPath(double[][] adjacencyMatrix, List<List<Integer>> dronePaths, double[] droneDistances, int destination, double limitDistance, boolean prioritizeAllDrones) {
        double minDistance = Double.MAX_VALUE;
        int minIndex = -1;
        int unassignedDroneIndex = -1;
        for (int i = 0; i < dronePaths.size(); i++) {
            List<Integer> path = dronePaths.get(i);
            if (prioritizeAllDrones && path.size() == 1) { // 如果有未分配任务的无人机且优先启动所有无人机
                unassignedDroneIndex = i;
                break;
            }
            int lastNode = path.get(path.size() - 1);
            double currentDistance = adjacencyMatrix[lastNode][destination];
            double totalDistance = droneDistances[i] + currentDistance + adjacencyMatrix[destination][0];

            if (currentDistance < minDistance && totalDistance <= limitDistance) {
                minDistance = currentDistance;
                minIndex = i;
            }
        }
        if (unassignedDroneIndex != -1) { // 当找到未分配任务的无人机时，优先使用未分配任务的无人机
            return unassignedDroneIndex;
        }
        return minIndex;
    }
}

class Edge {
    int from;
    int to;
    double distance;

    Edge(int from, int to, double distance) {
        this.from = from;
        this.to = to;
        this.distance = distance;
    }
}


class DijkstraResult {
    double distance;
    List<Integer> path;

    DijkstraResult(double distance, List<Integer> path) {
        this.distance = distance;
        this.path = path;
    }
}
