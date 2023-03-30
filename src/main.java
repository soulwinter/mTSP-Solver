import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class main {
    public static void main(String[] args) {
        Data data = new Data();


// Generate a random number of forbidden zones between 1 and 5
//        int numForbiddenZones = (int) (Math.random() * 5) + 1;
//        List<ForbiddenZone> forbiddenZones = new ArrayList<>();
//
//        for (int i = 0; i < numForbiddenZones; i++) {
//            ForbiddenZone zone = new ForbiddenZone();
//
//            // Generate a random number of nodes between 3 and 8
//            zone.nodeNum = (int) (Math.random() * 6) + 3;
//            zone.x = new double[zone.nodeNum];
//            zone.y = new double[zone.nodeNum];
//
//            // Generate random nodes
//            double centerX = Math.random() * 100;
//            double centerY = Math.random() * 100;
//            double radius = Math.random() * 50;
//            double angleStep = 2 * Math.PI / zone.nodeNum;
//
//            for (int j = 0; j < zone.nodeNum; j++) {
//                zone.x[j] = centerX + radius * Math.cos(angleStep * j);
//                zone.y[j] = centerY + radius * Math.sin(angleStep * j);
//            }
//
//            forbiddenZones.add(zone);
//        }
//
//        data.forbiddenZones = forbiddenZones.toArray(new ForbiddenZone[forbiddenZones.size()]);


        data.num = 20;
        data.x = new double[data.num];
        data.y = new double[data.num];
        int times = 0;
        while (times < data.num) {
            boolean pointInForbiddenZone = false;

            double x = Math.random() * 100;
            double y = Math.random() * 100;

//            for (ForbiddenZone zone : data.forbiddenZones) {
//                if (zone.isPointInside(x, y)) {
//                    pointInForbiddenZone = true;
//                    break;
//                }
//            }

            if (!pointInForbiddenZone) {
                data.x[times] = x;
                data.y[times] = y;
                times++;
            }
        }


        System.out.println("0: " + data.x[0] + " " + data.y[0]);

        int numberOfDrones = 5;

        double limitDistance = 500.0;
        int[][] paths = MTSPWithLimits.solveMTSP(data, numberOfDrones, limitDistance, true);

        if (paths.length == 0) {
            System.out.println("Error: Unable to complete the task with the given drones and limit distance.");
        } else {
            for (int i = 0; i < numberOfDrones; i++) {
                System.out.println("无人机 " + i + " 的路径：");
                System.out.println(Arrays.toString(paths[i]));
                double distance = 0;
                for (int j = 1; j < paths[i].length; j++) {
                    distance += Math.sqrt(Math.pow(MTSPWithLimits.dataWithForbiddenZones.x[paths[i][j]] - MTSPWithLimits.dataWithForbiddenZones.x[paths[i][j - 1]], 2) +
                            Math.pow(MTSPWithLimits.dataWithForbiddenZones.y[paths[i][j]] - MTSPWithLimits.dataWithForbiddenZones.y[paths[i][j - 1]], 2));
                }

                System.out.println("无人机 " + i + " 的距离：" + distance);
            }
            MTSPVisualizer.visualize(MTSPWithLimits.dataWithForbiddenZones, paths);
        }


    }
}
