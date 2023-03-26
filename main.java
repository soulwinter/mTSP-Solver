import java.util.Arrays;


public class main {

    public static void main(String[] args) {
        Data data = new Data();
        data.num = 50;
        data.x = new double[data.num];
        data.y = new double[data.num];
        double tX = 0, tY = 0;
        for (int i = 1; i < data.num; i++) {
            data.x[i] = Math.random() * 100;
            data.y[i] = Math.random() * 100;
            tX += data.x[i];
            tY += data.y[i];
            System.out.println(i + ": " + data.x[i] + " " +  data.y[i]);
        }

        data.x[0] = tX / (data.num-1);
        data.y[0] = tY / (data.num-1);
        System.out.println( "0: " + data.x[0] + " " +  data.y[0]);

        int numberOfDrones = 9;
        double limitDistance = 150.0;
        int[][] paths = MTSPWithLimits.solveMTSP(data, numberOfDrones, limitDistance);

        if (paths.length == 0) {
            System.out.println("Error: Unable to complete the task with the given drones and limit distance.");
        } else {
            for (int i = 0; i < numberOfDrones; i++) {
                System.out.println("无人机 " + i + " 的路径：");
                System.out.println(Arrays.toString(paths[i]));
                double distance = 0;
                for (int j = 1; j < paths[i].length; j++)
                {
                    distance += Math.sqrt(Math.pow(data.x[paths[i][j]]-data.x[paths[i][j-1]],2) + Math.pow(data.y[paths[i][j]]-data.y[paths[i][j-1]],2));
                }

                System.out.println("无人机 " + i + " 的距离：" + distance);
            }
            MTSPVisualizer.visualize(data, paths);
        }


    }

}