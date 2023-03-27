public class Utils {
    // 使用经纬度计算
//    private static double[][] calculateDistances(Data data) {
//        double[][] distances = new double[data.num][data.num];
//        double R = 6371e3; // 地球半径，单位为米
//
//        for (int i = 0; i < data.num; i++) {
//            for (int j = 0; j < data.num; j++) {
//                double lat1Radians = Math.toRadians(data.x[i]);
//                double lat2Radians = Math.toRadians(data.x[j]);
//                double deltaLatRadians = Math.toRadians(data.x[j] - data.x[i]);
//                double deltaLonRadians = Math.toRadians(data.y[j] - data.y[i]);
//
//                double a = Math.sin(deltaLatRadians / 2) * Math.sin(deltaLatRadians / 2) +
//                        Math.cos(lat1Radians) * Math.cos(lat2Radians) *
//                                Math.sin(deltaLonRadians / 2) * Math.sin(deltaLonRadians / 2);
//                double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
//
//                distances[i][j] = R * c;
//                System.out.println(distances[i][j]);
//            }
//        }
//        return distances;
//    }
}
