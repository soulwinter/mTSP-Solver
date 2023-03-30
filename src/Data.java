import java.util.ArrayList;

public class Data {
    public int num;
    public double[] x;
    public double[] y;
    public ForbiddenZone[] forbiddenZones;
    // 判断两点之间的连线是否穿过禁区内部（射线法）
    public boolean isCrossingForbiddenZone(double x1, double y1, double x2, double y2) {
        int numberOfTestPoints = 1000;

        for (int i = 1; i < numberOfTestPoints; i++) {
            double testX = x1 + i * (x2 - x1) / (numberOfTestPoints + 1);
            double testY = y1 + i * (y2 - y1) / (numberOfTestPoints + 1);

            if (isInsideForbiddenZone(testX, testY)) {
                return true;
            }
        }

        return false;
    }
    private boolean isInsideForbiddenZone(double x, double y) {
        if (forbiddenZones != null)
        {
            for (ForbiddenZone zone : forbiddenZones) {
                if (zone.isPointInside(x, y))
                {
                    return true;
                }
            }
        }


        return false;
    }
}

class ForbiddenZone {
    public int nodeNum;
    public double[] x;
    public double[] y;
    private boolean pointOnLineSegment(double x1, double y1, double x2, double y2, double px, double py) {
        double minX = Math.min(x1, x2);
        double maxX = Math.max(x1, x2);
        double minY = Math.min(y1, y2);
        double maxY = Math.max(y1, y2);

        if (px < minX || px > maxX || py < minY || py > maxY) {
            return false;
        }

        double slope = (y2 - y1) / (x2 - x1);
        double intercept = y1 - slope * x1;

        return Math.abs(py - (slope * px + intercept)) < 1e-10;
    }
    public boolean isPointInside(double pointX, double pointY) {
        boolean inside = false;
        int j = nodeNum - 1;
        for (int i = 0; i < nodeNum; j = i++) {
            if (pointOnLineSegment(x[i], y[i], x[j], y[j], pointX, pointY)) {
                return false;
            }
            if (((y[i] > pointY) != (y[j] > pointY)) &&
                    (pointX < (x[j] - x[i]) * (pointY - y[i]) / (y[j] - y[i]) + x[i])) {
                inside = !inside;
            }
        }
        return inside;
    }
}

