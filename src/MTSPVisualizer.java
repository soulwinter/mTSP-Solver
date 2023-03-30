import javax.swing.*;
import java.awt.*;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;

public class MTSPVisualizer extends JFrame {
    private static final int WIDTH = 800;
    private static final int HEIGHT = 800;
    private static final int POINT_SIZE = 10;

    private static final int OFFSET = 50;
    private Data data;
    private int[][] path;

    public MTSPVisualizer(Data data, int[][] path) {
        this.data = data;
        this.path = path;

        setTitle("MTSP Solution Visualizer");
        setSize(WIDTH, HEIGHT);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setLocationRelativeTo(null);
        setVisible(true);
    }

    @Override
    public void paint(Graphics g) {
        super.paint(g);
        Graphics2D g2d = (Graphics2D) g;


        // 绘制禁飞区
        if (data.forbiddenZones != null)
        {
            Color forbiddenZoneColor = new Color(255, 102, 102, 128); // 半透明淡红色
            g2d.setColor(forbiddenZoneColor);
            for (ForbiddenZone zone : data.forbiddenZones) {
                Path2D polygon = new Path2D.Double();
                polygon.moveTo(zone.x[0] * (WIDTH - POINT_SIZE) / 100 + OFFSET, zone.y[0] * (HEIGHT - POINT_SIZE) / 100 + OFFSET);
                for (int i = 1; i < zone.nodeNum; i++) {
                    polygon.lineTo(zone.x[i] * (WIDTH - POINT_SIZE) / 100 + OFFSET, zone.y[i] * (HEIGHT - POINT_SIZE) / 100 + OFFSET);
                }
                polygon.closePath();
                g2d.fill(polygon);
            }
        }


        for (int i = 0; i < data.num; i++) {
            Ellipse2D point = new Ellipse2D.Double(data.x[i] * (WIDTH - POINT_SIZE) / 100 + OFFSET,
                    data.y[i] * (HEIGHT - POINT_SIZE) / 100 + OFFSET,
                    POINT_SIZE,
                    POINT_SIZE);
            g2d.fill(point);
            g2d.setColor(Color.WHITE);
            g2d.drawString(Integer.toString(i), (int) (data.x[i] * (WIDTH - POINT_SIZE) / 100) + OFFSET, (int) (data.y[i] * (HEIGHT - POINT_SIZE) / 100) + OFFSET);
        }

        for (int i = 0; i < path.length; i++) {
            g2d.setColor(new Color((int) (Math.random() * 0x1000000)));
            for (int j = 0; j < path[i].length - 1; j++) {
                g2d.draw(new Line2D.Double(data.x[path[i][j]] * (WIDTH - POINT_SIZE) / 100 + POINT_SIZE / 2 + OFFSET,
                        data.y[path[i][j]] * (HEIGHT - POINT_SIZE) / 100 + POINT_SIZE / 2 + OFFSET,
                        data.x[path[i][j + 1]] * (WIDTH - POINT_SIZE) / 100 + POINT_SIZE / 2 + OFFSET,
                        data.y[path[i][j + 1]] * (HEIGHT - POINT_SIZE) / 100 + POINT_SIZE / 2 + OFFSET));
            }
        }
    }

    public static void visualize(Data data, int[][] path) {
        new MTSPVisualizer(data, path);
    }
}
