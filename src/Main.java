public class Main {
    public static void main(String[] args) {
        double x0 = 0.0;
        double y0 = 0.0;
        double x1 = 3;
        double y1 = 3;
        double theta0 = 0;
        double theta1 = 0;
        double maxv = 2;
        double maxa = 6;
        double maxd = 5;
        
        

        Spline s = new Spline(x0, y0, theta0, x1, y1, theta1);
        
        TrajectoryGeneration t = new TrajectoryGeneration(maxv, maxa, maxd, s.getArcLength(), 0, 0, 0.01, s);

        System.out.println(s);

        System.out.println("Distance " + s.getDistance());

        System.out.println("ArcLength " + s.getArcLength());
        
        t.generate();
        
        System.out.println(t);
    }
}