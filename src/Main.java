public class Main {
    public static void main(String[] args) {
        double x0 = 0.0;
        double y0 = 0.0;
        double x1 = 5;
        double y1 = 5;
        double theta0 = 0;
        double theta1 = Math.toRadians(60);
        double maxv = 1.75;
        double maxa = 3;
        double maxd = 3;
        double dt = .02;
        
        

        Spline s = new Spline(x0, y0, theta0, x1, y1, theta1);
        //Spline g = new Spline(x1, y1, theta1, 5, 0, 0);
        //Spline h = new Spline(5, 0, 0, 7, 0, 0);
        
        Spline[] splines = {s};
        
        TrajectoryGeneration t = new TrajectoryGeneration(maxv, maxa, maxd, s.getArcLength(), 0, 0, dt, splines);

        System.out.println(s);

        System.out.println("Distance " + s.getDistance());

        System.out.println("ArcLength " + s.getArcLength());
        
        t.generate();
        
        //System.out.println(t);
    }
}