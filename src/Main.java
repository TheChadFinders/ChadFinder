public class Main {
    public static void main(String[] args) {
        double x0 = 0.0;
        double y0 = 0.0;
        double x1 = 2.65;
        double y1 = 1.53;
        double theta0 = 0;
        double theta1 = 0;
        double maxv = 2.5;
        double maxa = 3;
        double maxd = 3;
        double initialVel = 0;
        double finalVel = 0;
        double calcdt = 0.0001;
        double dt = 0.02;
        
        Waypoint[] points = new Waypoint[]{
        	new Waypoint(x0, y0, theta0),
        	new Waypoint(x1, y1, theta1),
        	new Waypoint(5, 4, 0)
        };
        
        Waypoint way1 = new Waypoint(x0, y0, theta0);
        Waypoint way2 = new Waypoint(x1, y1, theta1);
        
        //Spline s = new Spline(way1, way2);
        //Spline g = new Spline(x1, y1, theta1, 5, 4, 0);
        //Spline h = new Spline(5, 0, 0, 7, 0, 0);
        
        //Spline[] splines = {s};
        //Spline[] splines1 = {g};
        
        //TrajectoryGeneration t = new TrajectoryGeneration(maxv, maxa, maxd, s.getArcLength(), 0, 2, calcdt, dt, splines);

        //System.out.println(g);

        //System.out.println("Distance " + s.getDistance());

        //System.out.println("ArcLength " + s.getArcLength());
        
        //System.out.println(g.secondDerivative(0) + "asdfadf");
        
        Trajectory t = new Trajectory(points);
        t.configureTrajectory(maxv, maxa, maxd, initialVel, finalVel, calcdt, dt);
        
        t.generate();
        //t.generate();
        //TrajectoryGeneration t2 = new TrajectoryGeneration(maxv, maxa, maxd, g.getArcLength(), 1, 0, calcdt, dt, splines1);
        //t.configureNewTrajectory(maxv, maxa, maxd, g.getArcLength(), 2, 0, calcdt, dt, splines1);
        //System.out.println("switching");
        //System.out.println(t.upvel());
        //t.generate();
       // System.out.println(t);
        
        //System.out.println(t);
    }
}