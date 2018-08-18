import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;

public class Main {
	
	static PrintWriter log;
	static File oldFile, newFile;
	
	static String name = "yuh";
	
    public static void main(String[] args) {
        double x0 = 0.0;
        double y0 = 0.0;
        double x1 = 1.53;
        double y1 = 2.65;
        double theta0 = 0;
        double theta1 = Math.PI/3;
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
        	//new Waypoint(5, 4, Math.PI/2)
        };
        
        //Waypoint way1 = new Waypoint(x0, y0, theta0);
        //Waypoint way2 = new Waypoint(x1, y1, theta1);
        
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
        ArrayList<TrajectoryGeneration.TrajectoryPoint> leftWheel = t.getLeftWheelTrajectory();
        ArrayList<TrajectoryGeneration.TrajectoryPoint> rightWheel = t.getRightWheelTrajectory();

        if (log == null) {
			try {
				oldFile = new File("Paths/" + name + ".txt");
				if(oldFile.exists()) {
					oldFile.delete();
				}
				newFile = new File("Paths/" + name + ".txt");
				newFile.createNewFile();
				log = new PrintWriter(newFile);
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		
		for (int i = 0; i < leftWheel.size(); i++) {
			
			log.printf("%f, %f, %f, %f, %f\n",
					leftWheel.get(i).vel, leftWheel.get(i).pos, rightWheel.get(i).vel, rightWheel.get(i).pos, dt);
		}
		
		if(log != null) {
    		log.close();
    		log = null;
    	}
    }
}