import java.util.ArrayList;
public class TrajectoryGeneration {
  
	private double maxVelocity;
	private double maxAcceleration;
	private double maxDeceleration;
	private double initialVelocity;
	private double finalVelocity;
	private double arcLength;
	private double dt;
	private double x = 0;
	private Spline s;
	private int index;
	private double currentUpperPos = 0, currentLowerPos = 0, currentUpperVel, currentLowerVel;
	private ArrayList<TrajectoryPoint> upperWheel = new ArrayList<TrajectoryPoint>();
	private ArrayList<TrajectoryPoint> lowerWheel = new ArrayList<TrajectoryPoint>();
	
	public TrajectoryGeneration(double maxVelocity, double maxAcceleration, double maxDeceleration,
			double arcLength, double initialVel, double finalVel, Spline spline) {
		// TODO Auto-generated constructor stub
		this.maxVelocity = maxVelocity;
		this.maxAcceleration = maxAcceleration;
		this.maxDeceleration = maxDeceleration;
		this.arcLength = arcLength;
		this.s = spline;
		this.initialVelocity = initialVel;
		this.finalVelocity = finalVel;
		this.currentLowerVel = initialVel;
		this.currentUpperVel = initialVel;
	}
	
	private enum MotionState{
		ACCELERATING, CRUISING, DECELERATING, END
	}
	
	private MotionState state = MotionState.END;
	
	private void setState(MotionState newState){
		state = newState;
	}
	
	private MotionState getState(){
		return state;
	}
	
	private class TrajectoryPoint{
		public double pos = 0, vel = 0, time;
		
		public TrajectoryPoint(){
		}
		
		public TrajectoryPoint(double pos, double vel, double time){
			this.pos = pos;
			this.vel = vel;
		}
	}
	
	public double getCruiseVel(){
		double first = Math.pow(finalVelocity, 2) - Math.pow(initialVelocity, 2) + 2*maxDeceleration*arcLength;
		double denom = maxAcceleration / (maxAcceleration + maxDeceleration);
		return Math.sqrt(first * denom + Math.pow(initialVelocity, 2));
	}

	public void generate(){
		while(x < s.getDistance()){
			TrajectoryPoint upper = new TrajectoryPoint();
			TrajectoryPoint lower = new TrajectoryPoint();
			if(getState() == MotionState.ACCELERATING){
				if(s.isConcaveUp(x)){
					currentLowerPos += currentLowerVel * dt + maxAcceleration * dt * dt * 0.5;
					currentLowerVel = currentLowerVel + maxAcceleration * dt;
					lower.pos = currentLowerPos;
					lower.vel = currentLowerVel;
					
				
				}
			}
		}
	}
	
}
