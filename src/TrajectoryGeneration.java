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
	private int index = 0;
	private double currentUpperPos = 0, currentLowerPos = 0, currentUpperVel, currentLowerVel;
	private double currentInnerArcLength = 0;
	private double cruiseVel;
	private double decelerateDistance;
	private boolean finished = false;
	private ArrayList<TrajectoryPoint> upperWheel = new ArrayList<TrajectoryPoint>();
	private ArrayList<TrajectoryPoint> lowerWheel = new ArrayList<TrajectoryPoint>();
	
	public TrajectoryGeneration(double maxVelocity, double maxAcceleration, double maxDeceleration,
			double arcLength, double initialVel, double finalVel, double dt, Spline spline) {
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
		this.cruiseVel = Math.min(getCruiseVel(), maxVelocity);
		this.decelerateDistance = getDecelerateDistance();
		this.dt = dt;
		setState(MotionState.ACCELERATING);
		finished = false;
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
	
	public double getDecelerateDistance(){
		return (Math.pow(cruiseVel, 2) - Math.pow(initialVelocity, 2)) / (2 * maxDeceleration);
	}

	public void generate(){
		System.out.println(s.getArcLengths().length);
		while(x < 5 && index < 422290){
			System.out.println(currentLowerVel + " " + currentUpperVel);
			TrajectoryPoint upper = new TrajectoryPoint();
			TrajectoryPoint lower = new TrajectoryPoint();
			if(getState() == MotionState.ACCELERATING){
				if(s.isConcaveUp(x)){
					currentLowerPos += currentLowerVel * dt + maxAcceleration * dt * dt * 0.5;
					currentLowerVel = currentLowerVel + maxAcceleration * dt;
					updateMotionState();
					
					if(getState() == MotionState.ACCELERATING){
						lower.pos = currentLowerPos;
						lower.vel = currentLowerVel;

						double velRatio = s.getWheelVelRatio(x);
						currentUpperVel = currentLowerVel * velRatio;
						currentUpperPos += currentUpperVel * dt;
						
						upper.pos = currentUpperPos;
						upper.vel = currentUpperVel;
						
						lowerWheel.add(lower);
						upperWheel.add(upper);

						double inVelRatio = s.getInnVelRatio(x, false);
						double innerVel = currentLowerVel / inVelRatio;
						currentInnerArcLength += innerVel * dt;

						x = inverseArcLength(currentInnerArcLength);
					}
				}
				else{
					currentUpperPos += currentUpperVel * dt + maxAcceleration * dt * dt * 0.5;
					currentUpperVel = currentUpperVel + maxAcceleration * dt;
					updateMotionState();
					
					if(getState() == MotionState.ACCELERATING){
						upper.pos = currentUpperPos;
						upper.vel = currentUpperVel;

						double velRatio = s.getWheelVelRatio(x);
						currentLowerVel = currentUpperVel / velRatio;
						currentLowerPos += currentLowerVel * dt;
						
						lower.pos = currentLowerPos;
						lower.vel = currentLowerVel;
						
						lowerWheel.add(lower);
						upperWheel.add(upper);

						double inVelRatio = s.getInnVelRatio(x, true);
						double innerVel = currentUpperVel / inVelRatio;
						currentInnerArcLength += innerVel * dt;

						x = inverseArcLength(currentInnerArcLength);
					}
				}
			}
			else if(getState() == MotionState.CRUISING){
				if(s.isConcaveUp(x)){
					currentLowerPos += cruiseVel * dt;
					currentLowerVel = cruiseVel;
					updateMotionState();
					
					if(getState() == MotionState.CRUISING){
						lower.pos = currentLowerPos;
						lower.vel = currentLowerVel;
						
						double velRatio = s.getWheelVelRatio(x);
						currentUpperVel = currentLowerVel * velRatio;
						currentUpperPos += currentUpperVel * dt;
						
						upper.pos = currentUpperPos;
						upper.vel = currentUpperVel;
						
						lowerWheel.add(lower);
						upperWheel.add(upper);

						double inVelRatio = s.getInnVelRatio(x, false);
						double innerVel = currentLowerVel / inVelRatio;
						currentInnerArcLength += innerVel * dt;

						x = inverseArcLength(currentInnerArcLength);
					}
				}
				else{
					currentUpperPos += cruiseVel * dt;
					currentUpperVel = cruiseVel;
					updateMotionState();
					
					if(getState() == MotionState.CRUISING){
						upper.pos = currentUpperPos;
						upper.vel = currentUpperVel;

						double velRatio = s.getWheelVelRatio(x);
						currentLowerVel = currentUpperVel / velRatio;
						currentLowerPos += currentLowerVel * dt;
						
						lower.pos = currentLowerPos;
						lower.vel = currentLowerVel;
						
						lowerWheel.add(lower);
						upperWheel.add(upper);

						double inVelRatio = s.getInnVelRatio(x, true);
						double innerVel = currentUpperVel / inVelRatio;
						currentInnerArcLength += innerVel * dt;

						x = inverseArcLength(currentInnerArcLength);
					}
				}
			}
			else{
				if(s.isConcaveUp(x)){
					currentLowerPos += currentLowerVel * dt - maxDeceleration * dt * dt * 0.5;
					currentLowerVel = currentLowerVel - maxDeceleration * dt;

					lower.pos = currentLowerPos;
					lower.vel = currentLowerVel;

					double velRatio = s.getWheelVelRatio(x);
					currentUpperVel = currentLowerVel * velRatio;
					currentUpperPos += currentUpperVel * dt;

					upper.pos = currentUpperPos;
					upper.vel = currentUpperVel;

					lowerWheel.add(lower);
					upperWheel.add(upper);

					double inVelRatio = s.getInnVelRatio(x, false);
					double innerVel = currentLowerVel / inVelRatio;
					currentInnerArcLength += innerVel * dt;

					x = inverseArcLength(currentInnerArcLength);
				}
				else{
					currentUpperPos += currentUpperVel * dt - maxDeceleration * dt * dt * 0.5;
					currentUpperVel = currentUpperVel - maxDeceleration * dt;
					
					upper.pos = currentUpperPos;
					upper.vel = currentUpperVel;

					double velRatio = s.getWheelVelRatio(x);
					currentLowerVel = currentUpperVel / velRatio;
					currentLowerPos += currentLowerVel * dt;

					lower.pos = currentLowerPos;
					lower.vel = currentLowerVel;

					lowerWheel.add(lower);
					upperWheel.add(upper);

					double inVelRatio = s.getInnVelRatio(x, true);
					double innerVel = currentUpperVel / inVelRatio;
					currentInnerArcLength += innerVel * dt;

					x = inverseArcLength(currentInnerArcLength);
				}
			}
		}
	}
	
	public double inverseArcLength(double arcLength){
		while(index < s.getArcLengths().length && arcLength > s.getArcLengths()[index]){
			index++;
		}
		if(arcLength > s.getArcLengths()[index]) {
			finished = true;
		}
		return index * s.getDX();
	}
	
	private void updateMotionState(){
		if(s.getUpperArcLength() - currentUpperPos <= decelerateDistance ||
				s.getLowerArcLength() - currentLowerPos <= decelerateDistance){
			setState(MotionState.DECELERATING);
		}
		if(getState() == MotionState.ACCELERATING){
			if(currentUpperVel >= cruiseVel || currentLowerVel >= cruiseVel){
				setState(MotionState.CRUISING);
			}
		}
	}
	
	public String toString(){
		return "" + currentUpperPos;
	}
	
}
