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
	private Spline[] splines;
	private int index = 0;
	private int splineIndex = 0;
	private double currentUpperPos = 0, currentLowerPos = 0, currentUpperVel, currentLowerVel;
	private double currentInnerArcLength = 0;
	private double cruiseVel;
	private double decelerateDistance;
	private double currentTime;
	private double totalUpperArcLength;
	private double totalLowerArcLength;
	private double previousUpperArcLengths;
	private double previousLowerArcLengths;
	private double totalXDistance;
	private boolean finished = false;
	private ArrayList<TrajectoryPoint> upperWheel = new ArrayList<TrajectoryPoint>();
	private ArrayList<TrajectoryPoint> lowerWheel = new ArrayList<TrajectoryPoint>();
	
	public TrajectoryGeneration(double maxVelocity, double maxAcceleration, double maxDeceleration,
			double arcLength, double initialVel, double finalVel, double dt, Spline[] splines) {
		// TODO Auto-generated constructor stub
		this.maxVelocity = maxVelocity;
		this.maxAcceleration = maxAcceleration;
		this.maxDeceleration = maxDeceleration;
		this.arcLength = arcLength;
		this.initialVelocity = initialVel;
		this.finalVelocity = finalVel;
		this.currentLowerVel = initialVel;
		this.currentUpperVel = initialVel;
		this.cruiseVel = Math.min(getCruiseVel(), maxVelocity);
		this.decelerateDistance = getDecelerateDistance();
		this.dt = dt;
		this.currentTime = 0;
		this.previousUpperArcLengths = 0;
		this.previousLowerArcLengths = 0;
		setState(MotionState.ACCELERATING);
		finished = false;
		for(Spline s : splines){
			this.totalUpperArcLength += s.getUpperArcLength();
			this.totalLowerArcLength += s.getLowerArcLength();
			this.totalXDistance += s.getDistance();
		}
		this.splines = splines;
		this.splineIndex = 0;
		this.s = splines[0];
		System.out.println(decelerateDistance + " decelerate distance");
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
		//System.out.println(s.getArcLengths().length);
		while(x < this.totalXDistance && getState() != MotionState.END){
			System.out.println(currentLowerVel + " " + currentLowerPos + " " + currentUpperVel + " " + currentUpperPos + " " + index);
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
						lower.time = currentTime;

						double velRatio = s.getWheelVelRatio(x);
						currentUpperVel = currentLowerVel * velRatio;
						currentUpperPos += currentUpperVel * dt;
						
						upper.pos = currentUpperPos;
						upper.vel = currentUpperVel;
						upper.time = currentTime;

						lowerWheel.add(lower);
						upperWheel.add(upper);

						double inVelRatio = s.getInnVelRatio(x, false);
						double innerVel = currentLowerVel / inVelRatio;
						currentInnerArcLength += innerVel * dt + maxAcceleration / inVelRatio * dt * dt * 0.5;

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
						upper.time = currentTime;

						double velRatio = s.getWheelVelRatio(x);
						currentLowerVel = currentUpperVel / velRatio;
						currentLowerPos += currentLowerVel * dt;
						
						lower.pos = currentLowerPos;
						lower.vel = currentLowerVel;
						lower.time = currentTime;

						lowerWheel.add(lower);
						upperWheel.add(upper);

						double inVelRatio = s.getInnVelRatio(x, true);
						double innerVel = currentUpperVel / inVelRatio;
						currentInnerArcLength += innerVel * dt + maxAcceleration / inVelRatio * dt * dt * 0.5;;

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
						lower.time = currentTime;

						double velRatio = s.getWheelVelRatio(x);
						currentUpperVel = currentLowerVel * velRatio;
						currentUpperPos += currentUpperVel * dt;
						
						upper.pos = currentUpperPos;
						upper.vel = currentUpperVel;
						upper.time = currentTime;
						
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
						upper.time = currentTime;

						double velRatio = s.getWheelVelRatio(x);
						currentLowerVel = currentUpperVel / velRatio;
						currentLowerPos += currentLowerVel * dt;
						
						lower.pos = currentLowerPos;
						lower.vel = currentLowerVel;
						lower.time = currentTime;

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
				updateMotionState();
				if(s.isConcaveUp(x)){
					currentLowerPos += currentLowerVel * dt - maxDeceleration * dt * dt * 0.5;
					currentLowerVel = currentLowerVel - maxDeceleration * dt;

					lower.pos = currentLowerPos;
					lower.vel = currentLowerVel;
					lower.time = currentTime;

					double velRatio = s.getWheelVelRatio(x);
					currentUpperVel = currentLowerVel * velRatio;
					currentUpperPos += currentUpperVel * dt;

					upper.pos = currentUpperPos;
					upper.vel = currentUpperVel;
					upper.time = currentTime;

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
					upper.time = currentTime;

					double velRatio = s.getWheelVelRatio(x);
					currentLowerVel = currentUpperVel / velRatio;
					currentLowerPos += currentLowerVel * dt;

					lower.pos = currentLowerPos;
					lower.vel = currentLowerVel;
					lower.time = currentTime;

					lowerWheel.add(lower);
					upperWheel.add(upper);

					double inVelRatio = s.getInnVelRatio(x, true);
					double innerVel = currentUpperVel / inVelRatio;
					currentInnerArcLength += innerVel * dt;

					x = inverseArcLength(currentInnerArcLength);
				}
			}
			currentTime += dt;
		}
	}
	
	public double inverseArcLength(double arcLength){
		while(index < s.getArcLengths().length && arcLength > s.getArcLengths()[index]){
			index++;
		}
		/*
		if(arcLength < s.getArcLengths()[index]) {
			finished = true;
			System.out.println("" + index * s.getDX());
		}*/
		if(getState() == MotionState.END){
			index = s.getArcLengths().length;
		}
		return index * s.getDX();
	}
	
	private void updateMotionState(){
		//System.out.println(currentUpperPos - previousUpperArcLengths);
		if(currentUpperPos - previousUpperArcLengths >= s.getUpperArcLength() || 
				currentLowerPos - previousLowerArcLengths >= s.getLowerArcLength()){
			if(splineIndex < splines.length - 1){
				splineIndex++;
				index = 0;
				currentInnerArcLength = 0;
				previousUpperArcLengths = currentUpperPos;
				previousLowerArcLengths = currentUpperPos;
				this.s = splines[splineIndex];
				System.out.println(s.getArcLength());
			}
		}
		if((this.totalUpperArcLength - currentUpperPos <= decelerateDistance) && (currentUpperVel >= currentLowerVel) ||
				(this.totalLowerArcLength - currentLowerPos <= decelerateDistance) && (currentUpperVel <= currentLowerVel)){
			setState(MotionState.DECELERATING);
			System.out.println("DECELERATING");
		}
		if(getState() == MotionState.ACCELERATING){
			if(currentUpperVel >= cruiseVel || currentLowerVel >= cruiseVel){
				setState(MotionState.CRUISING);
				System.out.println("CRUISING");
			}
		}
		if(currentUpperVel < finalVelocity || currentLowerVel < finalVelocity) {
			setState(MotionState.END);
			System.out.println("END");
			System.out.println(currentUpperPos);
			System.out.println(currentLowerPos);
		}
	}
	
	public String toString(){
		return "" + currentUpperPos;
	}
	
}
