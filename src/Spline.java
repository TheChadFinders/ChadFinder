public class Spline {
    //ax^5 bx^4 cx^3 dx^2 ex

    private double a = -1;
    private double b = 2.7;
    private double c = -0.06;
    private double d = -3;
    private double e = 0;

    private double distance;
    private double xOffset;
    private double yOffset;

    private final double dx = 0.00001;
    private double arcLength;
    private double[] arcLengthIntegral;
    
    private final double width = 0.1;

    public Spline(double x0, double y0, double theta0, double x1, double y1, double theta1){
        System.out.println("Reticulating splines...");
        xOffset = x0;
        yOffset = y0;

        distance = Math.sqrt((x1-x0) * (x1-x0) + (y1-y0) * (y1-y0));
        if (distance==0){
            return;
        }

        double yp0_hat = Math.tan(theta0);
        double yp1_hat = Math.tan(theta1);

        //Here I go straight for the Quintic
        /*a = -(3 * (yp0_hat + yp1_hat)) / (distance * distance * distance * distance);
        b = (8 * yp0_hat + 7 * yp1_hat) / (distance * distance * distance);
        c = -(6 * yp0_hat + 4 * yp1_hat) / (distance * distance);
        d = 0;
        e = yp0_hat;
        */
        arcLength = evaluateArcLength();
        

    }

    @Override
    public String toString() {
        String s = "A -> " + a + "\n" +
                "B -> " + b + "\n" +
                "C -> " + c + "\n" +
                "D -> " + d + "\n" +
                "E -> " + e + "\n";
        return s;
    }

    /**
     *  eval(x): Returns the function evaluated at x
     */
    public double eval(double x){
        return a*Math.pow(x,5) + b*Math.pow(x,4) + c*Math.pow(x,3) + d*Math.pow(x,2) + e*x;
    }


    /**
     *  eval(x,nDerivative): Returns the nDerivative of the function evaluated at x
     */
    public double firstDerivative(double x){
    	return 5*a*Math.pow(x,4) + 4*b*Math.pow(x,3) + 3*c*Math.pow(x,2) + 2*d*x + e;
    }
    
    public double secondDerivative(double x){
    	return 5*4*a*Math.pow(x,3) + 4*3*b*Math.pow(x,2) + 3*2*c*x + 2*d;
    }
    
    public boolean isConcaveUp(double x){
    	return secondDerivative(x) > 0;
    }
    
    public double getXVelocity(double x, boolean isUp) {
    	double firstDeriv = firstDerivative(x);
    	double secDeriv = secondDerivative(x);
    	double sP = width/(2*Math.pow(Math.pow(firstDeriv,2) + 1,1.5));
    	
    	if (isUp)
    		return (1 - secDeriv*sP);
    	else
    		return (1 + secDeriv*sP);
    }
    
    public double getYVelocity(double x, boolean isUp) {
    	double firstDeriv = firstDerivative(x);
    	double secDeriv = secondDerivative(x);
    	double sP = -(width*firstDeriv) / (2*Math.pow(Math.pow(firstDeriv,2) + 1,1.5));
    	
    	if (isUp)
    		return (firstDeriv + secDeriv*sP);
    	else
    		return (firstDeriv - secDeriv*sP);
    }

    public double getVel(double x, boolean isUp) {
    	return Math.sqrt(Math.pow(getXVelocity(x,isUp), 2) + Math.pow(getYVelocity(x, isUp), 2));
    }
    
    public double getWheelVelRatio(double x) {
    	return getVel(x, true) / getVel(x, false);
    }
    
    public double evaluateArcLength(){
        double a = 0;
        double integral = 0;
        
        arcLengthIntegral = new double[(int)(distance/dx)];

        for(int i=0; i<arcLengthIntegral.length; i++) {
        	arcLengthIntegral[i] = integral;
        	integral += Math.sqrt(1+Math.pow(secondDerivative(a), 2)) * dx;
        	a += dx;	
        }
        return integral;
    }
   
    //GETTERS

    public double getA() {
        return a;
    }

    public double getB() {
        return b;
    }

    public double getC() {
        return c;
    }

    public double getD() {
        return d;
    }

    public double getE() {
        return e;
    }

    public double getArcLength() {
		return arcLength;
	}
    
	public double getDistance() {
        return distance;
    }

    public double getxOffset() {
        return xOffset;
    }

    public double getyOffset() {
        return yOffset;
    }

    public double[] getCoefficients(){
        double[] coeff = {a,b,c,d,e};
        return coeff;
    }
}