public class Spline
{
    //ax^5 bx^4 cx^3 dx^2 ex

    private double a = 0;
    private double b = 0;
    private double c = 0;
    private double d = 0;
    private double e = 0;

    private double distance;
    private double xOffset;
    private double yOffset;

    private final double dx = 0.00001;
    private double arcLength;
    private double[] arcLengthIntegral;

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
        a = -(3 * (yp0_hat + yp1_hat)) / (distance * distance * distance * distance);
        b = (8 * yp0_hat + 7 * yp1_hat) / (distance * distance * distance);
        c = -(6 * yp0_hat + 4 * yp1_hat) / (distance * distance);
        d = 0;
        e = yp0_hat;
        
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
    public double eval(double x, int nDerivative){
        switch (nDerivative){
            case 0: return a*Math.pow(x,5) + b*Math.pow(x,4) + c*Math.pow(x,3) + d*Math.pow(x,2) + e*x;
            case 1: return 5*a*Math.pow(x,4) + 4*b*Math.pow(x,3) + 3*c*Math.pow(x,2) + 2*d*x + e;
            case 2: return 5*4*a*Math.pow(x,3) + 4*3*b*Math.pow(x,2) + 3*2*c*x + 2*d;

            default: return 0;
        }

    }

    public double evaluateArcLength(){
        double a = 0;
        double integral = 0;
        
        arcLengthIntegral = new double[(int)(distance/dx)];

        for(int i=0; i<arcLengthIntegral.length; i++) {
        	arcLengthIntegral[i] = integral;
        	integral += Math.sqrt(1+Math.pow(eval(a,2), 2)) * dx;
        	a += dx;	
        }

        return integral;
    }



    public double[] getXandY(double percentage) {
        double[] result = new double[2];

        percentage = Math.max(Math.min(percentage, 1), 0);
        double x = arcLengthIntegral.length * percentage;
        double y = arcLengthIntegral[(int)(x)];
        x *= dx;
        
        result[0] = x;
        result[1] = y;

        return result;
    }

    public void printCenter(){
        System.out.println("X -> " + getXandY(0.5)[0] + "\n" + "Y -> " + getXandY(0.5)[1]);
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
