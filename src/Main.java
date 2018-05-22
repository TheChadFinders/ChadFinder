public class Main {
    public static void main(String[] args) {
        double x0 = 0.0;
        double y0 = 0.0;
        double x1 = 3;
        double y1 = 3;
        double theta0 = Math.PI / 3;
        double theta1 = -Math.PI / 8;

        Spline s = new Spline(x0, y0, theta0, x1, y1, theta1);

        System.out.println(s);

        System.out.println("Distance " + s.getDistance());

        s.printCenter();


        System.out.println("ArcLength " + s.getArcLength());
    }
}