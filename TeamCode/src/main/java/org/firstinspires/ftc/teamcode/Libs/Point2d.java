package org.firstinspires.ftc.teamcode.Libs;

import java.util.Locale;

public class Point2d {
    private double x;
    private double y;

    Point2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * returns the distance between this.Point2d and tgtPt
     * @param point2
     * @return distance
     */
    double distance(Point2d point2) {
        double sq_dist = (point2.x - x)*(point2.x -x) + (point2.y - y)*(point2.y - y);
        double distance = Math.sqrt(sq_dist);
        return distance;
    }

    /**
     * returns the distance between two points; neither members of this.
     * @param point1
     * @param point2
     * @return distance
     */
    double distanceBetween(Point2d point1, Point2d point2){
        double sq_dist = (point2.x - point1.x)*(point2.x - point1.x) + (point2.y - point1.y)*(point2.y - point1.y);
        double distance = Math.sqrt(sq_dist);
        return distance;
    }

    /**
     * returns the angle made by the points
     * @param point1 point 1
     * @param point2 point 2
     * @return angle
     */
    double angle(Point2d point1, Point2d point2) {
        double seg1FldHdg = Math.atan2(y - point1.getY(), x - point1.getX());
        double seg2FldHdg = Math.atan2(point2.getY() - y, (point2.getX() - x));
        double angle = Math.toDegrees(seg2FldHdg - seg1FldHdg);
        return angle;
    }

    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }

    public void setX(double x) { this.x = x; }
    public void setY(double y) { this.y = y; }

    public String toString() {
        return String.format(Locale.US, "(%5.2f: %5.2f)", x, y);
    }

}
