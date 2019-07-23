package org.firstinspires.ftc.teamcode.Libs;

import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.internal.opengl.models.Geometry;
import org.firstinspires.ftc.teamcode.Libs.Point2d;

import java.util.Locale;


/**
 * Created by Zachary Collins on 7/23/19.
 * Instagram: @hilariously_random
 *
 * A Vector object for java with vector addition
 */

public class Vector2d{
    private Point2d point;
    private Point2d point2;
    private double angle;
    private double magnitude;


    public Vector2d(Point2d point, double magnitude, double angle){
        this.point =  point;
        this.magnitude = magnitude;
        this.angle = angle;
        this.point2.setX(point.getX() + magnitude * Math.sin(angle));
        this.point2.setY(point.getY() + magnitude * Math.cos(angle));
    }
    public Vector2d(Point2d point, Point2d point2) {
        this(point, Math.sqrt(Math.pow(point2.getX() - point.getX(),2) + Math.pow(point2.getY() - point.getY(),2)),
                Math.atan2(point2.getY() - point.getY(), point2.getX() - point.getX()) * 180 / Math.PI);
    }


    /**
     * adds two vectors together and returns a vector
     * @param vector2
     * @return new vector
     */
    public Vector2d add(Vector2d vector2){
        double x = magnitude * Math.sin(angle); //relative x and y for this.vector
        double y = magnitude * Math.cos(angle);
        
        x += vector2.getMagnitude() * Math.sin(vector2.getAngle()); //add relative x and y for vector2
        y += vector2.getMagnitude() * Math.cos(vector2.getAngle());
        
        x += point.getX(); //add origin x and y
        y += point.getY();
        
        double magnitude = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
        double angle = Math.atan2(y, x) * 180 / Math.PI;
        
        return new Vector2d(new Point2d(x, y), magnitude, angle);
    }

    public Point2d getPoint(){
        return point;
    }
    public Point2d getPoint2(){
        return point2;
    }
    public double getMagnitude() {
        return magnitude;
    }
    public double getAngle(){
        return angle;
    }

    public void setPoint(Point2d point){ this.point = point;}
    public void setPoint2(Point2d point){ this.point2 = point;}
    public void setMagnitude(double magnitude) { this.magnitude = magnitude;}
    public void setAngle(double angle) { this.angle = angle;}

    //TODO: format all info
    public String toString() {
        return String.format(Locale.US, "(%5.2f: %5.2f)", getPoint().getX(), getPoint().getY());
    }
}