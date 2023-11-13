package com.example.spotter.Model;

public class ImuSensor {


    double angle1_x;
    double angle1_y;
    double angle2_x;
    double angle2_y;
    double relative_x;
    double relative_y;



   public ImuSensor(){

        angle1_x = 0.00;
        angle1_y = 0.00;
        angle2_x = 0.00;
        angle2_y = 0.00;
        relative_x = 0.00;
        relative_y = 0.00;
    }
    public ImuSensor(double angle1_x,double angle1_y,double angle2_x,double angle2_y){

        this.angle1_x = angle1_x;
        this.angle1_y = angle1_y;
        this.angle2_x = angle2_x;
        this.angle2_y = angle2_y;
        this.relative_x = calculateRelativeAngle(angle1_x, angle2_x);
        this.relative_y = calculateRelativeAngle(angle1_y, angle2_y);
    }

    //Getters
    public double getAngle1_x() {

        return angle1_x;
    }
    public double getAngle1_y() {

        return angle1_y;
    }
    public double getAngle2_x() {

        return angle2_x;
    }
    public double getAngle2_y() {

        return angle2_y;
    }
    public double getRelative_x() {

        return relative_x;
    }
    public double getRelative_y() {

        return relative_y;
    }
    //Setters
    public void setAngle(double angle1_x, double angle1_y, double angle2_x, double angle2_y ){

        this.angle1_x = angle1_x;
        this.angle1_y = angle1_y;
        this.angle2_x = angle2_x;
        this.angle2_y = angle2_y;
    }
    public void setAngle1_x(double angle1_x) {

        this.angle1_x = angle1_x;
    }
    public void setAngle1_y(double angle1_y) {

        this.angle1_y = angle1_y;
    }
    public void setAngle2_x(double angle2_x) {

        this.angle2_x = angle2_x;
    }
    public void setAngle2_y(double angle2_y) {

        this.angle2_y = angle2_y;
    }
    public void setRelative_x(double relative_x) {

        this.relative_x = relative_x;
    }
    public void setRelative_y(double relative_y) {

        this.relative_y = relative_y;
    }
    //Methods
    public double calculateRelativeAngle(double angle1, double angle2){
       double relative;
        if (angle2 > 0)
        {
            angle2 = - angle2;
        }
       if (angle2 - angle1 < 0)
        {
            relative = angle2 - angle1 + 360;
        }
        else
        {
            relative = angle2 - angle1;
        }
        if (relative > 300) relative = 360 - relative;
       return relative;
    }

}
