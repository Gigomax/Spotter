package com.example.spotter.Model;

public class FlexSensor {

    double flex;

   public FlexSensor(){
        flex = 0.00;
    }
    public FlexSensor(double flex){

        this.flex = flex;
    }

    //Getters
    public double getFlex(){

        return flex;
    }
    //Setters
    public void setFlex(double flex){

        this.flex = flex;
    }
}
