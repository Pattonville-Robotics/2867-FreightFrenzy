package org.firstinspires.ftc.teamcode.dependencies;

public class rotationalMeasure {
    private double degrees;
    public rotationalMeasure(double degrees){
        this.degrees = degrees;
        this.add(0);
    }
    public void add(double deltaDegrees){
        this.degrees+=deltaDegrees;
        if(this.degrees>180){
            double leeway = this.degrees%180;
            degrees = -180 + leeway;
        }else if(this.degrees<-180){
            double leeway = this.degrees%-180;
            this.degrees = 180 + leeway;
        }
    }
    public void subtract(double deltaDegrees){
        this.add(-deltaDegrees);
    }
    public double get(){
        return degrees;
    }
    public double fixDegrees(double measure){
        if(measure>180){
            double leeway = measure%180;
            measure = -180 + leeway;
        }else if(measure<-180){
            double leeway = measure%-180;
            measure = 180 + leeway;
        }
        return measure;
    }
}
