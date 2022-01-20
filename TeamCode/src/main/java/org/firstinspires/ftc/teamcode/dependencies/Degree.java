package org.firstinspires.ftc.teamcode.dependencies;

public class Degree {
    public double rotate(double initial, double angle, rotationalDirection direction){
        initial += 180;
        double degrees = (direction == rotationalDirection.CLOCKWISE) ? (initial+angle)%360 : (initial-angle)%360;
        return degrees - 180;
    }

    public double rotate(double initial, double angle){
        return rotate(initial, angle, rotationalDirection.CLOCKWISE);
    }
}
