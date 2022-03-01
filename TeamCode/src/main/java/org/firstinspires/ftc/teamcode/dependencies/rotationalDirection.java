package org.firstinspires.ftc.teamcode.dependencies;

public enum rotationalDirection {
    CLOCKWISE, COUNTERCLOCKWISE;

    public rotationalDirection other(){
        return (this==rotationalDirection.CLOCKWISE) ? COUNTERCLOCKWISE : CLOCKWISE;
    }
}
