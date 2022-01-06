package org.firstinspires.ftc.teamcode.dependencies;

import org.opencv.core.Point;
import org.opencv.core.Rect;

public class Region {
    public Rect bounds;
    public int x;
    public int y;
    public int width;
    public int height;
    public Region(int x, int y, int width, int height){
        this.x=x;
        this.y=y;
        this.width=width;
        this.height=height;
        bounds = new Rect(new Point(x, y), new Point(x+width, y+height));
    }
    public Point getTopLeft(){
        return new Point(x,y);
    }
    public Point getTopRight(){
        return new Point(x+width,y);
    }
    public Point getBottomLeft(){
        return new Point(x,y+height);
    }
    public Point getBottomRight(){
        return new Point(x+width,y+height);
    }
}
