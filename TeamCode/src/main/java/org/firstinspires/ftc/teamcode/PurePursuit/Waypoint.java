package org.firstinspires.ftc.teamcode.PurePursuit;

public class Waypoint {
public double xCoord;
public double yCoord;
public double curvature;
public double velocity;

public Waypoint (double x, double y){
    xCoord = x; //This sets the parameter equal to the variables that we created
    yCoord = y;
}

public Waypoint () {
    //Leave it empty as a normal setting
}

}
