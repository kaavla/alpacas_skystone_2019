package org.firstinspires.ftc.teamcode.PurePursuit;

import java.util.ArrayList;
import java.util.List;

public class PurePursuitPath {
    //Creates a list for the user defined points
    List<Waypoint> Points = new ArrayList<Waypoint>();
    //PP stands for Processed Path, a list for the new and user defined points
    List<Waypoint> PP = new ArrayList<Waypoint>();

    //This is a function that adds user defined points to the list Points
    void addUserWaypoint(double x, double y) {
        //creates a waypoint P1
        Waypoint P1 = new Waypoint(x, y, true);
        Points.add(P1);
    }

    //This function adds Waypoints to the PP list, either user defined or not
    void addProcessedWaypoint(double x, double y, boolean userInserted) {
        //creates a waypoint P1
        Waypoint P1 = new Waypoint(x, y, userInserted);
        PP.add(P1);
    }

    //This calculates the curvature at any point
    double computeCurvatureForPoint(Waypoint Current, Waypoint Prev, Waypoint Next) {
        //sets all the variables
        double curvature;
        double radius;
        double k1;
        double k2;
        double circleCenterX;
        double circleCenterY;
        double x1 = Current.xCoord;
        double y1 = Current.yCoord;
        double x2 = Prev.xCoord;
        double y2 = Prev.yCoord;
        double x3 = Next.xCoord;
        double y3 = Next.yCoord;

        //Computations
        k1 = (Math.pow(x1, 2) + Math.pow(y1, 2) - Math.pow(x2, 2) - Math.pow(y2, 2))/(2 * (x1-x2));
        k2 = (y1-y2)/(x1-x2);
        circleCenterY = (Math.pow(x3, 2) + Math.pow(y3, 2) - Math.pow(x2, 2) - Math.pow(y2, 2) - (2 * k1 * (x3 - x2))) / (2 * (y3 - y2 - (k2 * x3) + (k2 * x2)));
        circleCenterX = k1 - (circleCenterY * k2);

        radius = Math.sqrt(Math.pow(x1 - circleCenterX, 2) + Math.pow(y1 - circleCenterY, 2));
        curvature = 1/radius;

        //enters the value of curvature and sets the double "computeCurvatureForPoint" equal to that value
        return curvature;
    }

    //This function applies the Curvature function for all points in the array
    void computeCurvatureForAllPoints() {
        Waypoint Previous;
        Waypoint Current;
        Waypoint Next;
        //the for loop applies the same steps to every point in the PP list
        for (int i = 0; i < PP.size(); i++) {
            //for the first and last point, there is no point before, or after, meaning that the
            //curvature will be = 0. We make the curvature of these points = 0 manually.
            if (i == 0 || i == PP.size() - 1) {
                PP.get(i).curvature = 0;
                continue;
            }
            Previous = PP.get(i-1);
            Current = PP.get(i);
            Next = PP.get(i+1);
            //uses the curvature function to find the curvature of all of the points
            PP.get(i).curvature = computeCurvatureForPoint(Current, Previous, Next);
        }
    }

    double distanceFormula(double x1, double y1, double x2, double y2) {
        double dist;
        dist = Math.sqrt(Math.pow(x2-x1, 2) + Math.pow(y2-y1, 2));
        return dist;
    }

    //This function adds waypoints to the PP list and also adds the pre-defined waypoints to it.
    void pointInsertion(double spacing) {
        Waypoint Current;
        Waypoint Next;
        int numPointsThatFit;

        //for every point within the below requirements, the current point is added, and
        //every point that can fit between with 6 inch spacing is also added to the PP list.
        for (int i = 0; i < Points.size(); i++) {
            Current = Points.get(i);
            Next = Points.get(i+1);

            //add the last user defined point to the new list
            if (i == Points.size()) {
                addProcessedWaypoint(Current.xCoord, Current.yCoord, Current.userDefined);
            }
            //This adds the current point to the list
            addProcessedWaypoint(Current.xCoord, Current.yCoord, Current.userDefined);

            //This finds out how many points can fit between the two points with the needed spacing
            numPointsThatFit = (int) Math.floor(distanceFormula(Current.xCoord, Current.yCoord, Next.xCoord, Next.yCoord)/
                    spacing);

            //This finds the change in x and change in y to get from point A to B
            double ChangeInXStart = Next.xCoord - Current.xCoord;
            double ChangeInYStart = Next.yCoord - Current.yCoord;

            //This finds the change in X and Change in Y in order to have the correct spacing
            //between points
            double ChangeInXSpacing = ChangeInXStart / numPointsThatFit;
            double ChangeInYSpacing = ChangeInYStart / numPointsThatFit;

            //This adds the new points to the list
            for(int index = 1; index <= numPointsThatFit; index++) {
                double newX = Current.xCoord + ChangeInXSpacing * index;
                double newY = Current.yCoord + ChangeInYSpacing * index;
                addProcessedWaypoint(newX, newY, false);
            }
        }
    }

    void pathSmoothing(double weight_data, double weight_smooth, double tolerance) {
        //Todo
    }

}
