package org.firstinspires.ftc.teamcode.PurePursuit;

import java.util.ArrayList;
import java.util.List;

public class PurePursuitPath extends Waypoint{

    List<Waypoint> Points = new ArrayList<Waypoint>();

    void addWaypoint(double x, double y) {
        Waypoint P1 = new Waypoint(x,y);
        Points.add(P1);
    }

    double computeCurvatureForPoint(Waypoint Current, Waypoint Prev, Waypoint Next) {
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

        k1 = (Math.pow(x1, 2) + Math.pow(y1, 2) - Math.pow(x2, 2) - Math.pow(y2, 2))/(2 * (x1-x2));
        k2 = (y1-y2)/(x1-x2);
        circleCenterY = (Math.pow(x3, 2) + Math.pow(y3, 2) - Math.pow(x2, 2) - Math.pow(y2, 2) - (2 * k1 * (x3 - x2))) / (2 * (y3 - y2 - (k2 * x3) + (k2 * x2)));
        circleCenterX = k1 - (circleCenterY * k2);

        radius = Math.sqrt(Math.pow(x1 - circleCenterX, 2) + Math.pow(y1 - circleCenterY, 2));
        curvature = 1/radius;

        return curvature;
    }

    void computeCurvatureForAllPoints() {
        Waypoint Previous;
        Waypoint Current;
        Waypoint Next;
        for (int i = 0; i < Points.size(); i++) {
            if (i == 0 || i == Points.size() - 1) {
                (Points.get(i)).curvature = 0;
                continue;
            }
            Previous = Points.get(i-1);
            Current = Points.get(i);
            Next = Points.get(i+1);
            double value = computeCurvatureForPoint(Current, Previous, Next);
            Points.get(i).curvature = value;
        }
    }

    void pointInsertion(double spacing) {
        //Todo
    }

    void pathSmoothing(double weight_data, double weight_smooth, double tolerance) {
        //Todo
    }

}
