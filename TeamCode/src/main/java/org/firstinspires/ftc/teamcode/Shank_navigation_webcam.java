package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
<<<<<<< HEAD
=======
import com.qualcomm.robotcore.util.RobotLog;
>>>>>>> 5e66efaa3ff04af10d785b413b876cedf75911a9

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
<<<<<<< HEAD
=======
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
>>>>>>> 5e66efaa3ff04af10d785b413b876cedf75911a9
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@TeleOp(name="SHANK - Navigation Nav Webcam", group ="Concept")
//@Disabled
public class Shank_navigation_webcam extends LinearOpMode {

<<<<<<< HEAD
=======
    // Constants
    private static final int     MAX_TARGETS    =   13;
    private static final double  ON_AXIS        =  10;      // Within 1.0 cm of target center-line
    private static final double  CLOSE_ENOUGH   =  20;      // Within 2.0 cm of final target standoff

    public  static final double  YAW_GAIN       =  0.018;   // Rate at which we respond to heading error
    public  static final double  LATERAL_GAIN   =  0.2;  // Rate at which we respond to off-axis error
    public  static final double  AXIAL_GAIN     =  0.2;  // Rate at which we respond to target distance errors
    public final double TARGET_DISTANCE =  400.0;    // Hold robot's center 400 mm from target


>>>>>>> 5e66efaa3ff04af10d785b413b876cedf75911a9
    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the doubleeiihcckgbrrruifcjtvdufgjrjdnechtthdkdfnbnndk
     *  quotes.
     */
    private static final String VUFORIA_KEY =
            "AXSRCQP/////AAABmZeKs8E+bExYlViUoU4W3x9D+ZqA3HLfy3PxlWiz0h5wh/awa/Oe9lra0C0CqlyRducvIyV5egl7zTYvGsbA34h3hCAV1eQtpnzQtYulVYRxD6W2Lnl47omLOHjXv3fTXLPnPDBugwDQUCqw4tN58FFEN5xoKEIPWwaQuOg43WHpfa6wenMv+bxuiwxM0Ciy+2gad/kkc+MTWzsFAL/yjTQhq718BNLr1FYZveMEwFHS43kILSKaL/+3/YGqd677av/z5tVDLkSRPUDuEYKIB1P0uCJd5AhIPnVvNigICEUxETMZiEt0RmKoQ3x9S6Y8AelTJgpHeuVgDHy5BmNP877er8Bsqr+WfHGho64CNbUx\n";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
<<<<<<< HEAD
    WebcamName webcamName = null;
=======
    //WebcamName webcamName = null;
>>>>>>> 5e66efaa3ff04af10d785b413b876cedf75911a9

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

<<<<<<< HEAD
    @Override public void runOpMode() {
        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
=======
    private boolean             targetFound;    // set to true if Vuforia is currently tracking a target
    private String              targetName;     // Name of the currently tracked target
    private double              robotX;         // X displacement from target center
    private double              robotY;         // Y displacement from target center
    private double              targetX;         // X displacement from target center
    private double              targetY;         // Y displacement from target center
    private double              robotBearing;   // Robot's rotation around the Z axis (CCW is positive)
    private double              abs_robotBearing;   // Robot's rotation around the Z axis (CCW is positive)
    private double              targetRange;    // Range from robot's center to target in mm
    private double              targetBearing;  // Heading of the target , relative to the robot's unrotated center
    private double              relativeBearing;// Heading to the target from the robot's current bearing.
    private double Y;
    private double L;
    private double A;


    //   eg: a Positive RelativeBearing means the robot must turn CCW to point at the target image.
    private CallistoHW robotCallisto = new CallistoHW();

    VuforiaTrackables targetsSkyStone;

    @Override public void runOpMode() {

        boolean closeenough = false;
        robotCallisto.init(hardwareMap);

        /*
         * Retrieve the camera we are to use.
         */
        //webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
>>>>>>> 5e66efaa3ff04af10d785b413b876cedf75911a9

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
<<<<<<< HEAD

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;
=======
        parameters.cameraDirection   = CAMERA_CHOICE;
        //parameters.useExtendedTracking = false;
>>>>>>> 5e66efaa3ff04af10d785b413b876cedf75911a9

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

<<<<<<< HEAD
        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
=======

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
>>>>>>> 5e66efaa3ff04af10d785b413b876cedf75911a9

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));
        //-x to X
        blueRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
        //dz 90 to -90
        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));
        //dz 90 to -90
        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 8.75f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 2.6f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        targetsSkyStone.activate();
<<<<<<< HEAD
        while (!isStopRequested()) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
=======


        while (!isStopRequested()) {

            closeenough = false;
            //if (targetsAreVisible()  && gamepad1.left_bumper) {
            if (targetsAreVisible() ) {
                closeenough = cruiseControl(TARGET_DISTANCE);
            }
            // Build telemetry messages with Navigation Information;
            addNavTelemetry();

            //  Move the robot according to the pre-determined axis motions
            if (closeenough == false) {
                robotCallisto.moveHolonomic(L, A, Y);
            } else {
                telemetry.addData("SHANK","Reached....");
            }
            telemetry.update();


>>>>>>> 5e66efaa3ff04af10d785b413b876cedf75911a9
        }

        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();
    }
<<<<<<< HEAD
=======
    public void addNavTelemetry() {
        if (targetFound)
        {
            // Display the current visible target name, robot info, target info, and required robot action.
            telemetry.addData("Visible", targetName);
            telemetry.addData("Robot", "[X]:[Y] (B) [%5.0f in]:[%5.0f inch] (%4.0f°)(%4.0f°)",
                    robotX/mmPerInch, robotY/mmPerInch, robotBearing, abs_robotBearing);
            telemetry.addData("Target", "[X]:[Y] [%5.0f inch]:[%5.0f inch] ",
                    targetX/mmPerInch, targetY/mmPerInch);
            telemetry.addData("Target", "[R] (B):(RB) [%5.0f] (%4.0f°):(%4.0f°)",
                    targetRange/mmPerInch, targetBearing, relativeBearing);
            telemetry.addData("- Turn    ", "%s %4.0f°",  relativeBearing < 0 ? ">>> CW " : "<<< CCW", Math.abs(relativeBearing));
            telemetry.addData("- Strafe  ", "%s %5.0f inch", (robotX - targetX) < 0 ? "RIGHT" : "LEFT", Math.abs(robotX - targetX)/mmPerInch);
            telemetry.addData("- Distance", "%5.0f inch", Math.abs(robotY - targetY)/mmPerInch);
        }
        else
        {
            telemetry.addData("Visible", "- - - -" );
        }
    }


    public boolean cruiseControl(double standOffDistance) {
        boolean closeEnough;

        // Priority #1 Rotate to always be pointing at the target (for best target retention).
        Y  = ((relativeBearing) * YAW_GAIN);

        // Priority #2  Drive laterally based on distance from X axis (same as y value)
        L  =((targetX - robotX ) * LATERAL_GAIN);

        // Priority #3 Drive forward based on the desiredHeading target standoff distance
        A  = (((targetY - robotY) - standOffDistance) * AXIAL_GAIN);

        // Send the desired axis motions to the robot hardware.

        // Determine if we are close enough to the target for action.
        closeEnough = ( ((Math.abs(targetY - robotY ) - standOffDistance) < CLOSE_ENOUGH) &&
                (Math.abs(targetX - robotX) < ON_AXIS));

        return (closeEnough);
    }

    public boolean targetsAreVisible()  {

        int targetTestID = 0;

        // Check each target in turn, but stop looking when the first target is found.
        while ((targetTestID < MAX_TARGETS) && !targetIsVisible(targetTestID)) {
            targetTestID++ ;
        }

        return (targetFound);
    }

    public boolean targetIsVisible(int targetId) {

        VuforiaTrackable target = targetsSkyStone.get(targetId);
        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener)target.getListener();
        OpenGLMatrix location  = null;
        VectorF trans;
        Orientation rot;

        // if we have a target, look for an updated robot position
        if ((target != null) && (listener != null) && listener.isVisible()) {
            targetFound = true;
            targetName = target.getName();

            RobotLog.ii("SHANK", "targetIsVisible - Found Target - %s(%d)", targetName, targetId);

            trans = target.getFtcFieldFromTarget().getTranslation();
            targetX = trans.get(0);
            targetY = trans.get(1);
            RobotLog.ii("SHANK", "targetIsVisible - Target - [%2.2f, %2.2f]", targetX/mmPerInch, targetY/mmPerInch);

            // If we have an updated robot location, update all the relevant tracking information
            location  = listener.getUpdatedRobotLocation();
            if (location != null) {

                // Create a translation and rotation vector for the robot.
                trans = location.getTranslation();
                rot = Orientation.getOrientation(location, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Robot position is defined by the standard Matrix translation (x and y)
                robotX = trans.get(0);
                robotY = trans.get(1);

                // Robot bearing (in +vc CCW cartesian system) is defined by the standard Matrix z rotation
                robotBearing = rot.thirdAngle;
                abs_robotBearing = robotBearing;
                if (abs_robotBearing  < 0)
                {
                    abs_robotBearing  = (abs_robotBearing ) + 360;
                }

                RobotLog.ii("SHANK", "targetIsVisible - Robot - [%2.2f, %2.2f, %2.2f, %2.2f]",
                        robotX/mmPerInch, robotY/mmPerInch,robotBearing ,abs_robotBearing );

                // target range is based on distance from robot position to origin.
                //targetRange = Math.hypot(robotX, robotY);
                targetRange = Math.hypot((robotX-targetX), (robotY-targetY));

                // target bearing is based on angle formed between the X axis to the target range line
                if (targetId == 9  ||
                    targetId == 10)
                {
                    //Red or Blue
                    targetBearing = Math.toDegrees(Math.asin(Math.abs(robotY - targetY) / targetRange));
                    if (targetX < robotX)
                    {
                        targetBearing = 180 - targetBearing;
                    } else
                    {
                        targetBearing = targetBearing;
                    }
                } else
                {
                    //Not supported
                    //Front or Read
                    targetBearing = Math.toDegrees(Math.asin(Math.abs(robotX - targetX) / targetRange));
                }

                // Target relative bearing is the target Heading relative to the direction the robot is pointing.
                //relativeBearing = targetBearing - robotBearing;
                relativeBearing = targetBearing - robotBearing;
                //if (relativeBearing > 180)
                //{
                //    relativeBearing = -1*(360-relativeBearing); //-1 -> Go CCW instead of CW
                //}
                RobotLog.ii("SHANK", "targetIsVisible - targetRange(%2.2f),targetBearing(%2.2f),relativeBearing(%2.2f)]",
                        targetRange,targetBearing,relativeBearing);

            }
            targetFound = true;
        }
        else  {
            // Indicate that there is no target visible
            targetFound = false;
            targetName = "None";
        }

        return targetFound;
    }

>>>>>>> 5e66efaa3ff04af10d785b413b876cedf75911a9
}

