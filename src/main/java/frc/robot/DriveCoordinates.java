package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.Inches;

public class DriveCoordinates {

    //robot width and length are NOT PRECISE. need to measure!
    private static final double robotWidth = 34.0; //the width of the robot with bumper in inches
    private static final double robotLength = 39.0; // width of robot with bumper in inches

    private double offset = 5; //the distance between the edge of the robot and the target, in inches

    public DriveCoordinates(){}

    public Pose2d getPose2d(int aprilTag, boolean faceToward){
        double tagX = 530.49;
        double tagY = 130.17;
        double thetaDegrees = 300;
        // ^ these are for apriltag 6

        double targetX = tagX + ((robotLength + offset) / 2) * Math.cos(thetaDegrees);
        double targetY = tagY + ((robotLength + offset) / 2) * Math.sin(thetaDegrees);

        if (faceToward){
            thetaDegrees -= 180;
        }

        return new Pose2d(Inches.of(targetX), Inches.of(targetY), new Rotation2d(thetaDegrees * Math.PI / 180));
    }
}
