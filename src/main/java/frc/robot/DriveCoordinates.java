package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class DriveCoordinates {
    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    //robot width and length are NOT PRECISE. need to measure!
    private static final double robotWidth = 34.0; //the width of the robot with bumper in inches
    private static final double robotLength = 39.0; // width of robot with bumper in inches

    private double offset = 5; //the distance between the edge of the robot and the target, in inches

    public DriveCoordinates(){}

    //work in progress. this will generate Pose2d coordinates relative to any apriltag,
    // with an offset to the left or right (for aligning to left or right branches),
    // and a parameter for whether the robot should face toward or away from  (for facing toward the reef or away from the coral station).
    public Pose2d getPose2d(int aprilTag, double parallelOffset, double normalOffset, boolean faceTowardTag){
        double tag6X = 530.49;
        double tag6Y = 130.17;
        double thetaDegrees6 = 300;
        // ^ these are for apriltag 6

        Pose3d tagPose3D = layout.getTagPose(6).get();
        double tagX = tagPose3D.getX();
        double tagY = tagPose3D.getY();
        double thetaDegrees = tagPose3D.getRotation().getAngle()*(180/Math.PI);

        double targetX = tagX + ((robotLength + offset) / 2) * Math.cos(thetaDegrees);
        double targetY = tagY + ((robotLength + offset) / 2) * Math.sin(thetaDegrees);

        // double targetX = tagX + ((robotLength + normalOffset) / 2) * Math.cos(thetaDegrees);
        // double targetY = tagY + ((robotLength + normalOffset) / 2) * Math.sin(thetaDegrees);

        if (faceTowardTag){
            thetaDegrees -= 180;
        }

        return new Pose2d(Inches.of(targetX), Inches.of(targetY), new Rotation2d(thetaDegrees * Math.PI / 180));
    }
}
