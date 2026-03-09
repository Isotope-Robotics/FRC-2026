package frc.robot.Subsystems;

import java.util.NoSuchElementException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class Vision {

    public NetworkTable april;
    private NetworkTableEntry robotPosTargetSpace;
    private NetworkTableEntry targetPosRobotSpace;
    private NetworkTableEntry globalRobotPose;
    private NetworkTableEntry aprilTagIDEntry;
    private NetworkTableEntry tagVisibility;

    public Vision (String tableID) {
        april = NetworkTableInstance.getDefault().getTable(tableID);
        robotPosTargetSpace = april.getEntry("botpose_targetspace");
        targetPosRobotSpace = april.getEntry("targetpose_robotspace");
        globalRobotPose = april.getEntry("botpose");
        aprilTagIDEntry = april.getEntry("tid");
        tagVisibility = april.getEntry("tv");
    }

    public static Pose2d toPose2d (NetworkTableEntry networkTable) throws NoSuchElementException {
        double[] posedata = networkTable.getDoubleArray(new double[0]);
        if (posedata.length == 0) throw new NoSuchElementException("No target was found");
        return new Pose3d(
            posedata[0],
            posedata[1],
            posedata[2],
            new Rotation3d(
                posedata[3],
                posedata[4],
                posedata[5]
            )
        ).toPose2d();
    }

    public Pose2d getRobotPosTargetSpace () throws NoSuchElementException {
        return toPose2d(robotPosTargetSpace);
    }

    public Pose2d getTargetPosRobotSpace () throws NoSuchElementException {
        return toPose2d(targetPosRobotSpace);
    }

    public Pose2d getGlobalRobotPose () throws NoSuchElementException {
        return toPose2d(globalRobotPose);
    }

    public Pose2d getGlobalTargetPose (int id) throws NoSuchElementException {
        return AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark).getTagPose(id).get().toPose2d();
    }

    public Pose2d getGlobalTargetPose () throws NoSuchElementException {
        return getGlobalTargetPose((int)aprilTagIDEntry.getInteger(-1));
    }

    public Translation2d getShooterTargetVector () {
        Translation2d shooterTargetVector;
        if (tagVisibility.getDouble(0.0) > 0) { 
            Pose2d robotPoseTargetSpace = getRobotPosTargetSpace();
            double robotXTargetSpace = robotPoseTargetSpace.getX();
            double robotYTargetSpace = robotPoseTargetSpace.getY();
            double robotYHubCenter = robotYTargetSpace + Constants.Shooter.hubWidth / 2;
            shooterTargetVector = new Translation2d(robotXTargetSpace, robotYHubCenter);
        }
        else {
            shooterTargetVector = new Translation2d();
        }
        return shooterTargetVector;
    }

}