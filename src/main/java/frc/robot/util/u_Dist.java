// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meter;

import java.util.function.Supplier;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.s_Drivetrain;

/** Add your docs here. */
public class u_Dist {
    // Subsystems
    s_Drivetrain s_Drivetrain;

    // Drivetrain
    CommandSwerveDrivetrain drivetrain;

    // Network Tables
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("uDist");

    // Pose sender
    StructPublisher<Pose2d> virtualGoalPoseSender = table.getStructTopic("Virtual Goal", Pose2d.struct).publish();

    // Drivetrain Suppliers
    Supplier<Pose2d> drivePose = () -> drivetrain.getState().Pose;
    Supplier<ChassisSpeeds> driveSpeeds = () -> drivetrain.getState().Speeds;

    // Field Speeds Supplier
    private Supplier<ChassisSpeeds> fieldSpeedsSupplier = () -> ChassisSpeeds
            .fromRobotRelativeSpeeds(driveSpeeds.get(), drivePose.get().getRotation());

    // Alliance (For flipping speed compensation signs)
    String alliance = "";

    // Goal pose
    Pose2d goalPose = new Pose2d();

    // Constants for alliance zones
    final double blueDistMeters = FieldConstants.ALLIANCE_ZONE.in(Meter) + 1.2192;
    final double redDistMeters = FieldConstants.FIELD_LENGTH.minus(FieldConstants.ALLIANCE_ZONE).in(Meter) - 1.2192;

    private final Pose2d HUB_BLUE_POSE = new Pose2d(4.620419, 4.034631, new Rotation2d());
    private final Pose2d HUB_RED_POSE = FlippingUtil.flipFieldPose(HUB_BLUE_POSE);

    private final Pose2d FEED_BLUE_LEFT = new Pose2d(3, 6, new Rotation2d());
    private final Pose2d FEED_BLUE_RIGHT = new Pose2d(3, 1.6, new Rotation2d());
    private final Pose2d FEED_RED_LEFT = FlippingUtil.flipFieldPose(FEED_BLUE_RIGHT);
    private final Pose2d FEED_RED_RIGHT = FlippingUtil.flipFieldPose(FEED_BLUE_LEFT);
    private final double halfField = FieldConstants.FIELD_WIDTH.div(2).in(Meter);

    public u_Dist(s_Drivetrain s_Drivetrain) {

        this.s_Drivetrain = s_Drivetrain;

        this.drivetrain = s_Drivetrain.getDrivetrain();

        bindAllianceTriggers();
    }

    public Pose2d getDrivepose() {
        return drivePose.get();
    }

    public Pose2d getTurretpose() {
        return drivePose.get().transformBy(new Transform2d(0.196, 0.0, new Rotation2d()));
    }

    public void updateGoalPose() {
        // setHubAsGoal(alliance);

        if (inAllianceZone(getTurretpose())) {
            setHubAsGoal(alliance);
        } else {
            setFeedAsGoal(getTurretpose(), alliance);
        }
    }

    private void setHubAsGoal(String alliance) {
        switch (alliance) {
            case "":
                this.goalPose = HUB_BLUE_POSE;
                break;
            case "blue":
                this.goalPose = HUB_BLUE_POSE;
                break;
            case "red":
                this.goalPose = HUB_RED_POSE;
                break;
            default:
                break;
        }
    }

    private void setFeedAsGoal(Pose2d turretPose, String alliance) {
        Pose2d feedGoal;
        if (alliance == "red") {
            if (turretPose.getY() > halfField) {
                feedGoal = FEED_RED_LEFT;
            } else {
                feedGoal = FEED_RED_RIGHT;
            }

        } else {
            if (turretPose.getY() > halfField) {
                feedGoal = FEED_BLUE_LEFT;
            } else {
                feedGoal = FEED_BLUE_RIGHT;
            }
        }

        this.goalPose = feedGoal;
    }

    // Get the distance from the robot turret to the goal pose without speed
    // compensation.
    public Distance getRawDistFromTurret() {
        updateGoalPose();
        return Meter.of(getTurretpose().getTranslation().getDistance(this.goalPose.getTranslation()));
    }

    public Distance getRawDistFromTurret(Pose2d goalPose) {
        return Meter.of(getTurretpose().getTranslation().getDistance(goalPose.getTranslation()));
    }

    // Get the distance from the robot to the goal pose with speed compensation and
    // compensation for turret position relative to robot.
    // --- NEW: Cached State Variables ---
    // store cached distance as primitive feet to avoid Distance object allocations
    private double cachedDistFeet = 0.0;
    private Pose2d cachedGoalPose = new Pose2d();

    // Set a constant iteration count for your speed compensation math
    private static final int ITERATION_COUNT = 4;
    // Publish to NetworkTables every N periodic cycles to reduce NT overhead
    private static final int PUBLISH_EVERY = 2;
    private int publishCounter = 0;

    public void periodic() {
        updateGoalPose();

        // Cache values used across iterations to avoid repeated expensive calls
        Pose2d turretPose = getTurretpose();
        double rawDistMeters = turretPose.getTranslation().getDistance(this.goalPose.getTranslation());
        double rawDistFeet = rawDistMeters * 3.28084; // meters -> feet
        double tof = u_Lut.getTofFrom(rawDistFeet);
        double speedModifier = findSpeedModifier();
        ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

        Pose2d prevPose = new Pose2d(
                goalPose.getX() + speedModifier * fieldSpeeds.vxMetersPerSecond * tof,
                goalPose.getY() + speedModifier * fieldSpeeds.vyMetersPerSecond * tof, new Rotation2d());

        double prevDistMeters = turretPose.getTranslation().getDistance(prevPose.getTranslation());
        double prevDistFeet = prevDistMeters * 3.28084;

        // Iteratively refine the goal pose and distance using cached turretPose and fieldSpeeds
        for (int i = 0; i < ITERATION_COUNT; ++i) {
            double tof2 = u_Lut.getTofFrom(prevDistFeet);
            // assume speed modifier and field speeds change slowly relative to iterations
            prevPose = new Pose2d(
                    goalPose.getX() + speedModifier * fieldSpeeds.vxMetersPerSecond * tof2,
                    goalPose.getY() + speedModifier * fieldSpeeds.vyMetersPerSecond * tof2, new Rotation2d());

            // recompute prevDist in feet using cached turretPose
            prevDistMeters = turretPose.getTranslation().getDistance(prevPose.getTranslation());
            prevDistFeet = prevDistMeters * 3.28084;
        }

        // Save to cache (store feet as primitive)
        this.cachedGoalPose = prevPose;
        this.cachedDistFeet = prevDistFeet;

        // Publish to NetworkTables at reduced frequency to lower NT overhead
        publishCounter = (publishCounter + 1) % PUBLISH_EVERY;
        if (publishCounter == 0) {
            virtualGoalPoseSender.set(prevPose);
        }
    }

    public Distance getDist() {
        return Feet.of(cachedDistFeet);
    }

    public Pose2d getGoal() {
        return cachedGoalPose;
    }

    public void bindAllianceTriggers() {
        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop()).onTrue(Commands.runOnce(() -> {
            if (DriverStation.getAlliance().isPresent()) {
                if (DriverStation.getAlliance().get() == Alliance.Red)
                    alliance = "red";
                else if (DriverStation.getAlliance().get() == Alliance.Blue)
                    alliance = "blue";
            }

        }));
    }

    public double findSpeedModifier() {
        switch (alliance) {
            case "":
                return -1;
            case "blue":
                return -1;
            case "red":
                return -1;
        }
        return -1;

    }

    public boolean inAllianceZone(Pose2d turretPose) {

        switch (alliance) {
            case "":
                if (DriverStation.getAlliance().isPresent()) {
                    if (DriverStation.getAlliance().get() == Alliance.Blue) {
                        alliance = "blue";
                    } else if (DriverStation.getAlliance().get() == Alliance.Red) {
                        alliance = "red";
                    }
                    return true;
                }
                break;
            case "blue":
                if (turretPose.getX() < blueDistMeters) {
                    return true;
                }
                return false;
            case "red":
                if (turretPose.getX() > redDistMeters) {
                    return true;
                }
                return false;
        }
        return true;
    }

}
