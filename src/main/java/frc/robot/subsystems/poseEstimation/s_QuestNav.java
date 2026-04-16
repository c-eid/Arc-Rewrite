// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.poseEstimation;

import java.util.function.Supplier;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.QuestNavConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.util.Touchboard;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class s_QuestNav extends SubsystemBase {
  /** Creates a new s_QuestNav. */

  QuestNav questNav = new QuestNav();
  CommandSwerveDrivetrain s_Swerve;

  private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();

  private final NetworkTable tbTable = networkTable.getTable("touchboard");

  StringSubscriber initalPose = tbTable.getStringTopic("initalPose").subscribe("BlueLeft");

  Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
      0.01, // Trust down to 1cm in X direction
      0.01, // Trust down to 1cm in Y direction
      0.0125 // Trust down to 1 degree rotational
  );

  boolean trustQuest = false;
  boolean initialized = false;

  public s_QuestNav(CommandSwerveDrivetrain s_Swerve) {
    this.s_Swerve = s_Swerve;

    Touchboard.bindOptGroup("initalPose",
        () -> Commands.runOnce(() -> setPoseFromString(() -> initalPose.get())).ignoringDisable(true));

    questNav.onTrackingAcquired(() -> {
      if (initialized == false) {
        trustQuest = true;
        initialized = true;
      }
    });

    questNav.onDisconnected(() -> {
      DriverStation.reportError("Quest disconnected!", false);

    });
    questNav.onTrackingLost(() -> DriverStation.reportError("Quest tracking lost!", false));
    questNav.onLowBattery(20, level -> DriverStation.reportWarning("Quest battery low: " + level + "%", false));
    questNav.onCommandFailure(
        response -> DriverStation.reportError("Pose reset failed: " + response.getErrorMessage(), false));
  }

  private static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  private boolean shouldReject(Pose3d pose) {
    return pose.getX() < 0.0
        || pose.getX() > FIELD_LAYOUT.getFieldLength()
        || pose.getY() < 0.0
        || pose.getY() > FIELD_LAYOUT.getFieldWidth();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    questNav.commandPeriodic();

    SmartDashboard.putBoolean("QuestNav/Trusted", trustQuest);
    SmartDashboard.putBoolean("QuestNav/Connected", questNav.isConnected());
    SmartDashboard.putBoolean("QuestNav/Tracking", questNav.isTracking());
    SmartDashboard.putNumber("QuestNav/Latency", questNav.getLatency());
    questNav.getBatteryPercent().ifPresent(
        b -> SmartDashboard.putNumber("QuestNav/Battery%", b));
    questNav.getTrackingLostCounter().ifPresent(
        c -> SmartDashboard.putNumber("QuestNav/TrackingLostCount", c));

    if (!trustQuest) {
      return;
    }
    // Get the latest pose data frames from the Quest
    PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

    // Loop over the pose data frames and send them to the pose estimator
    for (PoseFrame questFrame : questFrames) {
      // Make sure the Quest was tracking the pose for this frame
      if (questFrame.isTracking()) {
        // Get the pose of the Quest
        Pose3d questPose = questFrame.questPose3d();

        if (shouldReject(questPose)) {
          trustQuest = false;
          return;
        }
        // Get timestamp for when the data was sent
        double timestamp = questFrame.dataTimestamp();

        // Transform by the mount pose to get your robot pose
        Pose3d robotPose = questPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse());

        // You can put some sort of filtering here if you would like!

        // Add the measurement to our estimator
        s_Swerve.addVisionMeasurement(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
      }
    }
  }

  Pose2d BlueLeft = new Pose2d(4.401, 7.215, new Rotation2d());
  Pose2d BlueRight = new Pose2d(4.401, 0.833, new Rotation2d());
  Pose2d RedLeft = FlippingUtil.flipFieldPose(BlueLeft);
  Pose2d RedRight = FlippingUtil.flipFieldPose(BlueRight);

  public void setPoseFromString(Supplier<String> whereSupplier) {
    System.out.println(whereSupplier.get());
    String where = whereSupplier.get();
    if (where.equals("BlueLeft")) {
      setPose(new Pose3d(BlueLeft));
      s_Swerve.resetPose(BlueLeft);

    } else if (where.equals("BlueRight")) {
      setPose(new Pose3d(BlueRight));
      s_Swerve.resetPose(BlueRight);

    } else if (where.equals("RedLeft")) {
      setPose(new Pose3d(RedLeft));
      s_Swerve.resetPose(RedLeft);

    } else if (where.equals("RedRight")) {
      setPose(new Pose3d(RedRight));
      s_Swerve.resetPose(RedRight);

    }
  }

  public void setPose(Pose3d position) {
    questNav.setPose(position.transformBy(Constants.QuestNavConstants.ROBOT_TO_QUEST));
  }

}
