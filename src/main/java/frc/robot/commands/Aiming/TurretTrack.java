// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Aiming;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.s_Turret;
import frc.robot.util.u_Dist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretTrack extends Command {
  /** Creates a new Track. */
  s_Turret s_Turret;
  u_Dist u_Dist;

  private Pose2d robotPose;
  private Pose2d translatedTurretPose;
  private Pose2d translatedGoalPose;

  private Rotation2d toGoal;
  private Rotation2d robotRealtiveRotation;
  private double currentRotationChange;

  private double previousRotation = 0.0;
  private double currentRotation = 0.0;

  public TurretTrack(s_Turret s_Turret, u_Dist u_Dist) {
    addRequirements(s_Turret);

    this.s_Turret = s_Turret;
    this.u_Dist = u_Dist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    u_Dist.updateGoalPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    translatedGoalPose = u_Dist.getGoal(4);
    robotPose = u_Dist.getDrivepose();
    translatedTurretPose = u_Dist.getTurretpose();

    toGoal = Rotation2d.fromRadians(Math.atan2(translatedGoalPose.getY() - translatedTurretPose.getY(),
        translatedGoalPose.getX() - translatedTurretPose.getX()));

    robotRealtiveRotation = Rotation2d
        .fromRadians(toGoal.getRadians() - robotPose.getRotation().getRadians());

    currentRotationChange = (robotRealtiveRotation.getDegrees() - previousRotation);

    if (currentRotationChange > 180) {
      currentRotationChange = currentRotationChange - 360;
    } else if (currentRotationChange < -180) {
      currentRotationChange = currentRotationChange + 360;
    }
    currentRotation += currentRotationChange;

    if (currentRotation > 360) {
      currentRotation = (currentRotation % 360);

    } else if (currentRotation <= -360) {
      currentRotation = (currentRotation % 360);

    }

    s_Turret.setDegrees(currentRotation);

    SmartDashboard.putNumber("Turret/StoredRotation", currentRotation);


    previousRotation = robotRealtiveRotation.getDegrees();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
