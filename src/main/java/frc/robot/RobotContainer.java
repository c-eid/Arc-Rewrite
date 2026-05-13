// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.w3c.dom.ranges.DocumentRange;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.IO.TurretIO;

import frc.robot.commands.Aiming.HoodTrack;
import frc.robot.commands.Aiming.TurretTrack;
import frc.robot.commands.Development.TouchboardShootAngle;
import frc.robot.commands.Development.Zero;
import frc.robot.commands.Emergency.Reverse;
import frc.robot.commands.Intake.HomeIntake;
import frc.robot.commands.Intake.Intaking;
import frc.robot.commands.Intake.StoreIntake;
import frc.robot.commands.Launch.Shoot;
import frc.robot.commands.Serialization.Belt;
import frc.robot.commands.Serialization.Serialize;
import frc.robot.subsystems.s_Hood;
import frc.robot.subsystems.s_Belt;
import frc.robot.subsystems.s_Intake;
import frc.robot.subsystems.s_Serializer;
import frc.robot.subsystems.s_Turret;
import frc.robot.subsystems.s_Shooter;
import frc.robot.subsystems.drive.s_Drivetrain;
import frc.robot.subsystems.poseEstimation.s_QuestNav;
import frc.robot.util.u_Dist;

public class RobotContainer {

  // Controllers
  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController copilot = new CommandXboxController(1);

  // IO
  TurretIO turretSimulation = new TurretIO();

  // Subsystems
  s_Drivetrain s_Swerve = new s_Drivetrain();
  s_QuestNav s_QNav = new s_QuestNav(s_Swerve.getDrivetrain());
  s_Intake s_Intake = new s_Intake();
  s_Belt s_Belt = new s_Belt();
  s_Turret s_Turret = new s_Turret(turretSimulation);
  s_Hood s_Hood = new s_Hood(turretSimulation);
  s_Shooter s_Shooter = new s_Shooter();
  s_Serializer s_Serializer = new s_Serializer();

  // Util
  u_Dist u_Dist = new u_Dist(s_Swerve);

  // Commands
  Intaking intake = new Intaking(s_Intake);
  StoreIntake intakeUp = new StoreIntake(s_Intake);

  HoodTrack hoodTrack = new HoodTrack(s_Hood, u_Dist);
  TurretTrack turretTrack = new TurretTrack(s_Turret, u_Dist);

  Shoot revShooter = new Shoot(s_Shooter, u_Dist);

  Belt belt = new Belt(s_Belt);
  Serialize serialize = new Serialize(s_Serializer);
  
  Zero zeroAll = new Zero(s_Turret, s_Hood, s_Intake);
  TouchboardShootAngle touchboardShootAngle = new TouchboardShootAngle(s_Shooter, s_Hood);

  HomeIntake homeIntake = new HomeIntake(s_Intake);

  Reverse reverseAll = new Reverse(s_Shooter, s_Belt, s_Serializer, s_Intake);

  public RobotContainer() {
    s_Swerve.bindControllers(s_QNav, driver);

    configureDefaults();
    configureModifierBindings();
    configureCommandBindings();
  }

  private void configureDefaults() {
    s_Turret.setDefaultCommand(turretTrack);
    s_Hood.setDefaultCommand(hoodTrack);
  }

  private void configureModifierBindings() {

  }

  private void configureCommandBindings() {

    driver.leftTrigger(0.3).whileTrue(
        intake);

    driver.leftBumper().whileTrue(
        reverseAll);

    driver.rightTrigger(0.3).and(() -> s_Shooter.atSetpoint()).whileTrue(
        belt).whileTrue(
        serialize);
    driver.rightTrigger(0.3).whileTrue(
        revShooter);

    driver.rightBumper().whileTrue(
        revShooter).whileTrue(
        belt).whileTrue(
        serialize);

    driver.pov(0).onTrue(
        intakeUp);

    driver.pov(180).onTrue(
        homeIntake);

    driver.rightStick().toggleOnTrue(
        zeroAll);

    driver.x().whileTrue(
        touchboardShootAngle);

    driver.y().whileTrue(
        belt).whileTrue(
        serialize);
    // driver.rightBumper().whileTrue()

  }

  PathConstraints constraints = new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  public Command getAutonomousCommand() {
    return Commands.none(); //s_Hood.getSysIdRoutine();
  }
}
