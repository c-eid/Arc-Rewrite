// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.IO.TurretIO;

import frc.robot.commands.Aiming.HoodTrack;
import frc.robot.commands.Aiming.TurretTrack;
import frc.robot.commands.Development.TouchboardShootAngle;
import frc.robot.commands.Development.Zero;
import frc.robot.commands.Emergency.Lock;
import frc.robot.commands.Emergency.Reverse;
import frc.robot.commands.Emergency.TrenchShot;
import frc.robot.commands.Emergency.Zeroing;
import frc.robot.commands.Intake.Bounce;
import frc.robot.commands.Intake.HomeIntake;
import frc.robot.commands.Intake.Intaking;
import frc.robot.commands.Intake.OpenIntake;
import frc.robot.commands.Intake.StoreIntake;
import frc.robot.commands.Launch.Shoot;
import frc.robot.commands.Serialization.Belt;
import frc.robot.commands.Serialization.Serialize;
import frc.robot.commands.Serialization.SerializeUnjam;
import frc.robot.commands.Serialization.StopSerializer;
import frc.robot.generated.TunerConstants;
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
  s_QuestNav s_QNav = new s_QuestNav(s_Swerve.getDrivetrain(), driver);
  s_Intake s_Intake = new s_Intake();
  s_Belt s_Belt = new s_Belt();
  s_Turret s_Turret = new s_Turret(turretSimulation);
  s_Hood s_Hood = new s_Hood(turretSimulation);
  s_Shooter s_Shooter = new s_Shooter();
  s_Serializer s_Serializer = new s_Serializer();

  Bounce bounce = new Bounce(s_Intake);

  // Util
  u_Dist u_Dist = new u_Dist(s_Swerve);

  // Commands
  Intaking intake = new Intaking(s_Intake);
  StoreIntake intakeUp = new StoreIntake(s_Intake);

  HoodTrack hoodTrack = new HoodTrack(s_Hood, u_Dist);
  TurretTrack turretTrack = new TurretTrack(s_Turret, u_Dist, TunerConstants.getInstance());

  Shoot revShooter = new Shoot(s_Shooter, u_Dist);

  Belt belt = new Belt(s_Belt);
  Serialize serialize = new Serialize(s_Serializer);
  SerializeUnjam serializeUnjam = new SerializeUnjam(s_Belt, s_Serializer);
  
  Zero zeroAll = new Zero(s_Turret, s_Hood, s_Intake);
  TouchboardShootAngle touchboardShootAngle = new TouchboardShootAngle(s_Shooter, s_Hood);

  HomeIntake homeIntake = new HomeIntake(s_Intake);

  Reverse reverseAll = new Reverse(s_Shooter, s_Belt, s_Serializer, s_Intake);

  Zeroing zeroTurret = new Zeroing(s_Turret, ()-> driver.getRightX());

  Lock lockTurret = new Lock(s_Turret, s_Hood);
  TrenchShot trenchShot = new TrenchShot(s_Shooter);

 public Command currentAuto;

  public RobotContainer() {
    s_Swerve.bindControllers(s_QNav, driver);

    configureDefaults();
    configureModifierBindings();
    configureCommandBindings();
    bindNamedCommands();

    currentAuto = new PathPlannerAuto("2 swipe right side mid");

  }

  private void configureDefaults() {
    s_Turret.setDefaultCommand(turretTrack);
    s_Hood.setDefaultCommand(hoodTrack);
  }

  private void configureModifierBindings() {
    driver.rightTrigger(0.3).onTrue( Commands.runOnce(()-> s_Swerve.setSpeedModifier(0.2)));

    driver.rightTrigger(0.3).onFalse( Commands.runOnce(()-> s_Swerve.setSpeedModifier(1)));

  }

  private void configureCommandBindings() {

    driver.leftTrigger(0.3).whileTrue(
        intake);

    driver.leftBumper().whileTrue(
        reverseAll);

    driver.rightTrigger(0.3).debounce(0.3, DebounceType.kBoth).whileTrue(
        bounce)
        // .and(() -> s_Shooter.atSetpoint())
        .whileTrue(serializeUnjam);
        // .whileTrue(
        // belt).whileTrue(
        // serialize);

    driver.leftStick().and(driver.x()).toggleOnTrue(zeroTurret);
    driver.leftStick().and(driver.y()).toggleOnTrue(lockTurret)
         .toggleOnTrue(Commands.run(()-> {}).beforeStarting(()-> s_Shooter.setTrenchShot(true)).finallyDo(()-> s_Shooter.setTrenchShot(false)));
    
    // driver.rightTrigger(0.3).whileTrue(5
    //     belt);
        
    // driver.rightTrigger(0.3).whileTrue(
    //     Commands.waitSeconds(1).andThen(serialize) );

    driver.rightTrigger(0.3).and(new Trigger(()-> !s_Shooter.getTrenchShot())).whileTrue(
        revShooter);

    driver.rightTrigger(0.3).and(new Trigger(()-> s_Shooter.getTrenchShot())).whileTrue(
        trenchShot);

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
  private void bindNamedCommands(){
      NamedCommands.registerCommand("intake",new Intaking(s_Intake));

        NamedCommands.registerCommand("sintake",new OpenIntake(s_Intake));

        NamedCommands.registerCommand("revshoot", new Shoot(s_Shooter, u_Dist));
        NamedCommands.registerCommand("intakeup", new StoreIntake(s_Intake));

        NamedCommands.registerCommand("spindex",  new SerializeUnjam(s_Belt, s_Serializer));

        NamedCommands.registerCommand("stopspindexer", new StopSerializer(s_Belt, s_Serializer));
        NamedCommands.registerCommand("outtakeshooter", new Reverse(s_Shooter, s_Belt, s_Serializer, s_Intake));

        NamedCommands.registerCommand("stopshoot",Commands.runOnce(() -> s_Shooter.setRPM(0), s_Shooter));
        NamedCommands.registerCommand("Zero", Commands.runOnce(() -> s_Swerve.getDrivetrain().seedFieldCentric()));
        
        NamedCommands.registerCommand("shoot",Commands.none());
        NamedCommands.registerCommand("outtakespindexer",Commands.none());        
        NamedCommands.registerCommand("track turret", Commands.none());
        NamedCommands.registerCommand("track left", Commands.none());
        NamedCommands.registerCommand("zerohood", Commands.none());
        NamedCommands.registerCommand("track right", Commands.none());
        NamedCommands.registerCommand("sethood", Commands.none());

  }

  PathConstraints constraints = new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  public Command getAutonomousCommand() {
    return currentAuto;
  }
}
