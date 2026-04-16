// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.FieldConstants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.poseEstimation.s_QuestNav;

public class s_Drivetrain extends SubsystemBase {
  /** Creates a new s_Example. */
  public static s_Drivetrain m_Instance;
  private CommandSwerveDrivetrain drivetrain = TunerConstants.getInstance();

  private DoubleSupplier xStick = ()->0;
  private DoubleSupplier yStick = ()->0;
  private DoubleSupplier rotStick = ()->0;

  private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  private final NetworkTable driveStateTable = networkTable.getTable("DriveState/CommandTrain");

  private DoublePublisher PIDY = driveStateTable.getDoubleTopic("CALCY").publish();
  private DoublePublisher TrenchY = driveStateTable.getDoubleTopic("TrenchY").publish();
  private DoublePublisher RobotY = driveStateTable.getDoubleTopic("RobotY").publish();
  private DoublePublisher SpeedMod = driveStateTable.getDoubleTopic("SpeedMod").publish();

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate =  RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.FieldCentricFacingAngle trenchDriveRequest = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withHeadingPID(12, 0, 0);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private DoubleSupplier setX = ()->0;
  private DoubleSupplier setY = ()->0;
  private DoubleSupplier calcY = ()->0;
  // private DoubleSupplier setRot =  ()->0;
  private Rotation2d closestRot;

  private Supplier<Pose2d> robotPose = () -> drivetrain.getState().Pose;

  final SwerveRequest.Idle idle = new SwerveRequest.Idle();

  private Command idleDrive = drivetrain.applyRequest(() -> idle);

  private Command defaultDrive = drivetrain.applyRequest(() -> drive
      .withVelocityX(setX.getAsDouble() * MaxSpeed)
      .withVelocityY(setY.getAsDouble() * MaxSpeed)
      .withRotationalRate(-rotStick.getAsDouble() * MaxAngularRate));

  private Command trenchDrive = drivetrain.applyRequest(() -> trenchDriveRequest
      .withVelocityX(setX.getAsDouble() * MaxSpeed)
      .withVelocityY(calcY.getAsDouble())
      .withTargetDirection(closestRot));

  private PIDController trenchPIDY = new PIDController(12, 0, 0);

  public s_Drivetrain() {
    initialized = true;

    final var idle = new SwerveRequest.Idle();

    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    drivetrain.registerTelemetry(logger::telemeterize);

    drivetrain.setDefaultCommand(defaultDrive);
  }

  public CommandSwerveDrivetrain getDrivetrain(){
    return drivetrain;
  }

  public void bindControllers(s_QuestNav QNav, CommandXboxController driver){
    driver.start().onTrue(drivetrain.runOnce(()->drivetrain.seedFieldCentric()).ignoringDisable(true));
    driver.start().onTrue(Commands.runOnce(()->QNav.setPose(new Pose3d(new Pose2d(drivetrain.getState().Pose.getTranslation(), Rotation2d.fromDegrees(180))) )).ignoringDisable(true));

    xStick = () -> driver.getLeftX();
    yStick = () -> driver.getLeftY();

    driver.b().whileTrue(drivetrain.applyRequest(()->brake));

    if(RobotBase.isReal()){
      rotStick = () -> driver.getRightX();
    }else{
      // rotStick = () -> controller.getRightX();
      rotStick = () -> driver.getRawAxis(2);
    }
    setController();
  }

  public void setController() {
    this.calcY = () -> 0;

    SpeedMod.set(setSpeedModifier.getAsDouble());
    setY = () -> Math.copySign((xStick.getAsDouble() * xStick.getAsDouble()), (-xStick.getAsDouble())) * setSpeedModifier.getAsDouble();
    setX = () -> Math.copySign((yStick.getAsDouble() * yStick.getAsDouble()), (-yStick.getAsDouble())) * setSpeedModifier.getAsDouble();
    // setRot = () -> -rotStick.getAsDouble() * MaxAngularRate  * setSpeedModifier.getAsDouble();
    
    trenchDrive.cancel();
    drivetrain.removeDefaultCommand();
    drivetrain.setDefaultCommand(defaultDrive);
  }

  public void setTrenchLock() {
    this.setY = () -> 0;

    defaultDrive.cancel();
    drivetrain.removeDefaultCommand();

    drivetrain.setDefaultCommand(trenchDrive);
  }
  
  DoubleSupplier setSpeedModifier = ()->1;

  public void setSpeedModifier(double modifier){
    setSpeedModifier = ()-> modifier;
  }

  public void setValuesTrench() {
    setX = () -> Math.copySign((yStick.getAsDouble() * yStick.getAsDouble()), (-yStick.getAsDouble()));
    // setRot = () -> -rotStick.getAsDouble() * MaxAngularRate;
    if (robotPose.get().getRotation().getDegrees() > -90 && robotPose.get().getRotation().getDegrees() < 90) {
      closestRot = Rotation2d.fromDegrees(180);

    } else {
      closestRot = Rotation2d.fromDegrees(0);
    }

    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        calcY = () -> -trenchPIDY.calculate(robotPose.get().getY(), getTrenchY().in(Meters));

      } else {
        calcY = () -> trenchPIDY.calculate(robotPose.get().getY(), getTrenchY().in(Meters));

      }
    } else {
      calcY = () -> trenchPIDY.calculate(robotPose.get().getY(), getTrenchY().in(Meters));

    }

    PIDY.set(calcY.getAsDouble());
    TrenchY.set(getTrenchY().in(Meters));
    RobotY.set(robotPose.get().getY());
  }

  Pose2d robotPosition;

  private Distance getTrenchY() {
    robotPosition = this.robotPose.get();
    if (robotPosition.getMeasureY().gte(FieldConstants.FIELD_WIDTH.div(2))) {
      return FieldConstants.FIELD_WIDTH.minus(FieldConstants.TRENCH_CENTER);
    }
    return FieldConstants.TRENCH_CENTER;
  }

  public void idleOut() {
    drivetrain.setDefaultCommand(idleDrive);
  }

  public boolean initialized = false;

  public boolean checkSubsystem() {
    return getInitialized();
  }

  public boolean getInitialized() {
    return initialized;
  }

  public void stop() {

  }

  boolean doRejectUpdate;

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
     LimelightHelpers.SetRobotOrientation("limelight-rsl", drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    // SmartDashboard.putNumber("Raw Heading", drivetrain.getState().Pose.getRotation().getDegrees());
  }
}
