// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.sim.PhysicsSim;

public class s_Intake extends SubsystemBase {
  /** Creates a new s_Intake. */
  public static s_Intake m_Instance;

  private double ratio = 20;
  private double moi = 0.470247613;

  private SparkFlex intakeRollerBlack = new SparkFlex(32, MotorType.kBrushless);
  private SparkFlex intakeRollerBlue = new SparkFlex(33, MotorType.kBrushless);

  TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
  Slot0Configs slot0Configs = pivotConfigs.Slot0;

  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
  final VoltageOut m_VoltageOut = new VoltageOut(0);

  private TalonFX lPivotTalonFX = new TalonFX(30);
  private TalonFX rPivotTalonFX = new TalonFX(31);

  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(0.2).per(Second), // Ramp rate
          Volts.of(1), // Max step voltage
          Seconds.of(10), // Timeout
          (state) -> SignalLogger.writeString("sysid-test-state", state.toString())),
      new SysIdRoutine.Mechanism(
          (voltage) -> rPivotTalonFX.setVoltage(voltage.in(Volts)),
          null, // Phoenix 6 handles logging automatically via SignalLogger
          this));

  public s_Intake() {
    initialized = true;

    SparkBaseConfig config = new SparkFlexConfig().openLoopRampRate(0.4).smartCurrentLimit(40);

    intakeRollerBlack.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    intakeRollerBlue.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    slot0Configs.kS = 0.44773;
    slot0Configs.kV = .90174;
    slot0Configs.kA = 0.82725;
    slot0Configs.kG = 0.55342;

    slot0Configs.kP = 17.447;
    slot0Configs.kI = 0;
    slot0Configs.kD = 5.0287;

    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

    var motionMagicConfigs = pivotConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 1;
    motionMagicConfigs.MotionMagicAcceleration = 2;

    pivotConfigs.Feedback.SensorToMechanismRatio = ratio;

    pivotConfigs.CurrentLimits.StatorCurrentLimit = 60;
    pivotConfigs.CurrentLimits.SupplyCurrentLimit = 40;

    pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Rotations.convertFrom(140, Degrees);
    // pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
    // Rotations.convertFrom(0, Degrees);

    pivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    lPivotTalonFX.setControl(new Follower(31, MotorAlignmentValue.Opposed));


    PhysicsSim.getInstance().addTalonFX(rPivotTalonFX, moi, ratio, 2);

    rPivotTalonFX.getConfigurator().apply(pivotConfigs);
    lPivotTalonFX.getConfigurator().apply(pivotConfigs);

    rPivotTalonFX.setPosition(Degrees.of(140));
    lPivotTalonFX.setPosition(Degrees.of(140));

  }

  // private double currentDeg = 0.0;

  public static s_Intake getInstance() {
    if (m_Instance == null) {
      m_Instance = new s_Intake();
    }
    return m_Instance;
  }

  public void setDegrees(double deg) {
    SmartDashboard.putNumber("Intake/Intake Angle Setpoint", deg);
    rPivotTalonFX.setControl(m_request.withPosition(Degrees.of(deg)));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public Command getSysIdRoutine() {
    Command intakeSysidRoutine = Commands.sequence(
        sysIdDynamic(Direction.kForward),
        sysIdDynamic(Direction.kReverse),
        sysIdQuasistatic(Direction.kForward),
        sysIdQuasistatic(Direction.kReverse));
    return intakeSysidRoutine;
  }

  public void overrideDeg(double deg) {
    rPivotTalonFX.setPosition(Degrees.of(deg));
    lPivotTalonFX.setPosition(Degrees.of(deg));
  }

  public void setSpeed(double dutyCycle) {
    intakeRollerBlack.set(-dutyCycle);
    intakeRollerBlue.set(dutyCycle);
  }

  public void setLeftVoltage(double volts) {
    lPivotTalonFX.setControl(m_VoltageOut.withOutput(-volts));
  }

  public void setRightVoltage(double volts) {
    rPivotTalonFX.setControl(m_VoltageOut.withOutput(volts));
  }

  public Angle getLeftAngle() {
    return lPivotTalonFX.getPosition().getValue();
  }

  public Angle getRightAngle() {
    return rPivotTalonFX.getPosition().getValue();
  }

  public TalonFX getLeftPivotTalonFX() {
    return lPivotTalonFX;
  }

  public TalonFX getRightPivotTalonFx() {
    return rPivotTalonFX;
  }

  public boolean initialized = false;

  public boolean checkSubsystem() {
    return getInitialized();
  }

  public boolean getInitialized() {
    return initialized;
  }

  public void stop() {
    intakeRollerBlack.stopMotor();
    intakeRollerBlue.stopMotor();

    lPivotTalonFX.stopMotor();
    rPivotTalonFX.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Intake Angle", rPivotTalonFX.getPosition().getValue().in(Degrees));
  }

  @Override
  public void simulationPeriodic() {
  }
}
