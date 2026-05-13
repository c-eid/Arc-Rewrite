// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.IO.TurretIO;
import frc.robot.subsystems.sim.PhysicsSim;

public class s_Turret extends SubsystemBase {

  // Cosntants
  private final int motorId = 3;
  private final double ratio = (144.0 / 15.0) * 5.0;
  private final double moi = 0.106165886;

  // MotionMagicExpoSetup
  private final TalonFX turretMotor = new TalonFX(motorId);

  private TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
  private Slot0Configs pivotSlot0Configs = pivotConfigs.Slot0;
  private MotionMagicConfigs pivotMotionMagicConfigs = pivotConfigs.MotionMagic;

  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
  final VoltageOut m_volts = new VoltageOut(0);

  // Turret Specific Variables
  public Supplier<Boolean> inaccurate = () -> false;
  private DoubleSupplier offset = () -> 0;

  // Telemetry
  private TurretIO turretSimulation;

  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(0.5).per(Second), // Ramp rate
          Volts.of(3), // Max step voltage
          Seconds.of(4), // Timeout
          (state) -> SignalLogger.writeString("sysid-test-state", state.toString())),
      new SysIdRoutine.Mechanism(
          (voltage) -> turretMotor.setControl(m_volts.withOutput(voltage)),
          null, // Phoenix 6 handles logging automatically via SignalLogger
          this));

  public s_Turret(TurretIO turretSim) {
    this.turretSimulation = turretSim;

    pivotSlot0Configs.kS = 0.14316;
    pivotSlot0Configs.kV =6.417;
    pivotSlot0Configs.kA = 0.30931;
    pivotSlot0Configs.kG = 0;

    pivotSlot0Configs.kP =79.706; // 10;
    pivotSlot0Configs.kI = 0;
    pivotSlot0Configs.kD =7.3613;// 0.6;

    pivotMotionMagicConfigs.MotionMagicCruiseVelocity = 2;
    pivotMotionMagicConfigs.MotionMagicAcceleration = 4;
    pivotMotionMagicConfigs.MotionMagicJerk = 30;


    pivotConfigs.Feedback.SensorToMechanismRatio = ratio;

    pivotConfigs.CurrentLimits.StatorCurrentLimit = 70;
    pivotConfigs.CurrentLimits.SupplyCurrentLimit = 60;

    pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Rotations.convertFrom(360, Degree);
    pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Rotations.convertFrom(-360, Degree);

    pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    PhysicsSim.getInstance().addTalonFX(turretMotor, moi, ratio);

    turretMotor.getConfigurator().apply(pivotConfigs);

  }

  private Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  private Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public Command getSysIdRoutine() {
    Command turretSysidRoutine = Commands.sequence(
        sysIdDynamic(Direction.kForward),
        sysIdDynamic(Direction.kReverse),
        sysIdQuasistatic(Direction.kForward),
        sysIdQuasistatic(Direction.kReverse));
    return turretSysidRoutine;
  }

  public void setVoltage(double volts) {
    turretMotor.setVoltage(volts);
  }

  public void setDegrees(double degrees) {
    // Add Offset%
    degrees += offset.getAsDouble();

    // Actual Motor position
    double motorPosition = turretMotor.getPosition().getValue().in(Degree);

    // Check innacuracy by seeing if a 10 degree tolerance
    if (Math.abs(degrees - motorPosition) <= 10) {
      inaccurate = () -> false;
    } else {
      inaccurate = () -> true;
    }

    SmartDashboard.putNumber("Turret/Turret Angle Setpoint", degrees);

    // There was a check to see if degree

    // Check existed for sim to show actual degrees but I might just make a seperate
    // telemetry method
    // Showed velocity as well might be useful

    turretMotor.setControl(m_request.withPosition(Rotations.convertFrom(degrees, Degree)));
  }

  public void setOffset(double offset) {
    // Telemetry for offset

    this.offset = () -> offset;
  }

  public void overridePosition(double degrees) {
    turretMotor.setPosition(Degree.of(degrees));
  }

  public Angle getAngle() {
    return turretMotor.getPosition().getValue();
  }

  @Override
  public void periodic() {
    turretSimulation.setTurretDegrees(this.getAngle().in(Degree));
    SmartDashboard.putNumber("Turret/Turret Angle", this.getAngle().in(Degree));
    SmartDashboard.putNumber("Turret/Velocity", turretMotor.getVelocity().getValue().in(RotationsPerSecond));

  }

  @Override
  public void simulationPeriodic() {

  }
}
