// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.IO.TurretIO;
import frc.robot.subsystems.sim.PhysicsSim;

public class s_Hood extends SubsystemBase {
  // Turret Sim
  TurretIO turretSimulation;

  // Hood Constants
  private final int motorId = 5;
  private final double ratio = (30.0 / 16.0) * (40.0 / 20.0) * (34.0 / 16.0) * (210.0 / 40.0); // * 2.5

  // Talon Setup
  private CANBus turretBus = new CANBus("turret");
  private TalonFXS hoodTalon = new TalonFXS(motorId, turretBus);

  // Motion Magic Setup
  TalonFXSConfiguration pivotConfigs = new TalonFXSConfiguration();
  Slot0Configs pivotSlot0Configs = pivotConfigs.Slot0;
  MotionMagicConfigs pivotMotionMagicConfigs = pivotConfigs.MotionMagic;

  double moi = 0.0433934616;

  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
  final VoltageOut m_volts = new VoltageOut(0);

  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(0.3).per(Second), // Ramp rate
          Volts.of(3), // Max step voltage
          Seconds.of(10), // Timeout
          (state) -> SignalLogger.writeString("sysid-test-state", state.toString())),
      new SysIdRoutine.Mechanism(
          (voltage) -> hoodTalon.setControl(m_volts.withOutput(voltage.in(Volts))),
          null, // Phoenix 6 handles logging automatically via SignalLogger
          this));

  public s_Hood(TurretIO turretSim) {
    this.turretSimulation = turretSim;

    pivotSlot0Configs.kS = 0.34915;
    pivotSlot0Configs.kV = 3.284; // Unnessasary
    pivotSlot0Configs.kA = 0.074474;
    pivotSlot0Configs.kG = 0;

    pivotSlot0Configs.kP = 62.38;
    pivotSlot0Configs.kI = 0;
    pivotSlot0Configs.kD = 4.9581;

    var motionMagicConfigs = pivotConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 0.4;
    motionMagicConfigs.MotionMagicAcceleration = 0.8;

    pivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    pivotConfigs.ExternalFeedback.SensorToMechanismRatio = ratio;

    pivotConfigs.CurrentLimits.StatorCurrentLimit = 50;
    pivotConfigs.CurrentLimits.SupplyCurrentLimit = 25;

    pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Rotations.convertFrom(40, Degree);
    pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Rotations.convertFrom(0, Degree);
    
    pivotConfigs.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    hoodTalon.getConfigurator().apply(pivotConfigs);

    PhysicsSim.getInstance().addTalonFXS(hoodTalon, moi, ratio);

  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public Command getSysIdRoutine() {
    Command hoodSysidRoutine = Commands.sequence(
        sysIdDynamic(Direction.kForward),
        sysIdDynamic(Direction.kReverse),
        sysIdQuasistatic(Direction.kForward),
        sysIdQuasistatic(Direction.kReverse));
    return hoodSysidRoutine;
  }

  public void setVoltage(double volts) {
    hoodTalon.setVoltage(volts);
  }

  public void setDegrees(double degress) {
    SmartDashboard.putNumber("Turret/Hood Angle Setpoint", degress);
    hoodTalon.setControl(m_request.withPosition(Rotations.convertFrom(degress, Degrees)));
  }

  public Angle getAngle() {
    return hoodTalon.getPosition().getValue();
  }

  public Command getStallHome() {
    return Commands.run(() -> {
      hoodTalon.setVoltage(-1);
    }, this)
        .until(() -> hoodTalon.getStatorCurrent().getValue().gte(Amps.of(15)))
        .andThen(Commands.runOnce(
            () -> {
              hoodTalon.setVoltage(0);
              hoodTalon.setPosition(Degree.of(0));
            }));
  }

  @Override
  public void periodic() {
    turretSimulation.setHoodDegrees(this.getAngle().in(Degree));
    SmartDashboard.putNumber("Turret/Hood Angle", this.getAngle().in(Degree));
  }
}
