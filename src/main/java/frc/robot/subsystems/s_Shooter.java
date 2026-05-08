// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.sim.PhysicsSim;

public class s_Shooter extends SubsystemBase {
  /** Creates a new s_Shooter. */

  double ratio = 38.0 / 40.0;

  // TODO: Telemetry
  private CANBus turretBus = new CANBus("turret");

  private TalonFX leftTalonFlywheel = new TalonFX(7, turretBus);
  private TalonFX rightTalonFlywheel = new TalonFX(6, turretBus);

  final VelocityVoltage m_request = new VelocityVoltage(0);

  public double offset = 0;

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(1).per(Second), // Ramp rate
          Volts.of(8), // Max step voltage
          Seconds.of(15), // Timeout
          (state) -> SignalLogger.writeString("sysid-test-state", state.toString())),
      new SysIdRoutine.Mechanism(
          (voltage) -> leftTalonFlywheel.setVoltage(voltage.in(Volts)),
          null, // Phoenix 6 handles logging automatically via SignalLogger
          this));

  public s_Shooter() {
    var talonFXConfigs = new TalonFXConfiguration();
    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.34934; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.11527; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.010836; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.00015607; // A position error of 2.5 rotations results in 12 V output

    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 50;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    talonFXConfigs.Feedback.SensorToMechanismRatio = ratio;

    PhysicsSim.getInstance().addTalonFX(leftTalonFlywheel, 0.0012871344, ratio, 2);

    leftTalonFlywheel.getConfigurator().apply(talonFXConfigs);

    var supplyLimitConfig = new TalonFXConfiguration();

    supplyLimitConfig.CurrentLimits.SupplyCurrentLimit = 50;
    supplyLimitConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    rightTalonFlywheel.setControl(new Follower(7, MotorAlignmentValue.Opposed));
    rightTalonFlywheel.getConfigurator().apply(supplyLimitConfig);
  }

  double currentRpm = 0;

  public void setShooterVolts(double volts) {
    // shooter.setVoltage(Volts.of(volts));
    leftTalonFlywheel.setVoltage(volts);
  }

  public void setRPM(double rpm) {
    currentRpm = rpm + offset;
    SmartDashboard.putNumber("Shooter/Shooter Rpm Setpoint", rpm + offset);

    leftTalonFlywheel.setControl(m_request.withVelocity(RPM.of(rpm + offset)));
  }

  public void setRPM(Supplier<Double> rpm) {
    currentRpm = rpm.get() + offset;
    SmartDashboard.putNumber("Shooter/Shooter Rpm Setpoint", currentRpm);

    leftTalonFlywheel.setControl(m_request.withVelocity(RPM.of(currentRpm)));
  }

  public double getVelocity() {
    return leftTalonFlywheel.getVelocity().getValue().in(RPM) + offset;
  }

  public boolean atSetpoint() {
    return Math.abs(leftTalonFlywheel.getVelocity().getValue().in(RPM) + offset - currentRpm) < 200;

  }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public Command getSysIdRoutine() {
    Command shooterSysidRoutine = Commands.sequence(
        sysIdDynamic(Direction.kForward),
        sysIdDynamic(Direction.kReverse),
        sysIdQuasistatic(Direction.kForward),
        sysIdQuasistatic(Direction.kReverse));
    return shooterSysidRoutine;
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Shooter/At Setpoint", atSetpoint());
    SmartDashboard.putNumber("Shooter/Shooter Rpm", leftTalonFlywheel.getVelocity().getValue().in(RPM));

  }

  @Override
  public void simulationPeriodic() {
  }
}
