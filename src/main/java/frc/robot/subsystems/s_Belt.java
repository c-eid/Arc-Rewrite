// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class s_Belt extends SubsystemBase {
  /** Creates a new s_Shooter. */

  private TalonFX indexTalon = new TalonFX(50);
  final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

  final VoltageOut m_Voltage = new VoltageOut(0);

  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(1).per(Second), // Ramp rate
          Volts.of(8), // Max step voltage
          Seconds.of(15), // Timeout
          (state) -> SignalLogger.writeString("sysid-test-state", state.toString())),
      new SysIdRoutine.Mechanism(
          (voltage) -> indexTalon.setControl(m_Voltage.withOutput(voltage).withEnableFOC(true)),
          null, // Phoenix 6 handles logging automatically via SignalLogger
          this));

  public s_Belt() {
    // in init function
    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.47832; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12221; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.0035597; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.12198; // An error of 1 rps results in 0.11 V output

    // set Motion Magic Velocity settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)


    

    indexTalon.getConfigurator().apply(talonFXConfigs);
  }

  boolean jamming = false;
  double indexVelocity = 0;
  double startJamTimestamp = 0;

  public void setIndexRpm(double rpm) {
    indexTalon.setControl(m_request.withVelocity(RPM.of(rpm)).withEnableFOC(true));
  }

  public double getVelocity() {
    return indexTalon.getVelocity().getValue().in(RPM);
  }

  
  public double getAmps() {
    return indexTalon.getStatorCurrent().getValue().in(Amps);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret/Belt RPM", indexTalon.getVelocity().getValue().in(RPM));

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
  public void simulationPeriodic() {
  }
}
