// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IO.TurretIO;

public class s_Hood extends SubsystemBase {
  // Turret Sim
  TurretIO turretSimulation;

  // Hood Constants
  private final int motorId = 5;
  private final double ratio = (30 / 16) * (40 / 20) * (34 / 16) * (210 / 40); // * 2.5

  // Talon Setup
  private CANBus turretBus = new CANBus("turret");
  private TalonFXS hoodTalon = new TalonFXS(motorId, turretBus);

  // Motion Magic Setup
  TalonFXSConfiguration pivotConfigs = new TalonFXSConfiguration();
  Slot0Configs pivotSlot0Configs = pivotConfigs.Slot0;
  MotionMagicConfigs pivotMotionMagicConfigs = pivotConfigs.MotionMagic;

  final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);

  public s_Hood(TurretIO turretSim) {
    this.turretSimulation = turretSim;

    pivotSlot0Configs.kS = 0;
    pivotSlot0Configs.kV = 0; // Unnessasary
    pivotSlot0Configs.kA = 0;
    pivotSlot0Configs.kG = 0;

    pivotSlot0Configs.kP = 175;
    pivotSlot0Configs.kI = 0;
    pivotSlot0Configs.kD = 0;

    var motionMagicConfigs = pivotConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 0;
    motionMagicConfigs.MotionMagicExpo_kV = 0;
    motionMagicConfigs.MotionMagicExpo_kA = 0;

    pivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    pivotConfigs.ExternalFeedback.SensorToMechanismRatio = ratio;

    pivotConfigs.CurrentLimits.StatorCurrentLimit = 50;
    pivotConfigs.CurrentLimits.SupplyCurrentLimit = 25;

    pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Rotations.convertFrom(40, Degree);

    pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    hoodTalon.getConfigurator().apply(pivotConfigs);

  }

  public void setDegrees(double degress){
    hoodTalon.setControl(m_request.withEnableFOC(true));
  }

  public Angle getAngle(){
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

  }
}
