// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.util.MotorJamDetector;

public class s_Serializer extends SubsystemBase {
  /** Creates a new s_Spindex. */
  private SparkFlex SpindexFlexLeft = new SparkFlex(40, MotorType.kBrushless); // black wheel
  private DigitalInput beamBreakLeft = new DigitalInput(9);

  private SparkFlex SpindexFlexRight = new SparkFlex(41, MotorType.kBrushless); // blue wheel
  private DigitalInput beamBreakRight = new DigitalInput(8);

  private SparkClosedLoopController m_ControllerLeft = SpindexFlexLeft.getClosedLoopController();
  private SparkClosedLoopController m_ControllerRight = SpindexFlexRight.getClosedLoopController();

  private SparkFlexConfig config = new SparkFlexConfig();
  private SparkFlexConfig config2 = new SparkFlexConfig();

  MotorJamDetector detectorLeft;
  MotorJamDetector detectorRight;


  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(0.3).per(Second), // Ramp rate
          Volts.of(3), // Max step voltage
          Seconds.of(10), // Timeout
          (state) -> SignalLogger.writeString("sysid-test-state", state.toString())),
      new SysIdRoutine.Mechanism(
          (voltage) -> SpindexFlexLeft.setVoltage(voltage),
          null, // Need to use signallogger, regular crashes sysid
          this));

  // private RelativeEncoder leftEncoder = SpindexFlexLeft.getEncoder();
  // private RelativeEncoder rightEncoder = SpindexFlexRight.getEncoder();

  public s_Serializer() {
    config.closedLoop.p(0.0030574).i(0).d(0.0);
    config2.closedLoop.p(0.0030574).i(0).d(0.0);
    config.closedLoop.feedForward.kV(0.10517);
    config2.closedLoop.feedForward.kV(0.10517);

    config.closedLoop.feedForward.kA(0.0071234);
    config2.closedLoop.feedForward.kA(0.0071234);

    config.closedLoop.feedForward.kS(0.067918);
    config2.closedLoop.feedForward.kS(0.067918);

    config.smartCurrentLimit(70);
    config2.smartCurrentLimit(70);

    SpindexFlexLeft.configure(config, com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
    SpindexFlexRight.configure(config2, com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);

    this.detectorLeft = new MotorJamDetector(20, 200, 0.5);
    this.detectorRight= new MotorJamDetector(20, 200, 0.5);

  }

  public void setVoltage(double volts) {
    SpindexFlexLeft.setVoltage(volts);
    SpindexFlexRight.setVoltage(volts);

  }

  public void setDiffVoltage(double volts) {
    SpindexFlexLeft.setVoltage(-volts);
    SpindexFlexRight.setVoltage(volts);

  }

  final double primaryVoltage = 6;
  final double secondaryVoltage = 3;

  final double primarySetpoint = 4000;
  final double secondarySetpoint = 500;
  double rounded;

  // public double timeout = 100;

  // public void setFromBeamBreaks() {
  //   if (!beamBreakLeft.get() && !beamBreakRight.get()) {
  //     m_ControllerLeft.setSetpoint(primarySetpoint, ControlType.kVelocity);
  //     m_ControllerRight.setSetpoint(secondarySetpoint, ControlType.kVelocity);

  //   } else if (!beamBreakLeft.get() && beamBreakRight.get()) {

  //     m_ControllerLeft.setSetpoint(-primarySetpoint, ControlType.kVelocity);
  //     m_ControllerRight.setSetpoint(-secondarySetpoint, ControlType.kVelocity);

  //   } else if (!beamBreakRight.get() && beamBreakLeft.get()) {
  //     m_ControllerLeft.setSetpoint(secondarySetpoint, ControlType.kVelocity);
  //     m_ControllerRight.setSetpoint(primarySetpoint, ControlType.kVelocity);

  //   } else {
  //     m_ControllerLeft.setSetpoint(-primarySetpoint, ControlType.kVelocity);
  //     m_ControllerRight.setSetpoint(primarySetpoint, ControlType.kVelocity);
  //     // SpindexFlexLeft.setVoltage(-primaryVoltage);
  //     // SpindexFlexRight.setVoltage(primaryVoltage);
  //     // rounded = Math.round(Timer.getTimestamp() * 2) / 2.0;
  //     // if ((rounded % 1) == 0) {
  //     // SpindexFlexLeft.setVoltage(primaryVoltage);
  //     // SpindexFlexRight.setVoltage(secondaryVoltage);
  //     // } else {
  //     // SpindexFlexLeft.setVoltage(-secondaryVoltage);
  //     // SpindexFlexRight.setVoltage(-primaryVoltage);
  //     // }
  //   }
  // }

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

  String lastSide = "left";

  public void setFromBeamBreaks() {

    if (!beamBreakLeft.get() && !beamBreakRight.get()) {
      
      if(lastSide.equals("left")){
        SpindexFlexLeft.setVoltage(-primaryVoltage);
        SpindexFlexRight.setVoltage(-secondaryVoltage);
      } else{
        SpindexFlexLeft.setVoltage(secondaryVoltage);
        SpindexFlexRight.setVoltage(primaryVoltage);
      }
     

    } else if (!beamBreakLeft.get() && beamBreakRight.get()) {
      lastSide = "left";
      SpindexFlexLeft.setVoltage(-primaryVoltage);
      SpindexFlexRight.setVoltage(-secondaryVoltage);

    } else if (!beamBreakRight.get() && beamBreakLeft.get()) {
      lastSide = "right";
      SpindexFlexLeft.setVoltage(secondaryVoltage);
      SpindexFlexRight.setVoltage(primaryVoltage);

    } else {
      SpindexFlexLeft.setVoltage(-primaryVoltage);
      SpindexFlexRight.setVoltage(primaryVoltage);

      // SpindexFlexLeft.setVoltage(-primaryVoltage);
      // SpindexFlexRight.setVoltage(primaryVoltage);
      // rounded = Math.round(Timer.getTimestamp() * 2) / 2.0;
    }
  }

  public boolean isJammed(){
    return detectorLeft.update(SpindexFlexLeft.getOutputCurrent(), SpindexFlexLeft.getEncoder().getVelocity()) || detectorRight.update(SpindexFlexRight.getOutputCurrent(), SpindexFlexRight.getEncoder().getVelocity());
  }

  public void stop() {
    SpindexFlexLeft.stopMotor();
    SpindexFlexRight.stopMotor();

  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Beambreaks/R", beamBreakRight.get());
    SmartDashboard.putBoolean("Beambreaks/L", beamBreakLeft.get());

    SmartDashboard.putNumber("Serializer/LVelocity", SpindexFlexLeft.getEncoder().getVelocity());
    SmartDashboard.putNumber("Serializer/RVelocity", SpindexFlexRight.getEncoder().getVelocity());

    // This method will be called once per scheduler run
  }
}
