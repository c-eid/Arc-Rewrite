// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import org.ejml.sparse.csc.linsol.qr.LinearSolverQrLeftLooking_DSCC;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.s_Intake;
import frc.robot.util.MotorJamDetector;

public class HomeIntake extends Command {
  s_Intake intake;
  boolean leftHomed = false;
  boolean rightHomed = false;
  Supplier<TalonFX> right;
  Supplier<TalonFX> left;

  MotorJamDetector leftJamDetector;
  MotorJamDetector rightJamDetector;


  public HomeIntake(s_Intake intake) {
      this.intake = intake;
      addRequirements(intake);

      right = () -> intake.getRightPivotTalonFx();
      left = () -> intake.getLeftPivotTalonFX();

      leftJamDetector = new MotorJamDetector(20, 50, 0.15);
      rightJamDetector = new MotorJamDetector(20, 50, 0.15);


    }

  @Override
  public void initialize() {
    leftHomed = false;
    rightHomed = false; 

    intake.setLeftVoltage(-2);
    intake.setRightVoltage(-2);

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(rightJamDetector.update(right.get().getStatorCurrent().getValueAsDouble(), right.get().getVelocity().getValue().in(RPM))){
      rightHomed = true;
    }

    if(leftJamDetector.update(left.get().getStatorCurrent().getValueAsDouble(), left.get().getVelocity().getValue().in(RPM))){
      leftHomed = true;
    }

    System.out.println(rightHomed + " , " + leftHomed);
    System.out.println(right.get().getStatorCurrent().getValue().in(Amps) + ", " + left.get().getStatorCurrent().getValue().in(Amps));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setSpeed(0);
    intake.overrideDeg(3);

    left.get().setControl(new Follower(31, MotorAlignmentValue.Opposed));

    intake.setDegrees(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return leftHomed && rightHomed;
  }
}
