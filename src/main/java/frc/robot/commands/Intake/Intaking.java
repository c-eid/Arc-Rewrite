// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.s_Intake;

public class Intaking extends Command {
  s_Intake intake;
  public Intaking(s_Intake intake) {
      this.intake = intake;
      addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setDegrees(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(intake.getRightAngle().in(Degree));

    if(intake.getRightAngle().in(Degree) < 30){
      intake.setSpeed(1);
    } else{
      intake.setSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
