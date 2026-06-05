// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.s_Intake;

public class Bounce extends Command {
  s_Intake intake;
  Timer time;
  
  public Bounce(s_Intake intake) {
      this.intake = intake;
      time = new Timer();
      addRequirements(intake);
  }

  @Override
  public void initialize() {
    time.restart();
    intake.setDegrees(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(time.get() > 1.4){
      intake.setDegrees(90);
    }
    if(time.get() > 2.9){
      intake.setDegrees(120);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setDegrees(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
