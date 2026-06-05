// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Serialization;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.s_Belt;
import frc.robot.subsystems.s_Serializer;
import frc.robot.util.MotorJamDetector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SerializeUnjam extends Command {
  /** Creates a new Belt. */
  s_Belt s_Belt;
  s_Serializer s_Serializer;
  MotorJamDetector detectorBelt;


  boolean currentJamValue = false;

  public SerializeUnjam(s_Belt s_Belt, s_Serializer s_Serializer) {
    this.s_Belt = s_Belt;
    this.s_Serializer = s_Serializer;

    this.detectorBelt = new MotorJamDetector(20, 200, 0.5);
    
    addRequirements(s_Belt, s_Serializer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentJamValue = detectorBelt.update(s_Belt.getAmps(), s_Belt.getVelocity()) || s_Serializer.isJammed();
    
    System.out.println(currentJamValue);
    if(currentJamValue == true){
        s_Belt.setIndexRpm(-4041);
        s_Serializer.setDiffVoltage(-12);
        
    } else{
        s_Belt.setIndexRpm(4041);
        s_Serializer.setFromBeamBreaks();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Belt.setIndexRpm(0);
        s_Serializer.setDiffVoltage(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
