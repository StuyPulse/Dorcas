package com.stuypulse.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.stuypulse.robot.Constants;
import com.stuypulse.robot.subsystems.Climber;

public class ClimberSlowMoveUpCommand extends ClimberMoveCommand {
    
    public ClimberSlowMoveUpCommand(Climber climber) {
        super(climber, Constants.ClimberSettings.CLIMBER_SLOW_SPEED, true);
    }

}
