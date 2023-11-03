package org.frcteam2910.c2023.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2023.subsystems.arm.ArmSubsystem;
import org.frcteam2910.c2023.util.constants.ArmPoseConstants;

public class StowArmCommand extends CommandBase {
    ArmSubsystem arm;

    public StowArmCommand(ArmSubsystem armSubsystem) {
        arm = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        arm.setTargetPose(ArmPoseConstants.STOW);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
