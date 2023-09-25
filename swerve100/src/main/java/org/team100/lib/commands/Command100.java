// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.lib.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Add your docs here. */
public class Command100 extends Command{

    Command m_onFalseCommand;
    public Command100(Command onFalseCommand){
        m_onFalseCommand = onFalseCommand;
    }

    public void test(){
        System.out.println("ALLAL");
    }

    @Override
    public ConditionalCommand unless(BooleanSupplier condition) {
        return new ConditionalCommand(new InstantCommand(), m_onFalseCommand, condition);
      }

}
