package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.traits.CommonTalon;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class CoralIndexerSubsystem implements Subsystem{
    CommonTalon tets = new TalonFX(3);
}
