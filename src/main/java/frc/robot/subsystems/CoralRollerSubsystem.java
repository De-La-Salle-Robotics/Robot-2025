package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.utils.Trigger;
import frc.robot.Constants;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.SingleFadeAnimation;

public class CoralRollerSubsystem implements Subsystem{
    private double SuckOutput = -0.8;
    private double SpitOutput = 1;
    private double StopOutput = 0;
    private double EndEffectorRollersStopVelocity = 0.1;
    private final Color8Bit InPossition = new Color8Bit(Color.kGreen);
    private final Color8Bit NotInPossition = new Color8Bit(Color.kRed);
    private final Color8Bit PreMatch = new Color8Bit(Color.kYellow);
    private final Color8Bit InMatch = new Color8Bit(Color.kPurple);
    CANrange reefSensor = new CANrange(19);

    TalonFX coralEndEffectorRollers = new TalonFX(Constants.CoralEndEffectorConstants.RollerId, Constants.CANivoreName);
    CANdle leds = new CANdle(0, "*");
    Color8Bit color = new Color8Bit(Color.kYellow);
    Animation anim = new com.ctre.phoenix.led.SingleFadeAnimation(color.red, color.green, color.blue);

    DCMotorSim rollerSim;

    DutyCycleOut manualControlRequest = new DutyCycleOut(0);

    public CoralRollerSubsystem() {
        if(RobotBase.isSimulation()) {
            DCMotor rollerMotors = DCMotor.getKrakenX60(1);
            rollerSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(rollerMotors, 0.1, 1), rollerMotors);
        }
        anim.setSpeed(0.2);
    }

    @Override
    public void periodic() {
        leds.animate(anim);
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState rollerSimState = coralEndEffectorRollers.getSimState();

        rollerSim.setInputVoltage(rollerSimState.getMotorVoltage());

        rollerSim.update(0.02);

        rollerSimState.setRawRotorPosition(rollerSim.getAngularPosition());
        rollerSimState.setRotorVelocity(rollerSim.getAngularVelocity());
    }

    private void manualDriveCoralEndEffectorRollers(double output) {
        coralEndEffectorRollers.setControl(manualControlRequest.withOutput(output));
    }

    public Command manualCoralEndEffectorCommand(DoubleSupplier CoralEndEffectorInput) {
        return run(()-> {
            manualDriveCoralEndEffectorRollers(CoralEndEffectorInput.getAsDouble());
        });
    }

    public Command suckCommand() {
        return run(()->{
            manualDriveCoralEndEffectorRollers(SuckOutput);
        });
    }

    public Command spitCommand() {
        return run(()->{
            manualDriveCoralEndEffectorRollers(SpitOutput);
        });
    }

    public Command suckUntilHaveCoralCommand() {
        return run(()->manualDriveCoralEndEffectorRollers(SuckOutput)).until(new Trigger(()->{
            return coralEndEffectorRollers.getVelocity().getValueAsDouble()<EndEffectorRollersStopVelocity;
        }).debounce(1, DebounceType.kRising).asRisingEdge()).andThen(
        run(()->manualDriveCoralEndEffectorRollers(StopOutput))); 
    }
}
