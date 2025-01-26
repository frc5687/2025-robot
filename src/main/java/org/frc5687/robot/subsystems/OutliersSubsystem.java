package org.frc5687.robot.subsystems;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc5687.robot.util.BaseInputs;
import org.frc5687.robot.util.BaseOutputs;

public abstract class OutliersSubsystem<Inputs extends BaseInputs, Outputs extends BaseOutputs>
        extends SubsystemBase {
    protected final SubsystemIO<Inputs, Outputs> _io;
    protected final Inputs _inputs;
    protected final Outputs _outputs;

    private final Object inputLogger;
    private final Object outputLogger;
    private boolean _seperateControl = false;

    public OutliersSubsystem(SubsystemIO<Inputs, Outputs> io, Inputs inputs, Outputs outputs) {
        _io = io;
        _inputs = inputs;
        _outputs = outputs;

        String inputLoggerName = _inputs.getClass().getSimpleName() + "Logger";
        String outputLoggerName = _outputs.getClass().getSimpleName() + "Logger";
        // Hack in fucntinallity such that we can still use Epilogue for logging
        try {
            inputLogger =
                    Class.forName("edu.wpi.first.epilogue.Epilogue")
                            .getField(firstCharToLowerCase(inputLoggerName))
                            .get(null);

            outputLogger =
                    Class.forName("edu.wpi.first.epilogue.Epilogue")
                            .getField(firstCharToLowerCase(outputLoggerName))
                            .get(null);
        } catch (Exception e) {
            throw new RuntimeException("Failed to get loggers", e);
        }
        System.out.println(inputLoggerName);
    }

    protected abstract void processInputs();

    protected abstract void periodic(Inputs inputs, Outputs outputs);

    protected void setToSeperateControl(boolean seperateControl) {
        System.out.println("Set to seperate control: " + seperateControl);
        _seperateControl = seperateControl;
    }

    protected boolean isSeperateControl() {
        return _seperateControl;
    }

    protected void process() {
        _io.updateInputs(_inputs);
        // reflection to call update() on the logger, We are hacking in functionallity due to Epilgue
        // stuggling to find necessary IO classes
        try {
            inputLogger
                    .getClass()
                    .getMethod("update", EpilogueBackend.class, _inputs.getClass())
                    .invoke(
                            inputLogger, Epilogue.getConfig().backend.getNested(_inputs.getLogPath()), _inputs);
        } catch (Exception e) {
            e.printStackTrace();
        }

        processInputs();
        periodic(_inputs, _outputs);

        try {
            outputLogger
                    .getClass()
                    .getMethod("update", EpilogueBackend.class, _outputs.getClass())
                    .invoke(
                            outputLogger,
                            Epilogue.getConfig().backend.getNested(_outputs.getLogPath()),
                            _outputs);
        } catch (Exception e) {
            e.printStackTrace();
        }

        _io.writeOutputs(_outputs);
    }

    @Override
    public final void periodic() {
        if (!_seperateControl) {
            process();
        }
    }

    private String firstCharToLowerCase(String str) {
        return str.substring(0, 1).toLowerCase() + str.substring(1);
    }
}
