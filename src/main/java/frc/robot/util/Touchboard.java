package frc.robot.util;

import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Touchboard {

    private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private static NetworkTable datatable = inst.getTable("touchboard");


    public Touchboard() {

    }

    // Action Button Methods
    public static Trigger bindActionButton(String topic, Command command) {

        datatable.getBooleanTopic(topic).publish();
        final BooleanSubscriber dataSubscriber = datatable.getBooleanTopic(topic).subscribe(false);

        return new Trigger(() -> dataSubscriber.get()).whileTrue(command);
    }

    public static Trigger bindActionButton(String topic, Supplier<Command> command) {

        datatable.getBooleanTopic(topic).publish();
        final BooleanSubscriber dataSubscriber = datatable.getBooleanTopic(topic).subscribe(false);

        return new Trigger(() -> dataSubscriber.get()).whileTrue(Commands.deferredProxy(command));
    }

    // Toggle Button Methods
    public static Trigger bindToggleButton(String topic, Command command) {

        datatable.getBooleanTopic(topic).publish();
        final BooleanSubscriber dataSubscriber = datatable.getBooleanTopic(topic).subscribe(false);

        return new Trigger(() -> dataSubscriber.get()).whileTrue(command);
    }

    public static Trigger bindToggleButton(String topic, Supplier<Command> command) {

        datatable.getBooleanTopic(topic).publish();
        final BooleanSubscriber dataSubscriber = datatable.getBooleanTopic(topic).subscribe(false);

        return new Trigger(() -> dataSubscriber.get()).whileTrue(Commands.deferredProxy(command));

    }

    // One Shot Button methods

    public static Trigger bindOneShotButton(String topic, Command command) {

        final BooleanPublisher dataPublisher = datatable.getBooleanTopic(topic).publish();
        final BooleanSubscriber dataSubscriber = datatable.getBooleanTopic(topic).subscribe(false);

        Trigger trigger = new Trigger(() -> dataSubscriber.get());

        trigger.onTrue(Commands.runOnce(() -> dataPublisher.set(false)).ignoringDisable(true));

        return trigger.onTrue(command);
    }

    public static Trigger bindOneShotButton(String topic, Supplier<Command> command) {

        final BooleanPublisher dataPublisher = datatable.getBooleanTopic(topic).publish();
        final BooleanSubscriber dataSubscriber = datatable.getBooleanTopic(topic).subscribe(false);

        Trigger trigger = new Trigger(() -> dataSubscriber.get());

        trigger.onTrue(Commands.runOnce(() -> dataPublisher.set(false)).ignoringDisable(true));

        return trigger.onTrue(selfCancelingCommand(topic, command));
    }

    
    private static HashMap<String, BooleanSubscriber> BooleanSubscriberMap = new HashMap<String, BooleanSubscriber>();

    // AXIS + NUMBER COMPONENT value getter
    public static boolean getBooleanValue(String topic) {
        if (BooleanSubscriberMap.containsKey(topic)) {
            // System.out.println(BooleanSubscriberMap.size());
            return BooleanSubscriberMap.get(topic).get();
        } else {
            final BooleanSubscriber dataSubscriber = datatable.getBooleanTopic(topic).subscribe(false);

            BooleanSubscriberMap.put(topic, dataSubscriber);

            return BooleanSubscriberMap.get(topic).get();
        }
    }

    // Axis Methods

    public static Trigger bindAxis(String topic, Supplier<Command> command) {
        datatable.getDoubleTopic(topic).publish();

        DoubleSubscriber dataSubscriber = datatable.getDoubleTopic(topic).subscribe(0,
                PubSubOption.pollStorage(1), PubSubOption.keepDuplicates(true));

        return new Trigger(() -> dataSubscriber.readQueueValues().length > 0).onTrue(Commands.deferredProxy(command));
    }

    // Number Component Methods

    public static Trigger bindNumberComponent(String topic, Supplier<Command> command) {
        datatable.getDoubleTopic(topic).publish();

        DoubleSubscriber dataSubscriber = datatable.getDoubleTopic(topic).subscribe(0,
                PubSubOption.pollStorage(1), PubSubOption.keepDuplicates(true));

        return new Trigger(() -> dataSubscriber.readQueueValues().length > 0)
                .onTrue(selfCancelingCommand(topic, command));
    }

    // Array of doubleSubscribers to avoid recreation on every trigger

    private static HashMap<String, DoubleSubscriber> DoubleSubscriberMap = new HashMap<String, DoubleSubscriber>();

    // AXIS + NUMBER COMPONENT value getter
    public static double getDoubleValue(String topic) {
        if (DoubleSubscriberMap.containsKey(topic)) {
            // System.out.println(DoubleSubscriberMap.size());
            return DoubleSubscriberMap.get(topic).get();
        } else {
            final DoubleSubscriber dataSubscriber = datatable.getDoubleTopic(topic).subscribe(0.0);

            DoubleSubscriberMap.put(topic, dataSubscriber);

            return DoubleSubscriberMap.get(topic).get();
        }
    }

    // Dropdown Methods

    public static Trigger bindDropdown(String topic, Supplier<Command> command) {

        StringSubscriber dataSubscriber = datatable.getStringTopic(topic).subscribe("",
                PubSubOption.pollStorage(2));

        return new Trigger(() -> dataSubscriber.readQueueValues().length > 0)
                .onTrue(selfCancelingCommand(topic, command));
    }

    // Opt Group Methods

    public static Trigger bindOptGroup(String topic, Supplier<Command> command) {

        StringSubscriber dataSubscriber = datatable.getStringTopic(topic).subscribe("",
                PubSubOption.pollStorage(2));

        return new Trigger(() -> dataSubscriber.readQueueValues().length > 0)
                .onTrue(selfCancelingCommand(topic, command));
    }

    private static HashMap<String, StringSubscriber> StringSubscriberMap = new HashMap<String, StringSubscriber>();

    // OPT GROUP + DROPDOWN COMPONENT value getter
    public static String getStringValue(String topic) {
        if (StringSubscriberMap.containsKey(topic)) {
            return StringSubscriberMap.get(topic).get();
        } else {
            final StringSubscriber dataSubscriber = datatable.getStringTopic(topic).subscribe("");

            StringSubscriberMap.put(topic, dataSubscriber);

            return StringSubscriberMap.get(topic).get();
        }
    }

    // Helper Methods:

    private static final HashMap<String, Command> selfCancelingCommands = new HashMap<>();

    private static Command selfCancelingCommand(String topic, Supplier<Command> commandSupplier) {
        return Commands.runOnce(() -> {
            Command previousCommand = selfCancelingCommands.get(topic);
            if (previousCommand != null) {
                previousCommand.cancel();
            }

            Command newCommand = Commands.deferredProxy(commandSupplier);
            selfCancelingCommands.put(topic, newCommand);
            CommandScheduler.getInstance().schedule(newCommand);
        }).ignoringDisable(true);
    }
}
