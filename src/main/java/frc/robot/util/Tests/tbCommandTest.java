package frc.robot.util.Tests;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.Touchboard;

public class tbCommandTest {

    public tbCommandTest() {
        // Test code here
            int abIndex = 0;

        Touchboard.bindActionButton("abTest", Commands.run(() -> {

            System.out.println("abTest pressed" + abIndex);
        }));

        Touchboard.bindToggleButton("togTest", Commands.run(() -> {

            System.out.println("togTest pressed" + abIndex);
        }));

        Touchboard.bindOneShotButton("oneShotTest", Commands.run(() -> {

            System.out.println("oneShotTest pressed" + abIndex);
        }));
    }
}
