package frc.robot.util.Tests;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.Touchboard;

public class tbCommandTest {

    tbCommandTest() {
        // Test code here
        Touchboard.bindActionButton("abTest", Commands.run(() -> {
            int abIndex = 0;

            System.out.println("abTest pressed" + abIndex);
            abIndex++;
        }));

        Touchboard.bindToggleButton("togTest", Commands.run(() -> {
            int abIndex = 0;

            System.out.println("togTest pressed" + abIndex);
            abIndex++;
        }));

        Touchboard.bindOneShotButton("oneShotTest", Commands.run(() -> {
            int abIndex = 0;

            System.out.println("oneShotTest pressed" + abIndex);
            abIndex++;
        }));
    }
}
