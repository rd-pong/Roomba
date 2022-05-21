# Under Table Disinfection User Manual

# Run the Code

```bash
$ python3 disinfect_main_api.py
```

# Update iRobot Create 2 Firmware

You MUST update the iRobot Create 2 Firmware to the latest to avoid incorrect sensor values returned. See [Create 2 Open Interface (OI) Specification](./api/pycreate2/docs/iRobot/iRobot_Roomba_600_Open_Interface_Spec.pdf) "Appendix B: Known Bugs".

> The download will ONLY work on Windows computers; we have tested on Win 7 and 10. Connect your robot to your computer with the Create cable. You will need to unzip, download and install the `.exe` file, and then pick one of the two firmware `.enc` files provided and input your serial port corresponding to the Create cable. If you don’t pick the right one for your robot, it will fail but then just pick the other one! Click start. You will have either Firmware 3.7.8 or 3.8.2 depending on your processor when finished.

See `./iRobotCreate2_Update_Windows` directory in Cornell Box for update tool and firmware files. Please send an email to create@irobot.com if you have more questions.

# Modify, Build, and Install `pycreate2` Package

> https://github.com/MomsFriendlyRobotCompany/pycreate2

1. Modify the package to support clean mode call (or any other necessary new features).
2. Build the package using `poetry` and install the package locally with `pip3`.

    > https://python-poetry.org/docs/
    
    > The `poetry build` command will create a wheel and a tar file and place it in the `my_pacakge/dist` directory which you can then `pip install` into your environment of choice.
    
    > `pip install my_package-version.tar.gz`

    ```bash
    $ cd api/pycreate2/
    $ poetry build

    $ cd dist/
    $ pip3 install pycreate2-0.8.1.tar.gz
    ```

# Robot Arm Control via Raspberry Pi

> WARNING! Do NOT press any button on the arm controller board.

> Please also refer to the [official tutorial documentation](https://www.hiwonder.com/store/learn/42.html) from Hiwonder.

The Hiwonder xArm 1S robot arm communicates with Raspberry Pi using serial communication protocol. To set up a serial port connection,

```python
arm = serial.Serial(port='/dev/ttyS0',
                    baudrate=9600,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                    timeout=1)
```

Currently, 6 predefined positions are stored in the arm controller.

* xArm Status 0 → Storage Mode → armStorage()
* xArm Status 1 → Wandering Mode → armDown()
* xArm Status 2 → Table Disinfection Mode → armUp()
* xArm Status 3 → Looking for Keyboard → armSearchKeyboard()
* xArm Status 4 → Clean Keyboard Start→ armCleanKeyboardStart()
* xArm Status 5 → Clean Keyboard End → armCleanKeyboardEnd()

To set the arm to a specific position,

```python
# stop/reset arm
arm.write(b'\x55\x55\x02\x07')

# set to position #ID stored in the arm controller board
arm.write(b'\x55\x55\x05\x06\xPositionID\x01\x00')
```

For example, to set the arm to the up position when doing table disinfection (xArm Status 2), 

```python
arm.write(b'\x55\x55\x02\x07')
arm.write(b'\x55\x55\x05\x06\x02\x01\x00')
```

If you want to define more positions, please refer to [Hiwonder's tutorial](https://www.hiwonder.com/store/learn/42.html). The recommended way is to use their official PC software. You need to connect the arm controller board to your PC. In "Manual Coding" mode, manually put the servo positions of the arm to the desired position and click "Read Angle". Then choose a new position ID for this setting and click "Download" to save the position to the arm controller board. Carefully double-check the position ID before downloading, the data in the controller board cannot be reverted.
