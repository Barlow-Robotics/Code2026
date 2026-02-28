# Code2026

## Installation
All dependencies are managed via [uv](https://docs.astral.sh/uv/).
- Please follow the [installation instructions](https://docs.astral.sh/uv/getting-started/installation/#standalone-installer)

Once you have uv installed, you only need to run:
```
uv run frc sync
```

## Best Practices
We use ruff for formatting code. This ensures consistency and minimizes white-space-related conflicts.
 - VS Code should be set up to install the `charliemarsh.ruff` extension and will format-on-save.
 - Before pushing code always format using ruff:
    ```
    uv run ruff format
    uv run ruff check
    ```

## To simulate the robot
```
uv run frc sim
```

## To deploy to the robot
```
uv run frc deploy
```

If you want to skip running the tests
```
uv run frc deploy --skip-tests
```

## To run back a log file in replay mode, set the `LOG_PATH` environment variable and then run in simulation

An example is to run the following:

```sh
LOG_PATH=/path/to/log/file.wpilog uv run -- robotpy --main src sim
```

For replay watch, do the following:

```sh
LOG_PATH=/path/to/log/file.wpilog uv run -- robotpy --main src watch
```


Notes:
USB: 172.22.11.2
Radio: 10.45.72.1
RoboRIO: 10.45.72.2
Vision: 10.45.72.201
SSH into RoboRIO: admin@roboRIO-4572-frc.local


TODO at comp:
AprilTagField.k2026RebuiltAndyMark or AprilTagField.k2026RebuiltWelded depending on comp. 
Change motors from L2 to L1 or L3

2. Need new cameras
3. if len(targets) == 1 and targets[0].getPoseAmbiguity() > POSE_AMBIGUITY: OR just f len(targets) == 1
9. Need to update deploy/pathplanner/settings.json
12. Update Constants 