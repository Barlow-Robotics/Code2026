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

Notes:
USB: 172.22.11.2
Radio: 10.45.72.1
RoboRIO: 10.45.72.2
Vision: 10.45.72.201


TODO at comp:
AprilTagField.k2026RebuiltAndyMark or AprilTagField.k2026RebuiltWelded depending on comp. 
Change motors from L2 to L1 or L3

2. Need new cameras
3. if len(targets) == 1 and targets[0].getPoseAmbiguity() > POSE_AMBIGUITY: OR just f len(targets) == 1
4. Gyro magnitude problem, min_diff = min(diff_best, diff_alt); min_diff > math.radians(30):
5. Update gyro based on vision. 
6. If angular_velocity is high don't use vision. 
7. have to run uv pip install robotpy-pathplannerlib not just run ``` frc sync```. 
8. Figure out what AutoConstants.period is (and how it effects AutoBuilder)
9. Need to update deploy/pathplanner/settings.json
10. Need to figure out how to get pose to change in Sim. 