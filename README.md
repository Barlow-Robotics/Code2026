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


TODO at comp:
AprilTagField.k2026RebuiltAndyMark or AprilTagField.k2026RebuiltWelded depending on comp. 
Change motors from L2 to L1 or L3
Need to update _should_reject_by_alliance based on alliance

1. Need new Gyro
2. Need new cameras
3. Gyro comparison to disambiguate
4. Separate pose estimator for precise alignment