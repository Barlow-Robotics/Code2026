import subprocess
import sys

# Packages that only exist in the root pyproject.toml for local dev/sim
# and should NOT be expected in src/pyproject.toml
HOST_ONLY_PACKAGES = {"debugpy", "ruff"}


def main():
    if not _check_deps():
        sys.exit(1)

    subcmd = sys.argv[1] if len(sys.argv) > 1 else None

    if subcmd == "check":
        _run("uv", "run", "ruff", "check", "src")
    elif subcmd == "format":
        _run("uv", "run", "ruff", "format", "src")
    elif subcmd == "fix":
        _run("uv", "run", "ruff", "check", "--fix", "src")
    else:
        # Pass through to robotpy with --main src
        sys.argv.insert(1, "--main")
        sys.argv.insert(2, "src")
        from robotpy.main import main as robotpy_main

        robotpy_main()


def _check_deps():
    """Verify that root and src pyproject.toml dependencies are in sync."""
    import tomli
    from pathlib import Path
    from packaging.requirements import Requirement

    root_path = Path(__file__).resolve().parent
    root_pyproject = root_path / "pyproject.toml"
    src_pyproject = root_path / "src" / "pyproject.toml"

    with open(root_pyproject, "rb") as f:
        root_data = tomli.load(f)
    with open(src_pyproject, "rb") as f:
        src_data = tomli.load(f)

    # Parse root dependencies into normalized package names
    root_deps = {}
    robotpy_extras = set()
    for dep_str in root_data.get("project", {}).get("dependencies", []):
        req = Requirement(dep_str)
        name = req.name.lower().replace("-", "_")
        root_deps[name] = req
        if name == "robotpy" and req.extras:
            robotpy_extras = {e.lower().replace("-", "_") for e in req.extras}

    # Parse src robotpy config
    src_robotpy = src_data.get("tool", {}).get("robotpy", {})
    src_components = {c.lower().replace("-", "_") for c in src_robotpy.get("components", [])}
    src_requires = set()
    for req_str in src_robotpy.get("requires", []):
        req = Requirement(req_str)
        src_requires.add(req.name.lower().replace("-", "_"))

    errors = []

    # Check: src requires should all appear in root dependencies
    for pkg in sorted(src_requires):
        if pkg not in root_deps:
            errors.append(
                f'  "{pkg}" is in src/pyproject.toml [tool.robotpy] requires\n'
                f'  but missing from pyproject.toml [project] dependencies.\n'
                f'  -> Add "{pkg}" to the [project] dependencies in pyproject.toml\n'
                f'     so it is available locally for simulation.'
            )

    # Check: src components should all appear in root robotpy extras
    for comp in sorted(src_components):
        if comp not in robotpy_extras:
            errors.append(
                f'  robotpy component "{comp}" is in src/pyproject.toml [tool.robotpy] components\n'
                f'  but missing from the robotpy[...] extras in pyproject.toml.\n'
                f'  -> Add "{comp}" to the robotpy extras in pyproject.toml, e.g.:\n'
                f'     "robotpy[...,{comp},sim]"'
            )

    # Check: root non-dev, non-robotpy deps should appear in src requires
    for name in sorted(root_deps):
        if name in HOST_ONLY_PACKAGES or name == "robotpy":
            continue
        if name not in src_requires:
            errors.append(
                f'  "{name}" is in pyproject.toml [project] dependencies\n'
                f'  but missing from src/pyproject.toml [tool.robotpy] requires.\n'
                f'  -> Add "{name}" to the requires list in src/pyproject.toml\n'
                f'     so it gets installed on the robot.\n'
                f'  -> Or if this is a dev-only package, add it to HOST_ONLY_PACKAGES\n'
                f'     in _frc_cli.py to silence this warning.'
            )

    if errors:
        print("\nDependency mismatch between pyproject.toml and src/pyproject.toml:\n")
        print("\n\n".join(errors))
        print("\nBoth files must stay in sync. See README for details.\n")
        return False

    return True


def _run(*args):
    sys.exit(subprocess.call(args))
