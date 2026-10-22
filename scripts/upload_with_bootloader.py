import os
import subprocess

Import("env")


def _platformio(project_dir, *args):
    subprocess.check_call([env.subst("$PYTHONEXE"), "-m", "platformio", "run", *args], cwd=project_dir)


def upload_with_bootloader(target, source, env):
    project_dir = env.subst("$PROJECT_DIR")
    _platformio(os.path.join(project_dir, "bootloader"), "-t", "upload")
    _platformio(project_dir, "-e", env.subst("$PIOENV"), "-t", "upload")


env.AddCustomTarget(
    name="upload_with_bootloader",
    dependencies=None,
    actions=[upload_with_bootloader],
    title="Upload with bootloader",
    description="Flash the bootloader, then the application (first install on a new board)",
)
