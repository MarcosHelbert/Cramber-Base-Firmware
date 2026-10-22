import re

Import("env")

version = env.GetProjectConfig().get("firmware", "version", "")
match = re.fullmatch(r"(\d+)\.(\d+)\.(\d+)", version.strip())
if not match:
    raise SystemExit(f"platformio.ini: [firmware] version must be major.minor.patch, got '{version}'")

major, minor, patch = (int(part) for part in match.groups())
if minor > 255 or patch > 255:
    raise SystemExit("platformio.ini: [firmware] version minor and patch go up to 255")

env.Append(CPPDEFINES=[
    ("FIRMWARE_VERSION_MAJOR", major),
    ("FIRMWARE_VERSION_MINOR", minor),
    ("FIRMWARE_VERSION_PATCH", patch),
])
