import os
import struct
import subprocess
import zlib

Import("env")

BOOT_ADDR = 0x08000000
APP_HEADER_ADDR = 0x08008000
APP_HEADER_SIZE = 0x200
APP_HEADER_MAGIC = 0x43524D41


def _build_bootloader():
    bootloader_dir = os.path.join(env.subst("$PROJECT_DIR"), "bootloader")
    subprocess.check_call([env.subst("$PYTHONEXE"), "-m", "platformio", "run"], cwd=bootloader_dir)
    with open(os.path.join(bootloader_dir, ".pio", "build", "bootloader", "firmware.bin"), "rb") as f:
        return f.read()


def _write_full_firmware():
    bootloader = _build_bootloader()
    with open(env.subst("$BUILD_DIR/${PROGNAME}.bin"), "rb") as f:
        app = f.read()

    magic, _size, _crc, version = struct.unpack_from("<4I", app, 0)
    if magic != APP_HEADER_MAGIC:
        raise SystemExit("full firmware: the application has no header (is it linked with app.ld?)")
    if len(bootloader) > APP_HEADER_ADDR - BOOT_ADDR:
        raise SystemExit("full firmware: the bootloader doesn't fit in sectors 0-1")

    # The real size and CRC-32, so the bootloader checks the image from the first boot.
    body = app[APP_HEADER_SIZE:]
    header = struct.pack("<4I", APP_HEADER_MAGIC, len(body), zlib.crc32(body) & 0xFFFFFFFF, version)
    erased = bytes([0xFF])
    full = (bootloader + erased * (APP_HEADER_ADDR - BOOT_ADDR - len(bootloader))
            + header + erased * (APP_HEADER_SIZE - len(header)) + body)

    path = env.subst(f"$BUILD_DIR/cramber-dd-v{version}.bin")
    with open(path, "wb") as f:
        f.write(full)
    print(f"Full firmware: {path}")
    print(f"  bootloader {len(bootloader)} B, application v{version} {len(body)} B")
    return path


def build_full_firmware(target, source, env):
    _write_full_firmware()


def upload_full_firmware(target, source, env):
    path = _write_full_firmware().replace("\\", "/")
    openocd = env.PioPlatform().get_package_dir("tool-openocd")
    subprocess.check_call([
        os.path.join(openocd, "bin", "openocd"),
        "-s", os.path.join(openocd, "openocd", "scripts"),
        "-f", "interface/stlink.cfg",
        "-c", "transport select hla_swd",
        "-f", "target/stm32f4x.cfg",
        "-c", f"program {{{path}}} 0x{BOOT_ADDR:08X} verify reset; shutdown;",
    ])


env.AddCustomTarget(
    name="build_full_firmware",
    dependencies="$BUILD_DIR/${PROGNAME}.bin",
    actions=[build_full_firmware],
    title="Build full firmware",
    description="Bootloader + application in one .bin, for a programmer or Cramber Hub",
)

env.AddCustomTarget(
    name="upload_full_firmware",
    dependencies="$BUILD_DIR/${PROGNAME}.bin",
    actions=[upload_full_firmware],
    title="Upload full firmware",
    description="Build the full firmware and flash it with the ST-Link (bootloader + application)",
)
