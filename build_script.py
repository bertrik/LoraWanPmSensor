Import("env")
import shutil

# copies the firmware.bin file to a file with a target-specific name
def copy_firmware_bin(source, target, env):
    binfile_name = f'{env["PROJECT_DIR"]}/{env["PIOENV"]}.bin'
    shutil.copy(target[0].path, binfile_name)

env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", copy_firmware_bin)

