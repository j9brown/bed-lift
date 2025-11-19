# Import the current working construction
# environment to the `env` variable.
# alias of `env = DefaultEnvironment()`
Import("env")

# Platform IO does not execute the actions specified by Zephyr's CONFIG_OUTPUT_DISASSEMBLY
# or CONFIG_OUTPUT_SYMBOLS options, so we implement them as custom actions here.
env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(" ".join([
        "arm-none-eabi-objdump", "-d", "-S", "$BUILD_DIR/${PROGNAME}.elf", ">", "$BUILD_DIR/${PROGNAME}.lst"
    ]), "Generating disassembly in $BUILD_DIR/${PROGNAME}.lst")
)
env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(" ".join([
        "arm-none-eabi-nm", "$BUILD_DIR/${PROGNAME}.elf", ">", "$BUILD_DIR/${PROGNAME}.symbols"
    ]), "Generating symbols in $BUILD_DIR/${PROGNAME}.symbols")
)
env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(" ".join([
        "arm-none-eabi-readelf", "-e", "$BUILD_DIR/${PROGNAME}.elf", ">", "$BUILD_DIR/${PROGNAME}.stat"
    ]), "Generating ELF section statistics in $BUILD_DIR/${PROGNAME}.stat")
)
