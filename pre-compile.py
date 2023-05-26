import os, re, shutil, errno

path = 'Marlin'
dest = 'modified'
list = [
  "backlash.cpp", 
  "M17_M18_M84.cpp",
  "gcode.cpp"
]

for root, dirs, files in os.walk(path):
  with os.scandir(os.path.join(root)) as directory:
    for file in directory:
      if not file.name.startswith('.') and file.is_file():
        try:
          shutil.copy2(file.path, os.path.join(dest, file.path))
        except IOError as e:
          if e.errno != errno.ENOENT:
            raise
          os.makedirs(os.path.dirname(os.path.join(dest, file.path)), exist_ok=True)
          shutil.copy2(file.path, os.path.join(dest, file.path))

        if file.name in list:
          with open(os.path.join(dest, file.path), mode="r+") as content:
            lines = content.read()
            lines = re.sub(r'([A-Za-z])({)', r'\1 = \2', lines)
            content.seek(0)
            content.write(lines)
            content.close()
            print("Modified: " + os.path.join(dest, file.path))

        if file.name == "serial_hook.h":
          with open(os.path.join(dest, file.path), mode="r+") as content:
            lines = content.read()
            lines = lines.replace("static constexpr uint8_t output[] =", "static constexpr uint8_t output =")
            lines = lines.replace("#define _OUT_MASK(N) | output[N]", "#define _OUT_MASK(N) | output")
            content.seek(0)
            content.write(lines)
            content.close()
            print("Modified: " + os.path.join(dest, file.path))

        if file.name == "utility.h":
          with open(os.path.join(dest, file.path), mode="r+") as content:
            lines = content.read()
            lines = lines.replace("const xyze_char_t axis_codes LOGICAL_AXIS_ARRAY", "const xyze_char_t axis_codes = LOGICAL_AXIS_ARRAY")
            lines = lines.replace("const xyze_char_t iaxis_codes LOGICAL_AXIS_ARRAY", "const xyze_char_t iaxis_codes = LOGICAL_AXIS_ARRAY")
            content.seek(0)
            content.write(lines)
            content.close()
            print("Modified: " + os.path.join(dest, file.path))

        if file.name == "MarlinConfigPre.h":
          with open(os.path.join(dest, file.path), mode="r+") as content:
            lines = content.read()
            lines = lines.replace("<stdint.h>", "<stdint.h> \n#include <startup.h>")
            content.seek(0)
            content.write(lines)
            content.close()
            print("Modified: " + os.path.join(dest, file.path))

        if file.name == "numtostr.cpp":
          with open(os.path.join(dest, file.path), mode="r+") as content:
            lines = content.read()
            lines = lines.replace("pow", "POW")
            content.seek(0)
            content.write(lines)
            content.close()
            print("Modified: " + os.path.join(dest, file.path))

        if file.name == "trinamic.cpp":
          with open(os.path.join(dest, file.path), mode="r+") as content:
            lines = content.read()
            lines = lines.replace("<SPI.h>", "<SPI.h> \n#include \"../pins/pins.h\"")
            lines = re.sub(r'(conf)({)', r'\1 = \2', lines)
            content.seek(0)
            content.write(lines)
            content.close()
            print("Modified: " + os.path.join(dest, file.path))

        if file.name == "stepper.h":
          with open(os.path.join(dest, file.path), mode="r+") as content:
            lines = content.read()
            lines = lines.replace("static xyze_long_t advance_dividend;", "static xyze_ulong_t advance_dividend;")
            content.seek(0)
            content.write(lines)
            content.close()
            print("Modified: " + os.path.join(dest, file.path))

        if file.name == "stepper.cpp":
          with open(os.path.join(dest, file.path), mode="r+") as content:
            lines = content.read()
            lines = lines.replace("delta_error{", "delta_error = {")
            lines = lines.replace("xyze_long_t Stepper::advance_dividend{", "xyze_ulong_t Stepper::advance_dividend = {")
            lines = lines.replace("count_position{", "count_position = {")
            lines = lines.replace("count_direction{", "count_direction = {")
            lines = lines.replace("step_needed{", "step_needed = {")
            content.seek(0)
            content.write(lines)
            content.close()
            print("Modified: " + os.path.join(dest, file.path))

