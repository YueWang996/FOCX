file(REMOVE_RECURSE
  "app-template.bin"
  "app-template.map"
  "bootloader/bootloader.bin"
  "bootloader/bootloader.elf"
  "bootloader/bootloader.map"
  "config/sdkconfig.cmake"
  "config/sdkconfig.h"
  "flash_project_args"
  "project_elf_src_esp32.c"
  "CMakeFiles/app-template.elf.dir/project_elf_src_esp32.c.obj"
  "app-template.elf"
  "app-template.elf.manifest"
  "app-template.elf.pdb"
  "project_elf_src_esp32.c"
)

# Per-language clean rules from dependency scanning.
foreach(lang C)
  include(CMakeFiles/app-template.elf.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
