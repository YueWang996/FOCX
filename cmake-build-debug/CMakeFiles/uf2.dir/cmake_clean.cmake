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
  "CMakeFiles/uf2"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/uf2.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
