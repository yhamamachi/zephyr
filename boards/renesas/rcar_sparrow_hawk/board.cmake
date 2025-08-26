# SPDX-License-Identifier: Apache-2.0
if(CONFIG_BOARD_RCAR_SPARROW_HAWK_R8A779G3_R52)
  board_runner_args(openocd "--use-elf")
  include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
endif()
