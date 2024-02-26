add_executable(main main.c)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(main pico_stdlib pico_cyw43_arch_none hardware_i2c hardware_pwm hardware_uart hardware_clocks)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(main)

# enable usb output, disable uart output
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)
