add_executable(main main.c)
add_executable(rc_read rc_read.c)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(main pico_stdlib pico_cyw43_arch_none hardware_i2c hardware_pwm hardware_uart hardware_clocks)
target_link_libraries(rc_read pico_stdlib pico_cyw43_arch_none hardware_i2c hardware_pwm hardware_uart hardware_clocks)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(rc_read main)

# enable usb output, disable uart output
pico_enable_stdio_usb(rc_read main 1)
pico_enable_stdio_uart(rc_read main 1)

# Force program to load into RAM, saving write cycles on flash memory
pico_set_binary_type(rc_read main no_flash)