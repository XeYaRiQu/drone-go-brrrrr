add_executable(main_test main_test.c)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(main_test pico_stdlib pico_cyw43_arch_none hardware_i2c hardware_pwm hardware_uart hardware_clocks)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(main_test)

# enable usb output, disable uart output
pico_enable_stdio_usb(main_test 1)
pico_enable_stdio_uart(main_test 0)

# Force program to load into RAM, saving write cycles on flash memory
pico_set_binary_type(main_test no_flash)