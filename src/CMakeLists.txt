add_executable(test test.c)

target_link_libraries(test freertos)

set_target_properties(test PROPERTIES
    OUTPUT_NAME "test.elf"
)

# Generate output files
add_custom_target(test-strip ALL avr-strip test.elf DEPENDS test)
add_custom_target(test-hex ALL avr-objcopy -R .eeprom -O ihex test.elf test.hex DEPENDS test-strip)
add_custom_target(test-eeprom avr-objcopy -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 -O ihex test.elf test.eep DEPENDS test-strip)

# Clean extra files
# TODO: make this actually work
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "test.hex;test.eep;test.lst")