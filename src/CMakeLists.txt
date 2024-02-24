# Basic Test
add_executable(servo-test servo-test.c)
target_link_libraries(servo-test freertos)
avr_add_extra_outputs(servo-test)

# UART Test
add_executable(uart-test uart-test.c)
target_link_libraries(uart-test freertos)
avr_add_extra_outputs(uart-test)

