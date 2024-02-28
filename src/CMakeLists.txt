# Basic Test
add_executable(blink blink.c)
avr_add_extra_outputs(blink)

# Basic Test
add_executable(blink-test blink-test.c)
target_link_libraries(blink-test freertos)
avr_add_extra_outputs(blink-test)

# Servo Test
add_executable(servo-test servo-test.c)
target_link_libraries(servo-test freertos)
avr_add_extra_outputs(servo-test)

# UART Test
add_executable(uart-test uart-test.c)
target_link_libraries(uart-test freertos)
avr_add_extra_outputs(uart-test)

# Stream Buffer Test
add_executable(streambuffer-test streambuffer-test.c)
target_link_libraries(streambuffer-test freertos)
avr_add_extra_outputs(streambuffer-test)

