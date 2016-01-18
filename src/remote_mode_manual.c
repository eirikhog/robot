

void UpdateManualMode(volatile InputState *input) {

    // TODO:
    // Get joystick X/Y
    uint16_t x = input->joystick.x;
    uint16_t y = input->joystick.y;
    // Determine direction to go
    // Set motor values.
    // NB: Make sure not to override the motors!
    
    const uint16_t deadzone = 20;
    
    // How to determine direction and speed:
    // X axis determines direction
    //   --> Rotate control vector
    // Y axis determines speed
    
    int32_t vec_x = ((int32_t)x - 0x3FF/2);
    int32_t vec_y = ((int32_t)y - 0x3FF/2);
    
    int16_t motor_left = 0;
    int16_t motor_right = 0;

    const int32_t vec_max = 0x3FF / 2;
    const int32_t motor_max = 0xFF;

    int16_t vec_x_abs = (uint16_t)abs(vec_x);
    if (vec_x_abs >= deadzone) {
        if (vec_x < 0) {
            // Turn left
            motor_left = (int16_t)(-vec_x_abs * motor_max / vec_max);
            motor_right = (int16_t)(vec_x_abs * motor_max / vec_max);
        } else {
            // Turn right
            motor_left = (int16_t)(vec_x_abs * motor_max / vec_max);
            motor_right = (int16_t)(-vec_x_abs * motor_max / vec_max);
        }
    }

    int16_t motor_left_f = 0;
    int16_t motor_right_f = 0;
    int16_t vec_y_abs = (int16_t)abs(vec_y);
    if (vec_y_abs >= deadzone) {
        motor_left_f = (vec_y_abs * motor_max / vec_max);
        motor_right_f = (vec_y_abs * motor_max / vec_max);
        if (vec_y > 0) {
            // Go backwards
            motor_left_f *= -1;
            motor_right_f *= -1;
        }
    }

    motor_left += motor_left_f;
    motor_right += motor_right_f;
    
    if (motor_left > 0xFF) {
        motor_left = 0xFF;
    } else if  (motor_left < -0xFF) {
        motor_left = -0xFF;
    }

    if (motor_right > 0xFF) {
        motor_right = 0xFF;
    } else if (motor_right < -0xFF) {
        motor_right = -0xFF;
    }

    // TODO: Send motor command to robot.

    // Debug output:
    printf("D: %d %d\n", motor_left, motor_right);

}
