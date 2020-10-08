#pragma once

float interpolate(float value, float input_min, float input_max, float output_min, float output_max) {
    return (value - input_min) / (input_max - input_min) * (output_max - output_min) + output_min;
}

// Consolidate these into smooth_interpolate and take a phase_shift as input?
float smooth_interpolate_decrease(float value, float input_min, float input_max, float output_min, float output_max) {
    double sin_input = interpolate(value, input_min, input_max, 0, PI);
    double sin_output = sin(sin_input + (PI / 2)) + 1;
    return interpolate(sin_output, 0, 2, output_min, output_max);
}

float smooth_interpolate_increase(float value, float input_min, float input_max, float output_min, float output_max) {
    float sin_input = interpolate(value, input_min, input_max, 0, PI);
    float sin_output = sin(sin_input - (PI / 2)) + 1;
    return interpolate(sin_output, 0, 2, output_min, output_max);
}