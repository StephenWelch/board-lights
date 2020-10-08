#pragma once
#include <Arduino.h>

class Plotter {
    private:
        bool first = true;
    public:
        template <typename T> void add(const char *name, const T &value) {
            if(first) {
                first = false;
            } else {
                // Add delimiter if this is not the first value we are plotting
                Serial.print(",");
            }

            Serial.print(name);
            Serial.print(":");
            Serial.print(value);
        }

        void commit() {
            first = true;
            Serial.println();
        }
};