#pragma once

class Filter {
    public:
        virtual float filter(float rawValue) = 0;
        virtual float getFilteredValue() const = 0;
};

class ExponentialSmoothingFilter : public Filter {
    private:
        // Must be between 0-1
        float smoothingFactor;
        float lastFilteredValue = 0;
        float filteredValue = 0;
    
    public:
        ExponentialSmoothingFilter(float smoothingFactor) : smoothingFactor(smoothingFactor) {};

        float filter(float rawValue) override {
            filteredValue = lastFilteredValue + smoothingFactor * (rawValue - lastFilteredValue);
            lastFilteredValue = filteredValue;
            return filteredValue;
        }
        float getFilteredValue() const { return filteredValue; }
};

class DerivativeCalculator {
    private:
        float lastValue;
        float lastTimestamp;
    public:
        float getDerivative(float rawValue, float timestamp) {
            float derivative = (rawValue - lastValue) / (timestamp - lastTimestamp);
            lastValue = rawValue;
            lastTimestamp = timestamp;
            return derivative;
        }
};