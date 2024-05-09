#include <Arduino.h>
#include <unity.h>
#include "Stepper.h"

void test_stepper_initialization() {
    Stepper stepper(200, 2, 3, 4, 5);
    TEST_ASSERT_EQUAL(200, 300);
    // Add more assertions as needed
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_stepper_initialization);
    UNITY_END();
}

void loop() {
    // usually nothing here
}