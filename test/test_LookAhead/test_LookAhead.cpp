#include <unity.h>
#include "LookAhead.h"

LookAhead lookAhead;

void test_collectSensorData() {
    lookAhead.init();
    lookAhead.collectSensorData(0b000001);
    TEST_ASSERT_EQUAL(0b000001, lookAhead.sensorData[0][0]);
}

void test_calculateLinePosition() {
    int linePosition = lookAhead.calculateLinePosition(0b000111);
    TEST_ASSERT_EQUAL(-2, linePosition);
    linePosition = lookAhead.calculateLinePosition(0b111000);
    TEST_ASSERT_EQUAL(2, linePosition);
}

void test_getLookAheadPoint() {
    lookAhead.init();
    lookAhead.collectSensorData(0b000111);
    lookAhead.collectSensorData(0b001110);
    lookAhead.collectSensorData(0b011100);
    LookAhead::Point lookAheadPoint = lookAhead.getLookAheadPoint(10);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0.0, lookAheadPoint.x); // Adjust expected value
    TEST_ASSERT_FLOAT_WITHIN(0.1, 1.5, lookAheadPoint.y); // Adjust expected value
}

void test_PID() {
    float output = lookAhead.PID(1.0);
    TEST_ASSERT_FLOAT_WITHIN(0.1, lookAhead.m_kp, output);
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_collectSensorData);
    RUN_TEST(test_calculateLinePosition);
    RUN_TEST(test_getLookAheadPoint);
    RUN_TEST(test_PID);
    UNITY_END();
}

void loop() {
    // Empty loop
}
