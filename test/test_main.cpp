#include <Arduino.h>
#include <unity.h>

void setUp(void){}

void tearDown(void){}

void test_test(void){
    TEST_ASSERT_TRUE(1==1);
}

void setup(){
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_test);
}

void loop(){
    UNITY_END();
}