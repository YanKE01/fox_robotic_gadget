#include <stdio.h>
#include <iostream>
#include <string.h>
#include "unity.h"
#include "unity_config.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "robotbox.h"

#define TEST_MEMORY_LEAK_THRESHOLD (-460)

static size_t before_free_8bit;
static size_t before_free_32bit;

static void check_leak(size_t before_free, size_t after_free, const char *type)
{
    ssize_t delta = after_free - before_free;
    printf("MALLOC_CAP_%s: Before %u bytes free, After %u bytes free (delta %d)\n", type, before_free, after_free, delta);
    TEST_ASSERT_MESSAGE(delta >= TEST_MEMORY_LEAK_THRESHOLD, "memory leak");
}

TEST_CASE("robotbox test six-axis robotic arm", "[robotbox]")
{
    Eigen::MatrixXd RODH(6, 4);

    //DH d,a,alpha,offset,theta
    RODH <<  131.22, 0.0, (PI / 2), 0.0,
         0.0, -110.4, 0.0, (-PI / 2),
         0.0, -96, 0.0, 0.0,
         63.4, 0.0, (PI / 2), (-PI / 2),
         75.05, 0.0, (-PI / 2), (PI / 2),
         45.6, 0.0, 0.0, 0.0;
    std::cout << "set DH d,a,alpha,offset:\n" << RODH << std::endl;

    robotbox robot(RODH);
    Eigen::MatrixXd forwardf;
    std::vector<double> theta = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    robot.fkine(forwardf, theta);
    std::cout << "forward kinematics:\n" << forwardf << std::endl;

    theta = {PI / 2, -PI / 3, 0, -PI / 6, 0, 0};
    robot.fkine(forwardf, theta);
    std::cout << "forward kinematics:\n" << forwardf << std::endl;

    std::vector<double> theikine;
    bool result = robot.ikine(theikine, forwardf, { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, {-2.9, -2.9, -2.9, -2.9, -2.9, -3.1}, {2.9, 2.9, 2.9, 2.9, 2.9, 3.1}, 1000, 100, 1);

    if (result) {
        std::cout << "The inverse kinematics: ";
        for (size_t i = 0; i < theikine.size(); i++) {
            std::cout << theikine[i] << " ";
        }
        std::cout << std::endl;
    }

    printf("test robotbox finish\n");
}

void setUp(void)
{
    before_free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    before_free_32bit = heap_caps_get_free_size(MALLOC_CAP_32BIT);
}

void tearDown(void)
{
    size_t after_free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    size_t after_free_32bit = heap_caps_get_free_size(MALLOC_CAP_32BIT);
    check_leak(before_free_8bit, after_free_8bit, "8BIT");
    check_leak(before_free_32bit, after_free_32bit, "32BIT");
}

extern "C" void app_main(void)
{
    printf("Test Robotbox \n");
    unity_run_menu();
}