#include "sim_task.h"

void sim_task(void* parameter) {
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1) {
        // Straight forward
        motor_data_t msg = { MOTOR_MOVE, 0.3f, 0.3f };
        DDS_PUBLISH("/motor", msg);
        vTaskDelay(pdMS_TO_TICKS(3000));

        // Semicircle
        msg = { MOTOR_MOVE, 0.1f, 0.3f };
        DDS_PUBLISH("/motor", msg);
        vTaskDelay(pdMS_TO_TICKS(7800));

        // Straight back
        msg = { MOTOR_MOVE, -0.3f, -0.3f };
        DDS_PUBLISH("/motor", msg);
        vTaskDelay(pdMS_TO_TICKS(3000));

        // Semicircle back
        msg = { MOTOR_MOVE, -0.3f, -0.1f };
        DDS_PUBLISH("/motor", msg);
        vTaskDelay(pdMS_TO_TICKS(7800));

        // Stop
        msg = { MOTOR_STOP, 0.0f, 0.0f };
        DDS_PUBLISH("/motor", msg);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}