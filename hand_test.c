#include <stdio.h>
#include "hand_sdk.h"

int main()
{
    printf("hello\n");
    hand_sdk.init();
    // printf("hand_sdk intialized done");
    // int8_t position[6] = {100, 100, 50, 50, 50, 50};
    // int8_t speed[6] = {100, 100, 0, 0, 100, 100};
    // int8_t current[6] = {100, 100, 0, 0, 100, 100};
    // hand_sdk.qiangnao_port[0] = 0;
    // hand_sdk.qiangnao_port[1] = 0;
    // char *com_list[] = {"/dev/ttyUSB2","/dev/ttyUSB1"};
    int com_num = 1;
    // for (uint8_t i = 0; i < com_num; i++)
    // {
    //     hand_sdk.qiangnao_port[i] = hand_sdk.open_com(&com_list[i]);
    //     printf("port %d:%s\n", hand_sdk.qiangnao_port[i], com_list[i]);
    //     printf("Open COM port:%d\n", hand_sdk.qiangnao_port[i]);
    // }
    // for (uint8_t i = 0; i < com_num; i++)
    // {
    //     int8_t position[6] = {80, 80, 80, 80, 80, 80};
    //     hand_sdk.set_motor_position(hand_sdk.qiangnao_port[i], position);
    // }

    // sleep(1);

    // for (uint8_t i = 0; i < com_num; i++)
    // {
        // int8_t position[6] = {0, 0, 0, 0, 0, 0};
        // int8_t position_[6] = {80, 80, 80, 80, 80, 80};
        // hand_sdk.send_position(hand_sdk.qiangnao_port[0], position_);
        // sleep(1);
        // hand_sdk.send_position(hand_sdk.qiangnao_port[0], position);
        // sleep(1);
        // hand_sdk.send_position(hand_sdk.qiangnao_port[1], position_);
        // sleep(1);
        // hand_sdk.send_position(hand_sdk.qiangnao_port[1], position);
        // sleep(1);
    // }
    for (uint8_t i = 0; i < com_num; i++)
    {
        hand_sdk.close_com(hand_sdk.qiangnao_port[i]);
        printf("Close COM port:%d\n", hand_sdk.qiangnao_port[i]);
    }
    return 0;
}
