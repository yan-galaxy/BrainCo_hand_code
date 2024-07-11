#include <hand_sdk.h>

#ifdef DEBUG
#define DEBUG_LOG(...) printf(__VA_ARGS__)
#else
#define DEBUG_LOG(...)
#endif

#define HEADER_SIZE 4
#define HEADER_INDEX 0
#define DST_SIZE 1
#define DST_INDEX 4
#define SRC_SIZE 1
#define SRC_INDEX 5
#define LENGTH_SIZE 2
#define LENGTH_INDEX 6
#define PAYLOAD_INDEX 8
#define CRC32_SIZE 4

#define MAIN_ADDR 1
#define MOTOR_ADDR 2
#define APP_ADDR 3

#define MOTOR_NUM 6

#define PARAM_ERR -1
#define MEM_ERR -2
#define FUNC_ERR -3
#define DELAY_MS 1 // 线程延迟时间，单位为毫秒
bool flag_position = false;
bool flag_speed = false;
bool flag_current = false;
bool stopThread_ = false;
bool target_update = false;
void* thread_function(void* arg) {
    // stopThread_ = false;
    while (!stopThread_)
    {
        if (target_update == true)
        {
            int8_t left[6];
            int8_t right[6];
            if (flag_position)
            {
                for (size_t i = 0; i < 6; i++)
                {
                    pthread_mutex_lock(&hand_sdk.mutex);
                    left[i] = hand_sdk.left_value[i];
                    right[i] = hand_sdk.right_value[i];
                    pthread_mutex_unlock(&hand_sdk.mutex);
                }
                hand_sdk.set_motor_position(hand_sdk.qiangnao_port[0], left);
                hand_sdk.set_motor_position(hand_sdk.qiangnao_port[1], right);
            }
            else if (flag_speed)
            {
                for (size_t i = 0; i < 6; i++)
                {
                    pthread_mutex_lock(&hand_sdk.mutex);
                    left[i] = hand_sdk.left_value[i];
                    right[i] = hand_sdk.right_value[i];
                    pthread_mutex_unlock(&hand_sdk.mutex);
                }
                hand_sdk.set_motor_speed(hand_sdk.qiangnao_port[0],left);
                hand_sdk.set_motor_speed(hand_sdk.qiangnao_port[1],right);
            }
            else if (flag_current)
            {
                for (size_t i = 0; i < 6; i++)
                {
                    pthread_mutex_lock(&hand_sdk.mutex);
                    left[i] = hand_sdk.left_value[i];
                    right[i] = hand_sdk.right_value[i];
                    pthread_mutex_unlock(&hand_sdk.mutex);
                }
                hand_sdk.set_motor_current(hand_sdk.qiangnao_port[0],left);
                hand_sdk.set_motor_current(hand_sdk.qiangnao_port[1],right);
            }
            target_update = false;
        }
        usleep(DELAY_MS * 5);
    }
    return NULL;
}
static void position_reset(int port)
{
    int8_t position[6] = {0, 100, 0, 0, 0, 0};//大拇指内收 大拇指外翻  食指  中指  小拇指
    hand_sdk.set_motor_position(port, position);
}
static int init_impl()
{
    hand_sdk.qiangnao_port[0] = 0;
    hand_sdk.qiangnao_port[1] = 0;
    flag_position = false;
    flag_speed = false;
    flag_current = false;
    target_update = false;
    // char *com_list[] = {"/dev/stark_serial_L","/dev/stark_serial_R"};
    char *com_list[] = {"/dev/ttyUSB0"," "};
    
    int com_num = 1;
    pthread_t thread_id; // 用于存储新线程的 ID

    // 创建新线程，传入线程函数和参数（这里不传入参数，所以传入 NULL）
    if (pthread_create(&thread_id, NULL, thread_function, NULL) != 0) {
        fprintf(stderr, "Error creating thread\n");
        return 0;
    }

    hand_sdk.qiangnao_port[0] = hand_sdk.open_com(&com_list[0]);
    printf("port %d:%s\n", hand_sdk.qiangnao_port[0], com_list[0]);
    printf("Open COM port:%d\n", hand_sdk.qiangnao_port[0]);

    position_reset(hand_sdk.qiangnao_port[0]);
    sleep(1);
    {
        int8_t position[6] = {0, 100, 0, 0, 0, 0};//大拇指内收 大拇指外翻  食指  中指  小拇指
        int8_t speed[6] = {0, 0, 20, 0, 0, 0};//大拇指内收 大拇指外翻  食指  中指  小拇指
        int8_t current[6] = {15, 0, 30, 20, 20, 20};//大拇指内收 大拇指外翻  食指  中指  小拇指
        // hand_sdk.set_motor_position(hand_sdk.qiangnao_port[0], position);
        // hand_sdk.set_motor_speed(hand_sdk.qiangnao_port[0], speed);
        hand_sdk.set_motor_current(hand_sdk.qiangnao_port[0], current);
    }
    sleep(5);
    {
        int8_t position[6] = {0, 100, 0, 0, 0, 0};
        int8_t speed[6] = {0, 0, 0, 0, 0, 0};//大拇指内收 大拇指外翻  食指  中指  小拇指
        int8_t current[6] = {0, 0, 0, 0, 0, 0};//大拇指内收 大拇指外翻  食指  中指  小拇指
        hand_sdk.set_motor_position(hand_sdk.qiangnao_port[0], position);
        // hand_sdk.set_motor_speed(hand_sdk.qiangnao_port[0], speed);
        // hand_sdk.set_motor_current(hand_sdk.qiangnao_port[0], current);
    }
    
    




    /*
    {
        int8_t position[6] = {0, 50, 0, 0, 0, 0};//大拇指内收 大拇指外翻  食指  中指  小拇指
        int8_t speed[6] = {0, 0, 0, 0, 0, 0};//大拇指内收 大拇指外翻  食指  中指  小拇指
        int8_t current[6] = {0, 0, 0, 0, 0, 0};//大拇指内收 大拇指外翻  食指  中指  小拇指
        hand_sdk.set_motor_position(hand_sdk.qiangnao_port[0], position);
        // hand_sdk.set_motor_speed(hand_sdk.qiangnao_port[0], speed);
        // hand_sdk.set_motor_speed(hand_sdk.qiangnao_port[0], current);
    }
    sleep(1);
    {
        int8_t position[6] = {0, 50, 50, 0, 0, 0};//大拇指内收 大拇指外翻  食指  中指  小拇指
        int8_t speed[6] = {0, 0, 0, 0, 0, 0};//大拇指内收 大拇指外翻  食指  中指  小拇指
        int8_t current[6] = {0, 0, 0, 0, 0, 0};//大拇指内收 大拇指外翻  食指  中指  小拇指
        hand_sdk.set_motor_position(hand_sdk.qiangnao_port[0], position);
        // hand_sdk.set_motor_speed(hand_sdk.qiangnao_port[0], speed);
        // hand_sdk.set_motor_speed(hand_sdk.qiangnao_port[0], current);
    }
    sleep(1);
    {
        int8_t position[6] = {0, 50, 50, 50, 0, 0};//大拇指内收 大拇指外翻  食指  中指  小拇指
        int8_t speed[6] = {0, 0, 0, 0, 0, 0};//大拇指内收 大拇指外翻  食指  中指  小拇指
        int8_t current[6] = {0, 0, 0, 0, 0, 0};//大拇指内收 大拇指外翻  食指  中指  小拇指
        hand_sdk.set_motor_position(hand_sdk.qiangnao_port[0], position);
        // hand_sdk.set_motor_speed(hand_sdk.qiangnao_port[0], speed);
        // hand_sdk.set_motor_speed(hand_sdk.qiangnao_port[0], current);
    }
    sleep(1);
    {
        int8_t position[6] = {0, 50, 50, 50, 50, 0};//大拇指内收 大拇指外翻  食指  中指  小拇指
        int8_t speed[6] = {0, 0, 0, 0, 0, 0};//大拇指内收 大拇指外翻  食指  中指  小拇指
        int8_t current[6] = {0, 0, 0, 0, 0, 0};//大拇指内收 大拇指外翻  食指  中指  小拇指
        hand_sdk.set_motor_position(hand_sdk.qiangnao_port[0], position);
        // hand_sdk.set_motor_speed(hand_sdk.qiangnao_port[0], speed);
        // hand_sdk.set_motor_speed(hand_sdk.qiangnao_port[0], current);
    }
    sleep(1);
    {
        int8_t position[6] = {0, 50, 50, 50, 50, 50};//大拇指内收 大拇指外翻  食指  中指  小拇指
        int8_t speed[6] = {0, 0, 0, 0, 0, 0};//大拇指内收 大拇指外翻  食指  中指  小拇指
        int8_t current[6] = {0, 0, 0, 0, 0, 0};//大拇指内收 大拇指外翻  食指  中指  小拇指
        hand_sdk.set_motor_position(hand_sdk.qiangnao_port[0], position);
        // hand_sdk.set_motor_speed(hand_sdk.qiangnao_port[0], speed);
        // hand_sdk.set_motor_speed(hand_sdk.qiangnao_port[0], current);
    }
    sleep(1);
    {
        int8_t position[6] = {0, 0, 0, 0, 0, 0};//大拇指内收 大拇指外翻  食指  中指  小拇指
        int8_t speed[6] = {0, 0, 0, 0, 0, 0};//大拇指内收 大拇指外翻  食指  中指  小拇指
        int8_t current[6] = {0, 0, 0, 0, 0, 0};//大拇指内收 大拇指外翻  食指  中指  小拇指
        hand_sdk.set_motor_position(hand_sdk.qiangnao_port[0], position);
        // hand_sdk.set_motor_speed(hand_sdk.qiangnao_port[0], speed);
        // hand_sdk.set_motor_speed(hand_sdk.qiangnao_port[0], current);
    }
    sleep(1);
    */
    return -1;

    printf("Thread finished\n");
}
static int scan_com_impl(char ***com_list)
{
    int ret;
    glob_t glob_info;
    char **com_items;

    if (com_list == NULL)
    {
        return PARAM_ERR;
    }

    memset(&glob_info, 0, sizeof(glob_info));

    ret = glob("/dev/ttyUSB*", GLOB_TILDE, NULL, &glob_info);

    if (ret != 0)
    {
        DEBUG_LOG("Error %i from glob: %s\n", errno, strerror(errno));
        return FUNC_ERR;
    }

    com_items = (char **)malloc(glob_info.gl_pathc * sizeof(char *));
    if (com_items == NULL)
    {
        DEBUG_LOG("Memory allocation error!\n");
        return MEM_ERR;
    }

    for (int i = 0; i < glob_info.gl_pathc; i++)
    {
        com_items[i] = (char *)malloc(strlen(glob_info.gl_pathv[i]) * sizeof(char));
        if (com_items[i] == NULL)
        {
            DEBUG_LOG("Memory allocation error!\n");
            for (int j = 0; j < i; j++)
            {
                free(com_items[j]);
            }

            free(com_items);
            return MEM_ERR;
        }

        strcpy(com_items[i], glob_info.gl_pathv[i]);
    }

    *com_list = com_items;

    ret = glob_info.gl_pathc;

    globfree(&glob_info);

    return ret;
}

static int open_com_impl(char **com_list)
{
    if (com_list == NULL)
    {
        return PARAM_ERR;
    }

    int port = open(*com_list, O_RDWR);
    struct termios tty;

    if (port < 0)
    {
        DEBUG_LOG("Error %i from open: %s\n", errno, strerror(errno));
        return FUNC_ERR;
    }

    if (tcgetattr(port, &tty) != 0)
    {
        DEBUG_LOG("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return FUNC_ERR;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;

    if (tcsetattr(port, TCSAFLUSH, &tty) != 0)
    {
        DEBUG_LOG("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        close(port);
        return FUNC_ERR;
    }

    return port;
}

static int close_com_impl(int port)
{
    stopThread_ = true;
    pthread_mutex_destroy(&hand_sdk.mutex);
    int ret;

    ret = close(port);
    if (ret < 0)
    {
        DEBUG_LOG("Error %i from close: %s\n", errno, strerror(errno));
        return FUNC_ERR;
    }

    return 0;
}

static void send_com_impl(int port, uint8_t *buf, uint32_t len)
{
    write(port, buf, len);
}

static int build_frame_impl(uint8_t **frame, uint8_t dst, uint8_t src, uint8_t *protobuf, uint8_t protobuf_size)
{
    uint8_t frame_size;
    uint8_t *frame_buf;
    uint8_t padding_size;
    uint32_t crc32;

    padding_size = (4 - protobuf_size % 4) % 4;

    frame_size = HEADER_SIZE + DST_SIZE + SRC_SIZE + LENGTH_SIZE + protobuf_size + padding_size + CRC32_SIZE;

    frame_buf = (uint8_t *)calloc(frame_size, sizeof(uint8_t));
    if (frame_buf == NULL)
    {
        DEBUG_LOG("Memory allocation error!\n");
        return MEM_ERR;
    }

    memcpy(frame_buf, "BnCP", HEADER_SIZE);

    frame_buf[DST_INDEX] = dst;
    frame_buf[SRC_INDEX] = src;

    frame_buf[LENGTH_INDEX] = protobuf_size >> 8 & 0xFF;
    frame_buf[LENGTH_INDEX + 1] = protobuf_size & 0xFF;

    memcpy(frame_buf + PAYLOAD_INDEX, protobuf, protobuf_size);

    crc32 = softCRC_CRC32(frame_buf, (frame_size - CRC32_SIZE) / 4, 0xffffffff, 0);

    frame_buf[frame_size - CRC32_SIZE] = crc32 >> 24 & 0xFF;
    frame_buf[frame_size - CRC32_SIZE + 1] = crc32 >> 16 & 0xFF;
    frame_buf[frame_size - CRC32_SIZE + 2] = crc32 >> 8 & 0xFF;
    frame_buf[frame_size - CRC32_SIZE + 3] = crc32 & 0xFF;

    *frame = frame_buf;

    return frame_size;
}

static int subscribe_motor_impl(int port, FingerStatusSampleRate sample_rate)
{
    uint8_t *frame;
    uint8_t frame_size;

    uint8_t *protobuf;
    size_t protobuf_size;

    UMainToMtr main_msg = U__MAIN_TO_MTR__INIT;
    MtrCmd mtr_cmd = MTR_CMD__INIT;

    mtr_cmd.ctrl_mode = MTR_CMD__CTRL_MODE__SUBSCRIBE;
    mtr_cmd.sample_rate = sample_rate;
    main_msg.mtr_cmd = &mtr_cmd;

    protobuf_size = u__main_to_mtr__get_packed_size(&main_msg);
    protobuf = (uint8_t *)malloc(protobuf_size);
    if (protobuf == NULL)
    {
        DEBUG_LOG("Memory allocation error!\n");
        return MEM_ERR;
    }

    u__main_to_mtr__pack(&main_msg, (uint8_t *)protobuf);

    frame_size = build_frame_impl(&frame, MOTOR_ADDR, MAIN_ADDR, protobuf, protobuf_size);

    send_com_impl(port, frame, frame_size);

    free(protobuf);
    free(frame);

    return frame_size;
}

static int unsubscribe_motor_impl(int port)
{
    uint8_t *frame;
    uint8_t frame_size;

    uint8_t *protobuf;
    size_t protobuf_size;

    UMainToMtr main_msg = U__MAIN_TO_MTR__INIT;
    MtrCmd mtr_cmd = MTR_CMD__INIT;

    mtr_cmd.ctrl_mode = MTR_CMD__CTRL_MODE__SUBSCRIBE;
    mtr_cmd.sample_rate = FINGER_STATUS_SAMPLE_RATE__RATE_OFF;
    main_msg.mtr_cmd = &mtr_cmd;

    protobuf_size = u__main_to_mtr__get_packed_size(&main_msg);
    protobuf = (uint8_t *)malloc(protobuf_size);
    if (protobuf == NULL)
    {
        DEBUG_LOG("Memory allocation error!\n");
        return MEM_ERR;
    }

    u__main_to_mtr__pack(&main_msg, (uint8_t *)protobuf);

    frame_size = build_frame_impl(&frame, MOTOR_ADDR, MAIN_ADDR, protobuf, protobuf_size);

    send_com_impl(port, frame, frame_size);

    free(protobuf);
    free(frame);

    return frame_size;
}

static int set_motor_position_impl(int port, int8_t *pos)
{
    uint8_t *frame;
    uint8_t frame_size;

    uint8_t *protobuf;
    size_t protobuf_size;

    UMainToMtr main_msg = U__MAIN_TO_MTR__INIT;
    MtrCmd mtr_cmd = MTR_CMD__INIT;
    FingerStatus finger_status = FINGER_STATUS__INIT;

    main_msg.mtr_cmd = &mtr_cmd;
    mtr_cmd.ctrl_mode = MTR_CMD__CTRL_MODE__POSITION;
    mtr_cmd.expect_status = &finger_status;
    finger_status.positions.data = pos;
    finger_status.positions.len = MOTOR_NUM;

    protobuf_size = u__main_to_mtr__get_packed_size(&main_msg);
    protobuf = (uint8_t *)malloc(protobuf_size);
    if (protobuf == NULL)
    {
        DEBUG_LOG("Memory allocation error!\n");
        return MEM_ERR;
    }

    u__main_to_mtr__pack(&main_msg, (uint8_t *)protobuf);

    frame_size = build_frame_impl(&frame, MOTOR_ADDR, MAIN_ADDR, protobuf, protobuf_size);

    send_com_impl(port, frame, frame_size);

    free(protobuf);
    free(frame);

    return frame_size;
}

static int set_motor_speed_impl(int port, int8_t *speed)
{
    uint8_t *frame;
    uint8_t frame_size;

    uint8_t *protobuf;
    size_t protobuf_size;

    UMainToMtr main_msg = U__MAIN_TO_MTR__INIT;
    MtrCmd mtr_cmd = MTR_CMD__INIT;
    FingerStatus finger_status = FINGER_STATUS__INIT;

    main_msg.mtr_cmd = &mtr_cmd;
    mtr_cmd.ctrl_mode = MTR_CMD__CTRL_MODE__SPEED;
    mtr_cmd.expect_status = &finger_status;
    finger_status.speeds.data = speed;
    finger_status.speeds.len = MOTOR_NUM;

    protobuf_size = u__main_to_mtr__get_packed_size(&main_msg);
    protobuf = (uint8_t *)malloc(protobuf_size);
    if (protobuf == NULL)
    {
        DEBUG_LOG("Memory allocation error!\n");
        return MEM_ERR;
    }

    u__main_to_mtr__pack(&main_msg, (uint8_t *)protobuf);

    frame_size = build_frame_impl(&frame, MOTOR_ADDR, MAIN_ADDR, protobuf, protobuf_size);

    send_com_impl(port, frame, frame_size);

    free(protobuf);
    free(frame);

    return frame_size;
}

static int set_motor_current_impl(int port, int8_t *current)
{
    uint8_t *frame;
    uint8_t frame_size;

    uint8_t *protobuf;
    size_t protobuf_size;

    UMainToMtr main_msg = U__MAIN_TO_MTR__INIT;
    MtrCmd mtr_cmd = MTR_CMD__INIT;
    FingerStatus finger_status = FINGER_STATUS__INIT;

    main_msg.mtr_cmd = &mtr_cmd;
    mtr_cmd.ctrl_mode = MTR_CMD__CTRL_MODE__CURRENT;
    mtr_cmd.expect_status = &finger_status;
    finger_status.currents.data = current;
    finger_status.currents.len = MOTOR_NUM;

    protobuf_size = u__main_to_mtr__get_packed_size(&main_msg);
    protobuf = (uint8_t *)malloc(protobuf_size);
    if (protobuf == NULL)
    {
        DEBUG_LOG("Memory allocation error!\n");
        return MEM_ERR;
    }

    u__main_to_mtr__pack(&main_msg, (uint8_t *)protobuf);

    frame_size = build_frame_impl(&frame, MOTOR_ADDR, MAIN_ADDR, protobuf, protobuf_size);

    send_com_impl(port, frame, frame_size);

    free(protobuf);
    free(frame);

    return frame_size;
}

static void send_position_impl(int port, int8_t *pos)
{
    flag_current = false;
    flag_position = true;
    flag_speed = false;
    target_update = true;
    size_t size = 6 * sizeof(int8_t);
    if (port == hand_sdk.qiangnao_port[0])
    {
        for (size_t i = 0; i < 6; i++)
        {
            pthread_mutex_lock(&hand_sdk.mutex);
            hand_sdk.left_value[i] = pos[i];
            pthread_mutex_unlock(&hand_sdk.mutex);
        }
    }
    else
    {
        for (size_t i = 0; i < 6; i++)
        {
            pthread_mutex_lock(&hand_sdk.mutex);
            hand_sdk.right_value[i] = pos[i];
            pthread_mutex_unlock(&hand_sdk.mutex);
        }
    }
}

static void send_speed_impl(int port, int8_t *speed)
{
    flag_current = false;
    flag_position = false;
    flag_speed = true;
    target_update = true;
    if (port == hand_sdk.qiangnao_port[0])
    {
        for (size_t i = 0; i < 6; i++)
        {
            pthread_mutex_lock(&hand_sdk.mutex);
            hand_sdk.left_value[i] = speed[i];
            pthread_mutex_unlock(&hand_sdk.mutex);
        }
    }
    else
    {
        for (size_t i = 0; i < 6; i++)
        {
            pthread_mutex_lock(&hand_sdk.mutex);
            hand_sdk.right_value[i] = speed[i];
            pthread_mutex_unlock(&hand_sdk.mutex);
        }
    }
}

static void send_currents_impl(int port, int8_t *current)
{
    flag_current = true;
    flag_position = false;
    flag_speed = false;
    target_update = true;
    if (port == hand_sdk.qiangnao_port[0])
    {
        for (size_t i = 0; i < 6; i++)
        {
            pthread_mutex_lock(&hand_sdk.mutex);
            hand_sdk.left_value[i] = current[i];
            pthread_mutex_unlock(&hand_sdk.mutex);
        }
    }
    else
    {
        for (size_t i = 0; i < 6; i++)
        {
            pthread_mutex_lock(&hand_sdk.mutex);
            hand_sdk.right_value[i] = current[i];
            pthread_mutex_unlock(&hand_sdk.mutex);
        }
    }
}

HandSDK hand_sdk = {
    .init = init_impl,
    .send_position = send_position_impl,
    .send_speed = send_speed_impl,
    .send_current = send_currents_impl,
    .scan_com = scan_com_impl,
    .open_com = open_com_impl,
    .close_com = close_com_impl,
    .send_com = send_com_impl,
    .build_frame = build_frame_impl,
    .subscribe_motor = subscribe_motor_impl,
    .unsubscribe_motor = unsubscribe_motor_impl,
    .set_motor_position = set_motor_position_impl,
    .set_motor_speed = set_motor_speed_impl,
    .set_motor_current = set_motor_current_impl
};