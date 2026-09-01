/*
 * ROS2 Humble UDP bridge สำหรับ iROB
 *
 * หน้าที่ของ node นี้:
 * 1. รอรับ feedback packet ขนาด 32 bytes จาก ESP32 ผ่าน UDP
 * 2. แปลง binary packet เป็น message irob_ros/msg/IROBController
 * 3. publish message ไปที่ topic /iROB_controller
 */

/* header สำหรับ POSIX socket และการจัดการ process signal */
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

/* header ของ ROS2 message ที่ generate จาก msg/IROBController.msg */
#include "irob_ros/msg/irob_controller.h"

/* rcl คือ ROS2 client library ระดับ C */
#include "rcl/error_handling.h"
#include "rcl/rcl.h"
#include "rosidl_runtime_c/message_type_support_struct.h"

/* UDP port ค่าเริ่มต้น ต้องตรงกับ IROB_UDP_PORT ใน iROB_ros.ino */
#define IROB_DEFAULT_UDP_PORT 6767

/* ขนาด feedback packet จาก ESP32 ต้องตรงกับส่วน TX ของ app_ros_comm.ino */
#define IROB_PACKET_SIZE 32

/* flag สำหรับควบคุม main loop: SIGINT/SIGTERM จะเปลี่ยนเป็น 0 เพื่อออกจาก loop */
static volatile sig_atomic_t keep_running = 1;

/* callback สำหรับรับ Ctrl+C หรือ signal ปิด process */
static void handle_signal(int signal_number)
{
  (void)signal_number;
  keep_running = 0;
}

/* แปลง port จาก string เป็น uint16_t และกันค่าที่ไม่ถูกต้อง */
static bool parse_port_value(const char *text, uint16_t *port)
{
  char *end = NULL;
  const long value = strtol(text, &end, 10);

  if (text == end || *end != '\0' || value <= 0 || value > 65535) {
    return false;
  }

  *port = (uint16_t)value;
  return true;
}

/*
 * อ่านค่า UDP port สำหรับ node นี้
 * ลำดับความสำคัญ:
 * 1. ค่า default 6767
 * 2. environment variable IROB_UDP_PORT
 * 3. argument --port หรือ --port=<value>
 */
static uint16_t read_udp_port(int argc, char **argv)
{
  uint16_t port = IROB_DEFAULT_UDP_PORT;
  const char *env_port = getenv("IROB_UDP_PORT");

  /* ใช้ env override ได้ สะดวกตอน ros2 run โดยไม่ต้องแก้ source */
  if (env_port != NULL && env_port[0] != '\0') {
    if (!parse_port_value(env_port, &port)) {
      fprintf(stderr, "Invalid IROB_UDP_PORT '%s', using %u\n",
        env_port, (unsigned int)port);
    }
  }

  /* ใช้ command-line override ได้ทั้งตอนทดสอบตรงและตอน launch */
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
      if (!parse_port_value(argv[i + 1], &port)) {
        fprintf(stderr, "Invalid --port value '%s', using %u\n",
          argv[i + 1], (unsigned int)port);
      }
      ++i;
    } else if (strncmp(argv[i], "--port=", 7) == 0) {
      if (!parse_port_value(argv[i] + 7, &port)) {
        fprintf(stderr, "Invalid --port value '%s', using %u\n",
          argv[i] + 7, (unsigned int)port);
      }
    }
  }

  return port;
}

/* สร้าง UDP socket และ bind กับ port ที่ใช้รับ feedback จาก ESP32 */
static int create_udp_socket(uint16_t port)
{
  /* AF_INET + SOCK_DGRAM คือ IPv4 UDP */
  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    perror("socket");
    return -1;
  }

  /* ทำให้ restart node ได้เร็ว โดยไม่ติด port เดิมที่ OS ยังถืออยู่ชั่วคราว */
  const int reuse = 1;
  if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
    perror("setsockopt(SO_REUSEADDR)");
    close(fd);
    return -1;
  }

  struct sockaddr_in address;
  memset(&address, 0, sizeof(address));
  address.sin_family = AF_INET;

  /* รับ packet จากทุก network interface รวมถึง WiFi */
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  address.sin_port = htons(port);

  /* ผูก socket กับ port ที่ต้องการฟัง */
  if (bind(fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
    perror("bind");
    close(fd);
    return -1;
  }

  return fd;
}

/* แปลง byte little-endian 2 byte จาก ESP32 packet เป็น int16_t */
static int16_t read_i16_le(const uint8_t *bytes)
{
  const uint16_t value = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
  return (int16_t)value;
}

/* คำนวณ XOR checksum จาก byte 0..30 โดย byte 31 คือ checksum ที่รับมา */
static uint8_t checksum_xor(const uint8_t *bytes)
{
  uint8_t checksum = 0;

  for (size_t i = 0; i < IROB_PACKET_SIZE - 1; ++i) {
    checksum ^= bytes[i];
  }

  return checksum;
}

/* ถอดรหัส binary packet ขนาด 32 bytes ไปเป็น ROS2 message */
static void packet_to_msg(const uint8_t *packet, irob_ros__msg__IROBController *msg)
{
  /* header และสถานะจาก MCU */
  msg->header0 = packet[0];
  msg->header1 = packet[1];
  msg->cmd_data_mcu = packet[2];

  /* feedback motor เรียงลำดับ LF, LB, RB, RF ตาม app_ros_comm.ino */
  msg->motor1_fb = read_i16_le(&packet[3]);
  msg->motor2_fb = read_i16_le(&packet[5]);
  msg->motor3_fb = read_i16_le(&packet[7]);
  msg->motor4_fb = read_i16_le(&packet[9]);

  /* ค่า mouse/optical-flow velocity ถ้ามี sensor ส่งมา */
  msg->mouse_x_vel = (int8_t)packet[11];
  msg->mouse_y_vel = (int8_t)packet[12];

  /* ค่า gyro raw */
  msg->gyro_x_raw = read_i16_le(&packet[13]);
  msg->gyro_y_raw = read_i16_le(&packet[15]);
  msg->gyro_z_raw = read_i16_le(&packet[17]);

  /* ค่า magnetometer raw */
  msg->mag_x_raw = read_i16_le(&packet[19]);
  msg->mag_y_raw = read_i16_le(&packet[21]);
  msg->mag_z_raw = read_i16_le(&packet[23]);

  /* ค่า accelerometer raw */
  msg->acc_x_raw = read_i16_le(&packet[25]);
  msg->acc_y_raw = read_i16_le(&packet[27]);
  msg->acc_z_raw = read_i16_le(&packet[29]);

  /* checksum ที่มากับ packet */
  msg->cks = packet[31];
}

int main(int argc, char **argv)
{
  /* อ่าน config ก่อน init ROS2 */
  const uint16_t udp_port = read_udp_port(argc, argv);

  /* flag เหล่านี้ช่วยให้ cleanup ปล่อยเฉพาะ resource ที่ init สำเร็จแล้ว */
  int exit_code = EXIT_FAILURE;
  int socket_fd = -1;
  bool init_options_ready = false;
  bool context_ready = false;
  bool node_ready = false;
  bool publisher_ready = false;
  bool msg_ready = false;

  /* ให้ Ctrl+C หยุด receive loop ได้แบบเรียบร้อย */
  signal(SIGINT, handle_signal);
  signal(SIGTERM, handle_signal);

  /* object ของ ROS2 ต้อง zero-initialize ก่อนส่งเข้า rcl init function */
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_context_t context = rcl_get_zero_initialized_context();
  rcl_node_t node = rcl_get_zero_initialized_node();
  rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
  irob_ros__msg__IROBController msg;

  /* init options ของ ROS2 context */
  if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) {
    fprintf(stderr, "rcl_init_options_init failed: %s\n",
      rcl_get_error_string().str);
    rcl_reset_error();
    goto cleanup;
  }
  init_options_ready = true;

  /* init rcl พร้อม argc/argv เพื่อให้ ROS argument มาตรฐานยังใช้งานได้ */
  if (rcl_init(argc, argv, &init_options, &context) != RCL_RET_OK) {
    fprintf(stderr, "rcl_init failed: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
    goto cleanup;
  }
  context_ready = true;

  /* สร้าง ROS2 node ชื่อ iROB_ESP */
  rcl_node_options_t node_options = rcl_node_get_default_options();
  if (rcl_node_init(&node, "iROB_ESP", "", &context, &node_options) != RCL_RET_OK) {
    fprintf(stderr, "rcl_node_init failed: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
    goto cleanup;
  }
  node_ready = true;

  /* สร้าง publisher ของ topic /iROB_controller โดยใช้ message type ที่ generate จาก .msg */
  const rosidl_message_type_support_t *type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(irob_ros, msg, IROBController);
  rcl_publisher_options_t publisher_options = rcl_publisher_get_default_options();
  if (rcl_publisher_init(
      &publisher, &node, type_support, "iROB_controller", &publisher_options) != RCL_RET_OK) {
    fprintf(stderr, "rcl_publisher_init failed: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
    goto cleanup;
  }
  publisher_ready = true;

  /* init message object ที่จะใช้ซ้ำทุกครั้งก่อน publish */
  if (!irob_ros__msg__IROBController__init(&msg)) {
    fprintf(stderr, "IROBController message init failed\n");
    goto cleanup;
  }
  msg_ready = true;

  /* เริ่มเปิด UDP socket หลังจาก ROS2 พร้อม publish แล้ว */
  socket_fd = create_udp_socket(udp_port);
  if (socket_fd < 0) {
    goto cleanup;
  }

  printf("iROB_ESP listening on UDP 0.0.0.0:%u\n", (unsigned int)udp_port);
  printf("Publishing packet data to topic /iROB_controller\n");

  uint64_t bad_size_count = 0;
  uint64_t bad_header_count = 0;
  uint64_t bad_checksum_count = 0;

  /* loop หลัก: รับ UDP packet -> ตรวจ packet -> แปลงเป็น ROS2 message -> publish */
  while (keep_running) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(socket_fd, &read_fds);

    /* timeout ของ select() ทำให้ loop ตื่นมาตรวจ signal ปิดโปรแกรมได้ */
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;

    /* รอจนกว่า socket มีข้อมูล หรือ timeout */
    const int select_result = select(socket_fd + 1, &read_fds, NULL, NULL, &timeout);
    if (select_result < 0) {
      if (errno == EINTR) {
        continue;
      }
      perror("select");
      break;
    }

    if (select_result == 0) {
      continue;
    }

    /* อ่าน UDP datagram หนึ่งก้อน เพราะ UDP รักษาขอบเขต packet ให้อยู่แล้ว */
    uint8_t packet[IROB_PACKET_SIZE];
    struct sockaddr_in sender;
    socklen_t sender_length = sizeof(sender);
    const ssize_t bytes_received = recvfrom(
      socket_fd,
      packet,
      sizeof(packet),
      0,
      (struct sockaddr *)&sender,
      &sender_length);

    if (bytes_received < 0) {
      if (errno == EINTR) {
        continue;
      }
      perror("recvfrom");
      break;
    }

    /* ทิ้ง packet ที่ขนาดไม่ตรงกับ protocol */
    if (bytes_received != IROB_PACKET_SIZE) {
      if ((bad_size_count++ % 50U) == 0U) {
        char sender_ip[INET_ADDRSTRLEN] = {0};
        inet_ntop(AF_INET, &sender.sin_addr, sender_ip, sizeof(sender_ip));
        fprintf(stderr, "Ignoring UDP packet from %s:%u with %zd bytes; expected %u\n",
          sender_ip,
          (unsigned int)ntohs(sender.sin_port),
          bytes_received,
          (unsigned int)IROB_PACKET_SIZE);
      }
      continue;
    }

    /* feedback packet จาก ESP32 ต้องขึ้นต้นด้วย 'J', 'B' */
    if (packet[0] != 'J' || packet[1] != 'B') {
      if ((bad_header_count++ % 50U) == 0U) {
        fprintf(stderr, "Ignoring packet with bad header 0x%02X 0x%02X\n",
          packet[0], packet[1]);
      }
      continue;
    }

    /* แปลง binary packet ให้เป็น field ใน ROS2 message */
    packet_to_msg(packet, &msg);

    /*
     * กฎตรวจ checksum เพื่อให้เข้ากับ protocol เดิม:
     * - ถ้า cks เป็น 0 หมายถึงไม่ใช้ checksum
     * - ถ้า cks ไม่เป็น 0 ต้องตรงกับ XOR checksum
     */
    const uint8_t expected_checksum = checksum_xor(packet);
    if (msg.cks != 0U && msg.cks != expected_checksum) {
      if ((bad_checksum_count++ % 50U) == 0U) {
        fprintf(stderr, "Packet checksum mismatch: got 0x%02X expected 0x%02X\n",
          msg.cks, expected_checksum);
      }
    }

    /* publish feedback ล่าสุดเข้า ROS2 topic /iROB_controller */
    if (rcl_publish(&publisher, &msg, NULL) != RCL_RET_OK) {
      fprintf(stderr, "rcl_publish failed: %s\n", rcl_get_error_string().str);
      rcl_reset_error();
    }
  }

  exit_code = EXIT_SUCCESS;

cleanup:
  /* ปล่อย resource ย้อนลำดับจากที่ init */
  if (socket_fd >= 0) {
    close(socket_fd);
  }

  if (msg_ready) {
    irob_ros__msg__IROBController__fini(&msg);
  }

  if (publisher_ready) {
    rcl_publisher_fini(&publisher, &node);
  }

  if (node_ready) {
    rcl_node_fini(&node);
  }

  if (context_ready) {
    rcl_shutdown(&context);
    rcl_context_fini(&context);
  }

  if (init_options_ready) {
    rcl_init_options_fini(&init_options);
  }

  return exit_code;
}
