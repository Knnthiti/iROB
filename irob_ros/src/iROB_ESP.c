/*
 * ROS2 Humble UDP bridge for iROB.
 *
 * This node listens for the 32-byte ESP32 WiFi packet, converts it to
 * irob_ros/msg/IROBController, and publishes it on /iROB_controller.
 */

/* POSIX socket and process-control headers used by the UDP receiver. */
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

/* Generated ROS2 message header from msg/IROBController.msg. */
#include "irob_ros/msg/irob_controller.h"

/* Low-level C client library for ROS2. */
#include "rcl/error_handling.h"
#include "rcl/rcl.h"
#include "rosidl_runtime_c/message_type_support_struct.h"

/* Default UDP port must match IROB_UDP_PORT in iROB_ros.ino. */
#define IROB_DEFAULT_UDP_PORT 6767

/* ESP32 feedback packet size. This matches app_ros_comm.ino TX size. */
#define IROB_PACKET_SIZE 32

/* Set to 0 by SIGINT/SIGTERM so the receive loop can exit cleanly. */
static volatile sig_atomic_t keep_running = 1;

/* Signal handler for Ctrl+C and process shutdown. */
static void handle_signal(int signal_number)
{
  (void)signal_number;
  keep_running = 0;
}

/* Parse a UDP port from text and reject invalid or out-of-range values. */
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
 * Select the UDP listen port.
 * Priority:
 * 1. Default port 6767
 * 2. IROB_UDP_PORT environment variable
 * 3. --port or --port=<value> command-line argument
 */
static uint16_t read_udp_port(int argc, char **argv)
{
  uint16_t port = IROB_DEFAULT_UDP_PORT;
  const char *env_port = getenv("IROB_UDP_PORT");

  /* Environment variable override, useful with ros2 run. */
  if (env_port != NULL && env_port[0] != '\0') {
    if (!parse_port_value(env_port, &port)) {
      fprintf(stderr, "Invalid IROB_UDP_PORT '%s', using %u\n",
        env_port, (unsigned int)port);
    }
  }

  /* Command-line override for direct testing. */
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

/* Create, configure, and bind the UDP socket used to receive ESP32 packets. */
static int create_udp_socket(uint16_t port)
{
  /* AF_INET/SOCK_DGRAM is IPv4 UDP. */
  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    perror("socket");
    return -1;
  }

  /* Allow quick restart of the node without waiting for the OS to release the port. */
  const int reuse = 1;
  if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
    perror("setsockopt(SO_REUSEADDR)");
    close(fd);
    return -1;
  }

  struct sockaddr_in address;
  memset(&address, 0, sizeof(address));
  address.sin_family = AF_INET;

  /* Listen on every local interface, including the WiFi interface. */
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  address.sin_port = htons(port);

  /* Attach the socket to the requested UDP port. */
  if (bind(fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
    perror("bind");
    close(fd);
    return -1;
  }

  return fd;
}

/* Convert two little-endian bytes from the ESP32 packet into a signed int16. */
static int16_t read_i16_le(const uint8_t *bytes)
{
  const uint16_t value = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
  return (int16_t)value;
}

/* Calculate XOR checksum over bytes 0..30. Byte 31 stores the checksum. */
static uint8_t checksum_xor(const uint8_t *bytes)
{
  uint8_t checksum = 0;

  for (size_t i = 0; i < IROB_PACKET_SIZE - 1; ++i) {
    checksum ^= bytes[i];
  }

  return checksum;
}

/* Decode one valid 32-byte binary packet into a ROS2 message struct. */
static void packet_to_msg(const uint8_t *packet, irob_ros__msg__IROBController *msg)
{
  /* Header and command/status byte. */
  msg->header0 = packet[0];
  msg->header1 = packet[1];
  msg->cmd_data_mcu = packet[2];

  /* Motor feedback. Order is LF, LB, RB, RF to match app_ros_comm.ino. */
  msg->motor1_fb = read_i16_le(&packet[3]);
  msg->motor2_fb = read_i16_le(&packet[5]);
  msg->motor3_fb = read_i16_le(&packet[7]);
  msg->motor4_fb = read_i16_le(&packet[9]);

  /* Mouse or optical-flow velocity fields. */
  msg->mouse_x_vel = (int8_t)packet[11];
  msg->mouse_y_vel = (int8_t)packet[12];

  /* Raw gyroscope fields. */
  msg->gyro_x_raw = read_i16_le(&packet[13]);
  msg->gyro_y_raw = read_i16_le(&packet[15]);
  msg->gyro_z_raw = read_i16_le(&packet[17]);

  /* Raw magnetometer fields. */
  msg->mag_x_raw = read_i16_le(&packet[19]);
  msg->mag_y_raw = read_i16_le(&packet[21]);
  msg->mag_z_raw = read_i16_le(&packet[23]);

  /* Raw accelerometer fields. */
  msg->acc_x_raw = read_i16_le(&packet[25]);
  msg->acc_y_raw = read_i16_le(&packet[27]);
  msg->acc_z_raw = read_i16_le(&packet[29]);

  /* Received checksum byte. */
  msg->cks = packet[31];
}

int main(int argc, char **argv)
{
  /* Read runtime configuration before ROS initialization. */
  const uint16_t udp_port = read_udp_port(argc, argv);

  /* Cleanup flags let one cleanup block safely release only initialized resources. */
  int exit_code = EXIT_FAILURE;
  int socket_fd = -1;
  bool init_options_ready = false;
  bool context_ready = false;
  bool node_ready = false;
  bool publisher_ready = false;
  bool msg_ready = false;

  /* Allow Ctrl+C to stop the blocking receive loop gracefully. */
  signal(SIGINT, handle_signal);
  signal(SIGTERM, handle_signal);

  /* ROS2 objects are zero-initialized before passing them to rcl init functions. */
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_context_t context = rcl_get_zero_initialized_context();
  rcl_node_t node = rcl_get_zero_initialized_node();
  rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
  irob_ros__msg__IROBController msg;

  /* Initialize the ROS2 context options. */
  if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) {
    fprintf(stderr, "rcl_init_options_init failed: %s\n",
      rcl_get_error_string().str);
    rcl_reset_error();
    goto cleanup;
  }
  init_options_ready = true;

  /* Initialize rcl with argc/argv so standard ROS arguments still work. */
  if (rcl_init(argc, argv, &init_options, &context) != RCL_RET_OK) {
    fprintf(stderr, "rcl_init failed: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
    goto cleanup;
  }
  context_ready = true;

  /* Create the ROS2 node named iROB_ESP. */
  rcl_node_options_t node_options = rcl_node_get_default_options();
  if (rcl_node_init(&node, "iROB_ESP", "", &context, &node_options) != RCL_RET_OK) {
    fprintf(stderr, "rcl_node_init failed: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
    goto cleanup;
  }
  node_ready = true;

  /* Create the publisher for /iROB_controller using the generated message type. */
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

  /* Initialize the reusable ROS message object. */
  if (!irob_ros__msg__IROBController__init(&msg)) {
    fprintf(stderr, "IROBController message init failed\n");
    goto cleanup;
  }
  msg_ready = true;

  /* Start listening for UDP packets after ROS objects are ready. */
  socket_fd = create_udp_socket(udp_port);
  if (socket_fd < 0) {
    goto cleanup;
  }

  printf("iROB_ESP listening on UDP 0.0.0.0:%u\n", (unsigned int)udp_port);
  printf("Publishing packet data to topic /iROB_controller\n");

  uint64_t bad_size_count = 0;
  uint64_t bad_header_count = 0;
  uint64_t bad_checksum_count = 0;

  /* Main receive/publish loop. */
  while (keep_running) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(socket_fd, &read_fds);

    /* select() timeout lets the loop wake up and notice shutdown signals. */
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;

    /* Wait until the UDP socket has data or the timeout expires. */
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

    /* Read exactly one UDP datagram. UDP preserves packet boundaries. */
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

    /* Ignore packets that do not match the expected binary protocol size. */
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

    /* The ESP32 packet must start with 'J', 'B'. */
    if (packet[0] != 'J' || packet[1] != 'B') {
      if ((bad_header_count++ % 50U) == 0U) {
        fprintf(stderr, "Ignoring packet with bad header 0x%02X 0x%02X\n",
          packet[0], packet[1]);
      }
      continue;
    }

    /* Convert binary data to named ROS2 message fields. */
    packet_to_msg(packet, &msg);

    /*
     * Checksum compatibility rule:
     * - 0 means "not used", matching the older serial code that never filled cks.
     * - non-zero checksum must match the XOR checksum.
     */
    const uint8_t expected_checksum = checksum_xor(packet);
    if (msg.cks != 0U && msg.cks != expected_checksum) {
      if ((bad_checksum_count++ % 50U) == 0U) {
        fprintf(stderr, "Packet checksum mismatch: got 0x%02X expected 0x%02X\n",
          msg.cks, expected_checksum);
      }
    }

    /* Publish the latest packet to ROS2. */
    if (rcl_publish(&publisher, &msg, NULL) != RCL_RET_OK) {
      fprintf(stderr, "rcl_publish failed: %s\n", rcl_get_error_string().str);
      rcl_reset_error();
    }
  }

  exit_code = EXIT_SUCCESS;

cleanup:
  /* Release resources in reverse initialization order. */
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
