/*
 * ROS2 Humble command bridge for iROB.
 *
 * This node subscribes to /iROB_command, converts the ROS2 message to the
 * 13-byte command packet used by app_ros_comm.ino, and sends it to ESP32 by UDP.
 */

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "irob_ros/msg/irob_command.h"
#include "rcl/error_handling.h"
#include "rcl/rcl.h"
#include "rmw/types.h"
#include "rosidl_runtime_c/message_type_support_struct.h"

/* These defaults match iROB_ros.ino after the ESP32 received DHCP IP 192.168.1.100. */
#define IROB_DEFAULT_ESP32_IP "192.168.1.100"
#define IROB_DEFAULT_UDP_PORT 6767

/* ROS2 -> ESP32 packet size. This matches app_ros_comm.ino RX size. */
#define IROB_COMMAND_PACKET_SIZE 13

/* Set to 0 by SIGINT/SIGTERM so the node can exit cleanly. */
static volatile sig_atomic_t keep_running = 1;

typedef struct {
  char esp32_ip[INET_ADDRSTRLEN];
  uint16_t port;
} irob_command_config_t;

static void handle_signal(int signal_number)
{
  (void)signal_number;
  keep_running = 0;
}

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

static bool set_ipv4_value(const char *text, char *output, size_t output_size)
{
  struct in_addr test_address;

  if (inet_pton(AF_INET, text, &test_address) != 1) {
    return false;
  }

  snprintf(output, output_size, "%s", text);
  return true;
}

static void read_config(int argc, char **argv, irob_command_config_t *config)
{
  snprintf(config->esp32_ip, sizeof(config->esp32_ip), "%s", IROB_DEFAULT_ESP32_IP);
  config->port = IROB_DEFAULT_UDP_PORT;

  const char *env_ip = getenv("IROB_ESP32_IP");
  if (env_ip != NULL && env_ip[0] != '\0') {
    if (!set_ipv4_value(env_ip, config->esp32_ip, sizeof(config->esp32_ip))) {
      fprintf(stderr, "Invalid IROB_ESP32_IP '%s', using %s\n",
        env_ip, config->esp32_ip);
    }
  }

  const char *env_port = getenv("IROB_UDP_PORT");
  if (env_port != NULL && env_port[0] != '\0') {
    if (!parse_port_value(env_port, &config->port)) {
      fprintf(stderr, "Invalid IROB_UDP_PORT '%s', using %u\n",
        env_port, (unsigned int)config->port);
    }
  }

  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--esp32-ip") == 0 && i + 1 < argc) {
      if (!set_ipv4_value(argv[i + 1], config->esp32_ip, sizeof(config->esp32_ip))) {
        fprintf(stderr, "Invalid --esp32-ip value '%s', using %s\n",
          argv[i + 1], config->esp32_ip);
      }
      ++i;
    } else if (strncmp(argv[i], "--esp32-ip=", 11) == 0) {
      if (!set_ipv4_value(argv[i] + 11, config->esp32_ip, sizeof(config->esp32_ip))) {
        fprintf(stderr, "Invalid --esp32-ip value '%s', using %s\n",
          argv[i] + 11, config->esp32_ip);
      }
    } else if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
      if (!parse_port_value(argv[i + 1], &config->port)) {
        fprintf(stderr, "Invalid --port value '%s', using %u\n",
          argv[i + 1], (unsigned int)config->port);
      }
      ++i;
    } else if (strncmp(argv[i], "--port=", 7) == 0) {
      if (!parse_port_value(argv[i] + 7, &config->port)) {
        fprintf(stderr, "Invalid --port value '%s', using %u\n",
          argv[i] + 7, (unsigned int)config->port);
      }
    }
  }
}

static int create_udp_socket(void)
{
  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    perror("socket");
    return -1;
  }

  const int broadcast = 1;
  if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0) {
    perror("setsockopt(SO_BROADCAST)");
    close(fd);
    return -1;
  }

  return fd;
}

static bool make_destination(
  const irob_command_config_t *config,
  struct sockaddr_in *destination)
{
  memset(destination, 0, sizeof(*destination));
  destination->sin_family = AF_INET;
  destination->sin_port = htons(config->port);

  if (inet_pton(AF_INET, config->esp32_ip, &destination->sin_addr) != 1) {
    fprintf(stderr, "Invalid ESP32 IP '%s'\n", config->esp32_ip);
    return false;
  }

  return true;
}

static void write_i16_le(uint8_t *bytes, int16_t value)
{
  const uint16_t unsigned_value = (uint16_t)value;
  bytes[0] = (uint8_t)(unsigned_value & 0xFFU);
  bytes[1] = (uint8_t)((unsigned_value >> 8) & 0xFFU);
}

static void command_to_packet(
  const irob_ros__msg__IROBCommand *msg,
  uint8_t *packet)
{
  packet[0] = 'R';
  packet[1] = 'B';
  packet[2] = msg->reg;
  packet[3] = msg->ctk;

  write_i16_le(&packet[4], msg->motor1_ctrl);
  write_i16_le(&packet[6], msg->motor2_ctrl);
  write_i16_le(&packet[8], msg->motor3_ctrl);
  write_i16_le(&packet[10], msg->motor4_ctrl);

  packet[12] = msg->cmd_data_pc;
}

static bool send_command_packet(
  int socket_fd,
  const struct sockaddr_in *destination,
  const irob_ros__msg__IROBCommand *msg)
{
  uint8_t packet[IROB_COMMAND_PACKET_SIZE];
  command_to_packet(msg, packet);

  const ssize_t sent = sendto(
    socket_fd,
    packet,
    sizeof(packet),
    0,
    (const struct sockaddr *)destination,
    sizeof(*destination));

  if (sent != (ssize_t)sizeof(packet)) {
    if (sent < 0) {
      perror("sendto");
    } else {
      fprintf(stderr, "sendto sent %zd bytes; expected %u\n",
        sent, (unsigned int)sizeof(packet));
    }
    return false;
  }

  return true;
}

int main(int argc, char **argv)
{
  irob_command_config_t config;
  read_config(argc, argv, &config);

  int exit_code = EXIT_FAILURE;
  int socket_fd = -1;
  bool init_options_ready = false;
  bool context_ready = false;
  bool node_ready = false;
  bool subscription_ready = false;
  bool wait_set_ready = false;
  bool msg_ready = false;
  struct sockaddr_in destination;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_context_t context = rcl_get_zero_initialized_context();
  rcl_node_t node = rcl_get_zero_initialized_node();
  rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  irob_ros__msg__IROBCommand msg;

  signal(SIGINT, handle_signal);
  signal(SIGTERM, handle_signal);

  if (!make_destination(&config, &destination)) {
    goto cleanup;
  }

  socket_fd = create_udp_socket();
  if (socket_fd < 0) {
    goto cleanup;
  }

  if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) {
    fprintf(stderr, "rcl_init_options_init failed: %s\n",
      rcl_get_error_string().str);
    rcl_reset_error();
    goto cleanup;
  }
  init_options_ready = true;

  if (rcl_init(argc, argv, &init_options, &context) != RCL_RET_OK) {
    fprintf(stderr, "rcl_init failed: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
    goto cleanup;
  }
  context_ready = true;

  rcl_node_options_t node_options = rcl_node_get_default_options();
  if (rcl_node_init(&node, "iROB_CMD", "", &context, &node_options) != RCL_RET_OK) {
    fprintf(stderr, "rcl_node_init failed: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
    goto cleanup;
  }
  node_ready = true;

  const rosidl_message_type_support_t *type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(irob_ros, msg, IROBCommand);
  rcl_subscription_options_t subscription_options = rcl_subscription_get_default_options();
  if (rcl_subscription_init(
      &subscription, &node, type_support, "iROB_command", &subscription_options) != RCL_RET_OK) {
    fprintf(stderr, "rcl_subscription_init failed: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
    goto cleanup;
  }
  subscription_ready = true;

  if (rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, 0, &context, allocator) != RCL_RET_OK) {
    fprintf(stderr, "rcl_wait_set_init failed: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
    goto cleanup;
  }
  wait_set_ready = true;

  if (!irob_ros__msg__IROBCommand__init(&msg)) {
    fprintf(stderr, "IROBCommand message init failed\n");
    goto cleanup;
  }
  msg_ready = true;

  printf("iROB_CMD sending UDP commands to %s:%u\n",
    config.esp32_ip, (unsigned int)config.port);
  printf("Subscribing to topic /iROB_command\n");

  while (keep_running) {
    if (rcl_wait_set_clear(&wait_set) != RCL_RET_OK) {
      fprintf(stderr, "rcl_wait_set_clear failed: %s\n", rcl_get_error_string().str);
      rcl_reset_error();
      break;
    }

    if (rcl_wait_set_add_subscription(&wait_set, &subscription, NULL) != RCL_RET_OK) {
      fprintf(stderr, "rcl_wait_set_add_subscription failed: %s\n",
        rcl_get_error_string().str);
      rcl_reset_error();
      break;
    }

    const rcl_ret_t wait_result = rcl_wait(&wait_set, 100000000);
    if (wait_result == RCL_RET_TIMEOUT) {
      continue;
    }

    if (wait_result != RCL_RET_OK) {
      fprintf(stderr, "rcl_wait failed: %s\n", rcl_get_error_string().str);
      rcl_reset_error();
      break;
    }

    if (wait_set.subscriptions[0] == NULL) {
      continue;
    }

    while (keep_running) {
      rmw_message_info_t message_info;
      const rcl_ret_t take_result = rcl_take(&subscription, &msg, &message_info, NULL);

      if (take_result == RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
        break;
      }

      if (take_result != RCL_RET_OK) {
        fprintf(stderr, "rcl_take failed: %s\n", rcl_get_error_string().str);
        rcl_reset_error();
        break;
      }

      send_command_packet(socket_fd, &destination, &msg);
    }
  }

  exit_code = EXIT_SUCCESS;

cleanup:
  if (msg_ready) {
    irob_ros__msg__IROBCommand__fini(&msg);
  }

  if (wait_set_ready) {
    rcl_wait_set_fini(&wait_set);
  }

  if (subscription_ready) {
    rcl_subscription_fini(&subscription, &node);
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

  if (socket_fd >= 0) {
    close(socket_fd);
  }

  return exit_code;
}
