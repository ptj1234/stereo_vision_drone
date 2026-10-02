#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <traj_utils/msg/multi_bsplines.hpp>
#include <traj_utils/msg/bspline.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include <plan_env/param_helper.hpp>

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#define PORT 8080
#define UDP_PORT 8081
#define BUF_LEN 1048576    // 1MB
#define BUF_LEN_SHORT 1024 // 1KB

using namespace std;

int send_sock_, server_fd_, recv_sock_, udp_server_fd_, udp_send_fd_;
rclcpp::Node::SharedPtr node_;
rclcpp::Subscription<traj_utils::msg::MultiBsplines>::SharedPtr swarm_trajs_sub_;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr other_odoms_sub_;
rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr emergency_stop_sub_;
rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr one_traj_sub_;
rclcpp::Publisher<traj_utils::msg::MultiBsplines>::SharedPtr swarm_trajs_pub_;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr other_odoms_pub_;
rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr emergency_stop_pub_;
rclcpp::Publisher<traj_utils::msg::Bspline>::SharedPtr one_traj_pub_;
string tcp_ip_, udp_ip_;
int drone_id_;
double odom_broadcast_freq_;
char send_buf_[BUF_LEN], recv_buf_[BUF_LEN], udp_recv_buf_[BUF_LEN], udp_send_buf_[BUF_LEN];
struct sockaddr_in addr_udp_send_;
traj_utils::msg::MultiBsplines::SharedPtr bsplines_msg_;
nav_msgs::msg::Odometry::SharedPtr odom_msg_;
std_msgs::msg::Empty::SharedPtr stop_msg_;
traj_utils::msg::Bspline::SharedPtr bspline_msg_;

enum MESSAGE_TYPE
{
  ODOM = 888,
  MULTI_TRAJ,
  ONE_TRAJ,
  STOP
} massage_type_;

// ROS 1 Time::toSec()
double stampToSec(const builtin_interfaces::msg::Time &stamp)
{
  return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
}

// ROS 1 Time::fromSec(): split into sec/nsec before scaling so nanoseconds stay exact
builtin_interfaces::msg::Time secToStamp(double t)
{
  builtin_interfaces::msg::Time stamp;
  double sec = std::floor(t);
  int64_t nsec = std::llround((t - sec) * 1e9);
  if (nsec >= 1000000000LL)
  {
    sec += 1;
    nsec -= 1000000000LL;
  }
  stamp.sec = static_cast<int32_t>(sec);
  stamp.nanosec = static_cast<uint32_t>(nsec);
  return stamp;
}

int connect_to_next_drone(const char *ip, const int port)
{
  /* Connect */
  int sock = 0;
  struct sockaddr_in serv_addr;
  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
    printf("\n Socket creation error \n");
    return -1;
  }

  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(port);

  // Convert IPv4 and IPv6 addresses from text to binary form
  if (inet_pton(AF_INET, ip, &serv_addr.sin_addr) <= 0)
  {
    printf("\nInvalid address/ Address not supported \n");
    return -1;
  }

  if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
  {
    RCLCPP_WARN(node_->get_logger(), "Tcp connection to drone_%d Failed", drone_id_+1);
    return -1;
  }

  char str[INET_ADDRSTRLEN];
  RCLCPP_INFO(node_->get_logger(), "Connect to %s success!", inet_ntop(AF_INET, &serv_addr.sin_addr, str, sizeof(str)));

  return sock;
}

int init_broadcast(const char *ip, const int port)
{
  int fd;

  if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "[bridge_node]Socket sender creation error!");
    exit(EXIT_FAILURE);
  }

  int so_broadcast = 1;
  if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &so_broadcast, sizeof(so_broadcast)) < 0)
  {
    cout << "Error in setting Broadcast option";
    exit(EXIT_FAILURE);
  }

  addr_udp_send_.sin_family = AF_INET;
  addr_udp_send_.sin_port = htons(port);

  if (inet_pton(AF_INET, ip, &addr_udp_send_.sin_addr) <= 0)
  {
    printf("\nInvalid address/ Address not supported \n");
    return -1;
  }

  return fd;
}

int wait_connection_from_previous_drone(const int port, int &server_fd, int &new_socket)
{
  struct sockaddr_in address;
  int opt = 1;
  int addrlen = sizeof(address);

  // Creating socket file descriptor
  if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
  {
    perror("socket failed");
    exit(EXIT_FAILURE);
  }

  // Forcefully attaching socket to the port
  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                 &opt, sizeof(opt)))
  {
    perror("setsockopt");
    exit(EXIT_FAILURE);
  }
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(PORT);

  // Forcefully attaching socket to the port
  if (bind(server_fd, (struct sockaddr *)&address,
           sizeof(address)) < 0)
  {
    perror("bind failed");
    exit(EXIT_FAILURE);
  }
  if (listen(server_fd, 3) < 0)
  {
    perror("listen");
    exit(EXIT_FAILURE);
  }
  if ((new_socket = accept(server_fd, (struct sockaddr *)&address,
                           (socklen_t *)&addrlen)) < 0)
  {
    perror("accept");
    exit(EXIT_FAILURE);
  }

  char str[INET_ADDRSTRLEN];
  RCLCPP_INFO(node_->get_logger(),  "Receive tcp connection from %s", inet_ntop(AF_INET, &address.sin_addr, str, sizeof(str)) );

  return new_socket;
}

int udp_bind_to_port(const int port, int &server_fd)
{
  struct sockaddr_in address;
  int opt = 1;

  // Creating socket file descriptor
  if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) == 0)
  {
    perror("socket failed");
    exit(EXIT_FAILURE);
  }

  // Forcefully attaching socket to the port
  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                 &opt, sizeof(opt)))
  {
    perror("setsockopt");
    exit(EXIT_FAILURE);
  }
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(port);

  // Forcefully attaching socket to the port
  if (bind(server_fd, (struct sockaddr *)&address,
           sizeof(address)) < 0)
  {
    perror("bind failed");
    exit(EXIT_FAILURE);
  }

  return server_fd;
}

int serializeMultiBsplines(const traj_utils::msg::MultiBsplines::ConstSharedPtr &msg)
{
  char *ptr = send_buf_;

  unsigned long total_len = 0;
  total_len += sizeof(MESSAGE_TYPE) + sizeof(int32_t) + sizeof(size_t);
  for (size_t i = 0; i < msg->traj.size(); i++)
  {
    total_len += sizeof(int32_t) + sizeof(int32_t) + sizeof(double) + sizeof(int64_t) + sizeof(double);
    total_len += sizeof(size_t) + msg->traj[i].knots.size() * sizeof(double);
    total_len += sizeof(size_t) + (3 * msg->traj[i].pos_pts.size()) * sizeof(double);
    total_len += sizeof(size_t) + msg->traj[i].yaw_pts.size() * sizeof(double);
  }
  if (total_len + 1 > BUF_LEN)
  {
    RCLCPP_ERROR(node_->get_logger(), "[bridge_node] Topic is too large, please enlarge BUF_LEN");
    return -1;
  }

  *((MESSAGE_TYPE *)ptr) = MESSAGE_TYPE::MULTI_TRAJ;
  ptr += sizeof(MESSAGE_TYPE);

  *((int32_t *)ptr) = msg->drone_id_from;
  ptr += sizeof(int32_t);
  if (ptr - send_buf_ > BUF_LEN)
  {
  }
  *((size_t *)ptr) = msg->traj.size();
  ptr += sizeof(size_t);
  for (size_t i = 0; i < msg->traj.size(); i++)
  {
    *((int32_t *)ptr) = msg->traj[i].drone_id;
    ptr += sizeof(int32_t);
    *((int32_t *)ptr) = msg->traj[i].order;
    ptr += sizeof(int32_t);
    *((double *)ptr) = stampToSec(msg->traj[i].start_time);
    ptr += sizeof(double);
    *((int64_t *)ptr) = msg->traj[i].traj_id;
    ptr += sizeof(int64_t);
    *((double *)ptr) = msg->traj[i].yaw_dt;
    ptr += sizeof(double);

    *((size_t *)ptr) = msg->traj[i].knots.size();
    ptr += sizeof(size_t);
    for (size_t j = 0; j < msg->traj[i].knots.size(); j++)
    {
      *((double *)ptr) = msg->traj[i].knots[j];
      ptr += sizeof(double);
    }

    *((size_t *)ptr) = msg->traj[i].pos_pts.size();
    ptr += sizeof(size_t);
    for (size_t j = 0; j < msg->traj[i].pos_pts.size(); j++)
    {
      *((double *)ptr) = msg->traj[i].pos_pts[j].x;
      ptr += sizeof(double);
      *((double *)ptr) = msg->traj[i].pos_pts[j].y;
      ptr += sizeof(double);
      *((double *)ptr) = msg->traj[i].pos_pts[j].z;
      ptr += sizeof(double);
    }

    *((size_t *)ptr) = msg->traj[i].yaw_pts.size();
    ptr += sizeof(size_t);
    for (size_t j = 0; j < msg->traj[i].yaw_pts.size(); j++)
    {
      *((double *)ptr) = msg->traj[i].yaw_pts[j];
      ptr += sizeof(double);
    }
  }

  return ptr - send_buf_;
}

int serializeOdom(const nav_msgs::msg::Odometry::SharedPtr &msg)
{
  char *ptr = udp_send_buf_;

  unsigned long total_len = 0;
  total_len = sizeof(size_t) +
              msg->child_frame_id.length() * sizeof(char) +
              sizeof(size_t) +
              msg->header.frame_id.length() * sizeof(char) +
              sizeof(uint32_t) +
              sizeof(double) +
              7 * sizeof(double) +
              36 * sizeof(double) +
              6 * sizeof(double) +
              36 * sizeof(double);

  if (total_len + 1 > BUF_LEN)
  {
    RCLCPP_ERROR(node_->get_logger(), "[bridge_node] Topic is too large, please enlarge BUF_LEN");
    return -1;
  }

  *((MESSAGE_TYPE *)ptr) = MESSAGE_TYPE::ODOM;
  ptr += sizeof(MESSAGE_TYPE);

  // child_frame_id
  size_t len = msg->child_frame_id.length();
  *((size_t *)ptr) = len;
  ptr += sizeof(size_t);
  memcpy((void *)ptr, (void *)msg->child_frame_id.c_str(), len * sizeof(char));
  ptr += len * sizeof(char);

  // header
  len = msg->header.frame_id.length();
  *((size_t *)ptr) = len;
  ptr += sizeof(size_t);
  memcpy((void *)ptr, (void *)msg->header.frame_id.c_str(), len * sizeof(char));
  ptr += len * sizeof(char);
  *((uint32_t *)ptr) = 0; // ROS 1 header.seq slot, kept for wire compatibility
  ptr += sizeof(uint32_t);
  *((double *)ptr) = stampToSec(msg->header.stamp);
  ptr += sizeof(double);

  *((double *)ptr) = msg->pose.pose.position.x;
  ptr += sizeof(double);
  *((double *)ptr) = msg->pose.pose.position.y;
  ptr += sizeof(double);
  *((double *)ptr) = msg->pose.pose.position.z;
  ptr += sizeof(double);

  *((double *)ptr) = msg->pose.pose.orientation.w;
  ptr += sizeof(double);
  *((double *)ptr) = msg->pose.pose.orientation.x;
  ptr += sizeof(double);
  *((double *)ptr) = msg->pose.pose.orientation.y;
  ptr += sizeof(double);
  *((double *)ptr) = msg->pose.pose.orientation.z;
  ptr += sizeof(double);

  for (size_t j = 0; j < 36; j++)
  {
    *((double *)ptr) = msg->pose.covariance[j];
    ptr += sizeof(double);
  }

  *((double *)ptr) = msg->twist.twist.linear.x;
  ptr += sizeof(double);
  *((double *)ptr) = msg->twist.twist.linear.y;
  ptr += sizeof(double);
  *((double *)ptr) = msg->twist.twist.linear.z;
  ptr += sizeof(double);
  *((double *)ptr) = msg->twist.twist.angular.x;
  ptr += sizeof(double);
  *((double *)ptr) = msg->twist.twist.angular.y;
  ptr += sizeof(double);
  *((double *)ptr) = msg->twist.twist.angular.z;
  ptr += sizeof(double);

  for (size_t j = 0; j < 36; j++)
  {
    *((double *)ptr) = msg->twist.covariance[j];
    ptr += sizeof(double);
  }

  return ptr - udp_send_buf_;
}

int serializeStop(const std_msgs::msg::Empty::ConstSharedPtr &msg)
{
  char *ptr = udp_send_buf_;

  *((MESSAGE_TYPE *)ptr) = MESSAGE_TYPE::STOP;
  ptr += sizeof(MESSAGE_TYPE);

  return ptr - udp_send_buf_;
}

int serializeOneTraj(const traj_utils::msg::Bspline::ConstSharedPtr &msg)
{
  char *ptr = udp_send_buf_;

  unsigned long total_len = 0;
  total_len += sizeof(int32_t) + sizeof(int32_t) + sizeof(double) + sizeof(int64_t) + sizeof(double);
  total_len += sizeof(size_t) + msg->knots.size() * sizeof(double);
  total_len += sizeof(size_t) + (3 * msg->pos_pts.size()) * sizeof(double);
  total_len += sizeof(size_t) + msg->yaw_pts.size() * sizeof(double);
  if (total_len + 1 > BUF_LEN)
  {
    RCLCPP_ERROR(node_->get_logger(), "[bridge_node] Topic is too large, please enlarge BUF_LEN (2)");
    return -1;
  }

  *((MESSAGE_TYPE *)ptr) = MESSAGE_TYPE::ONE_TRAJ;
  ptr += sizeof(MESSAGE_TYPE);

  *((int32_t *)ptr) = msg->drone_id;
  ptr += sizeof(int32_t);
  *((int32_t *)ptr) = msg->order;
  ptr += sizeof(int32_t);
  *((double *)ptr) = stampToSec(msg->start_time);
  ptr += sizeof(double);
  *((int64_t *)ptr) = msg->traj_id;
  ptr += sizeof(int64_t);
  *((double *)ptr) = msg->yaw_dt;
  ptr += sizeof(double);

  *((size_t *)ptr) = msg->knots.size();
  ptr += sizeof(size_t);
  for (size_t j = 0; j < msg->knots.size(); j++)
  {
    *((double *)ptr) = msg->knots[j];
    ptr += sizeof(double);
  }

  *((size_t *)ptr) = msg->pos_pts.size();
  ptr += sizeof(size_t);
  for (size_t j = 0; j < msg->pos_pts.size(); j++)
  {
    *((double *)ptr) = msg->pos_pts[j].x;
    ptr += sizeof(double);
    *((double *)ptr) = msg->pos_pts[j].y;
    ptr += sizeof(double);
    *((double *)ptr) = msg->pos_pts[j].z;
    ptr += sizeof(double);
  }

  *((size_t *)ptr) = msg->yaw_pts.size();
  ptr += sizeof(size_t);
  for (size_t j = 0; j < msg->yaw_pts.size(); j++)
  {
    *((double *)ptr) = msg->yaw_pts[j];
    ptr += sizeof(double);
  }

  return ptr - udp_send_buf_;
}

int deserializeOneTraj(traj_utils::msg::Bspline::SharedPtr &msg)
{
  char *ptr = udp_recv_buf_;

  ptr += sizeof(MESSAGE_TYPE);

  msg->drone_id = *((int32_t *)ptr);
  ptr += sizeof(int32_t);
  msg->order = *((int32_t *)ptr);
  ptr += sizeof(int32_t);
  msg->start_time = secToStamp(*((double *)ptr));
  ptr += sizeof(double);
  msg->traj_id = *((int64_t *)ptr);
  ptr += sizeof(int64_t);
  msg->yaw_dt = *((double *)ptr);
  ptr += sizeof(double);
  msg->knots.resize(*((size_t *)ptr));
  ptr += sizeof(size_t);
  for (size_t j = 0; j < msg->knots.size(); j++)
  {
    msg->knots[j] = *((double *)ptr);
    ptr += sizeof(double);
  }

  msg->pos_pts.resize(*((size_t *)ptr));
  ptr += sizeof(size_t);
  for (size_t j = 0; j < msg->pos_pts.size(); j++)
  {
    msg->pos_pts[j].x = *((double *)ptr);
    ptr += sizeof(double);
    msg->pos_pts[j].y = *((double *)ptr);
    ptr += sizeof(double);
    msg->pos_pts[j].z = *((double *)ptr);
    ptr += sizeof(double);
  }

  msg->yaw_pts.resize(*((size_t *)ptr));
  ptr += sizeof(size_t);
  for (size_t j = 0; j < msg->yaw_pts.size(); j++)
  {
    msg->yaw_pts[j] = *((double *)ptr);
    ptr += sizeof(double);
  }

  return ptr - udp_recv_buf_;
}

int deserializeStop(std_msgs::msg::Empty::SharedPtr &msg)
{
  char *ptr = udp_recv_buf_;

  return ptr - udp_recv_buf_;
}

int deserializeOdom(nav_msgs::msg::Odometry::SharedPtr &msg)
{
  char *ptr = udp_recv_buf_;

  ptr += sizeof(MESSAGE_TYPE);

  // child_frame_id
  size_t len = *((size_t *)ptr);
  ptr += sizeof(size_t);
  msg->child_frame_id.assign((const char *)ptr, len);
  ptr += len * sizeof(char);

  // header
  len = *((size_t *)ptr);
  ptr += sizeof(size_t);
  msg->header.frame_id.assign((const char *)ptr, len);
  ptr += len * sizeof(char);
  ptr += sizeof(uint32_t); // ROS 1 header.seq slot (no seq in ROS 2)
  msg->header.stamp = secToStamp(*((double *)ptr));
  ptr += sizeof(double);

  msg->pose.pose.position.x = *((double *)ptr);
  ptr += sizeof(double);
  msg->pose.pose.position.y = *((double *)ptr);
  ptr += sizeof(double);
  msg->pose.pose.position.z = *((double *)ptr);
  ptr += sizeof(double);

  msg->pose.pose.orientation.w = *((double *)ptr);
  ptr += sizeof(double);
  msg->pose.pose.orientation.x = *((double *)ptr);
  ptr += sizeof(double);
  msg->pose.pose.orientation.y = *((double *)ptr);
  ptr += sizeof(double);
  msg->pose.pose.orientation.z = *((double *)ptr);
  ptr += sizeof(double);

  for (size_t j = 0; j < 36; j++)
  {
    msg->pose.covariance[j] = *((double *)ptr);
    ptr += sizeof(double);
  }

  msg->twist.twist.linear.x = *((double *)ptr);
  ptr += sizeof(double);
  msg->twist.twist.linear.y = *((double *)ptr);
  ptr += sizeof(double);
  msg->twist.twist.linear.z = *((double *)ptr);
  ptr += sizeof(double);
  msg->twist.twist.angular.x = *((double *)ptr);
  ptr += sizeof(double);
  msg->twist.twist.angular.y = *((double *)ptr);
  ptr += sizeof(double);
  msg->twist.twist.angular.z = *((double *)ptr);
  ptr += sizeof(double);

  for (size_t j = 0; j < 36; j++)
  {
    msg->twist.covariance[j] = *((double *)ptr);
    ptr += sizeof(double);
  }

  return ptr - udp_recv_buf_;
}

int deserializeMultiBsplines(traj_utils::msg::MultiBsplines::SharedPtr &msg)
{
  char *ptr = recv_buf_;

  ptr += sizeof(MESSAGE_TYPE);

  msg->drone_id_from = *((int32_t *)ptr);
  ptr += sizeof(int32_t);
  msg->traj.resize(*((size_t *)ptr));
  ptr += sizeof(size_t);
  for (size_t i = 0; i < msg->traj.size(); i++)
  {
    msg->traj[i].drone_id = *((int32_t *)ptr);
    ptr += sizeof(int32_t);
    msg->traj[i].order = *((int32_t *)ptr);
    ptr += sizeof(int32_t);
    msg->traj[i].start_time = secToStamp(*((double *)ptr));
    ptr += sizeof(double);
    msg->traj[i].traj_id = *((int64_t *)ptr);
    ptr += sizeof(int64_t);
    msg->traj[i].yaw_dt = *((double *)ptr);
    ptr += sizeof(double);

    msg->traj[i].knots.resize(*((size_t *)ptr));
    ptr += sizeof(size_t);
    for (size_t j = 0; j < msg->traj[i].knots.size(); j++)
    {
      msg->traj[i].knots[j] = *((double *)ptr);
      ptr += sizeof(double);
    }

    msg->traj[i].pos_pts.resize(*((size_t *)ptr));
    ptr += sizeof(size_t);
    for (size_t j = 0; j < msg->traj[i].pos_pts.size(); j++)
    {
      msg->traj[i].pos_pts[j].x = *((double *)ptr);
      ptr += sizeof(double);
      msg->traj[i].pos_pts[j].y = *((double *)ptr);
      ptr += sizeof(double);
      msg->traj[i].pos_pts[j].z = *((double *)ptr);
      ptr += sizeof(double);
    }

    msg->traj[i].yaw_pts.resize(*((size_t *)ptr));
    ptr += sizeof(size_t);
    for (size_t j = 0; j < msg->traj[i].yaw_pts.size(); j++)
    {
      msg->traj[i].yaw_pts[j] = *((double *)ptr);
      ptr += sizeof(double);
    }
  }

  return ptr - recv_buf_;
}

void multitraj_sub_tcp_cb(const traj_utils::msg::MultiBsplines::ConstSharedPtr msg)
{
  int len = serializeMultiBsplines(msg);
  if (send(send_sock_, send_buf_, len, 0) <= 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "TCP SEND ERROR!!!");
  }
}

void odom_sub_udp_cb(const nav_msgs::msg::Odometry::ConstSharedPtr in_msg)
{

  static rclcpp::Time t_last{0, 0, RCL_ROS_TIME};
  rclcpp::Time t_now = node_->now();
  if ((t_now - t_last).seconds() * odom_broadcast_freq_ < 1.0)
  {
    return;
  }
  t_last = t_now;

  auto msg = std::make_shared<nav_msgs::msg::Odometry>(*in_msg);
  msg->child_frame_id = string("drone_") + std::to_string(drone_id_);

  int len = serializeOdom(msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "UDP SEND ERROR (1)!!!");
  }
}

void emergency_stop_sub_udp_cb(const std_msgs::msg::Empty::ConstSharedPtr msg)
{

  int len = serializeStop(msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "UDP SEND ERROR (2)!!!");
  }
}

void one_traj_sub_udp_cb(const traj_utils::msg::Bspline::ConstSharedPtr msg)
{

  int len = serializeOneTraj(msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "UDP SEND ERROR (3)!!!");
  }
}

void server_fun()
{
  int valread;

  // Connect
  if (wait_connection_from_previous_drone(PORT, server_fd_, recv_sock_) < 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "[bridge_node]Socket recever creation error!");
    exit(EXIT_FAILURE);
  }

  while (true)
  {
    valread = read(recv_sock_, recv_buf_, BUF_LEN);

    if ( valread <= 0 )
    {
      RCLCPP_ERROR(node_->get_logger(), "Received message length <= 0, maybe connection has lost");
      close(recv_sock_);
      close(server_fd_);
      return;
    }

    if (valread == deserializeMultiBsplines(bsplines_msg_))
    {
      swarm_trajs_pub_->publish(*bsplines_msg_);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Received message length not matches the sent one!!!");
      continue;
    }
  }
}

void udp_recv_fun()
{
  int valread;
  struct sockaddr_in addr_client;
  socklen_t addr_len;

  // Connect
  if (udp_bind_to_port(UDP_PORT, udp_server_fd_) < 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "[bridge_node]Socket recever creation error!");
    exit(EXIT_FAILURE);
  }

  while (true)
  {
    if ((valread = recvfrom(udp_server_fd_, udp_recv_buf_, BUF_LEN, 0, (struct sockaddr *)&addr_client, (socklen_t *)&addr_len)) < 0)
    {
      perror("recvfrom error:");
      exit(EXIT_FAILURE);
    }

    char *ptr = udp_recv_buf_;
    switch (*((MESSAGE_TYPE *)ptr))
    {
    case MESSAGE_TYPE::STOP:
    {

      if (valread == sizeof(std_msgs::msg::Empty))
      {
        if (valread == deserializeStop(stop_msg_))
        {
          if (emergency_stop_pub_)
            emergency_stop_pub_->publish(*stop_msg_);
        }
        else
        {
          RCLCPP_ERROR(node_->get_logger(), "Received message length not matches the sent one (1)!!!");
          continue;
        }
      }

      break;
    }

    case MESSAGE_TYPE::ODOM:
    {
      if (valread == deserializeOdom(odom_msg_))
      {
        other_odoms_pub_->publish(*odom_msg_);
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Received message length not matches the sent one (2)!!!");
        continue;
      }

      break;
    }

    case MESSAGE_TYPE::ONE_TRAJ:
    {

      if ( valread == deserializeOneTraj(bspline_msg_) )
      {
        one_traj_pub_->publish(*bspline_msg_);
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Received message length not matches the sent one (3)!!!");
        continue;
      }

      break;
    }

    default:

      //RCLCPP_ERROR(node_->get_logger(), "Unknown received message???");

      break;
    }
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  node_ = rclcpp::Node::make_shared("rosmsg_tcp_bridge");
  auto &nh = node_;

  param_helper::param(nh, "next_drone_ip", tcp_ip_, string("127.0.0.1"));
  param_helper::param(nh, "broadcast_ip", udp_ip_, string("127.0.0.255"));
  param_helper::param(nh, "drone_id", drone_id_, -1);
  param_helper::param(nh, "odom_max_freq", odom_broadcast_freq_, 1000.0);

  bsplines_msg_ = std::make_shared<traj_utils::msg::MultiBsplines>();
  odom_msg_ = std::make_shared<nav_msgs::msg::Odometry>();
  stop_msg_ = std::make_shared<std_msgs::msg::Empty>();
  bspline_msg_ = std::make_shared<traj_utils::msg::Bspline>();

  if (drone_id_ == -1)
  {
    RCLCPP_ERROR(node_->get_logger(), "Wrong drone_id!");
    exit(EXIT_FAILURE);
  }

  string sub_traj_topic_name = string("/drone_") + std::to_string(drone_id_) + string("_planning/swarm_trajs");
  swarm_trajs_sub_ = nh->create_subscription<traj_utils::msg::MultiBsplines>(sub_traj_topic_name, 10, multitraj_sub_tcp_cb);

  if ( drone_id_ >= 1 )
  {
    string pub_traj_topic_name = string("/drone_") + std::to_string(drone_id_ - 1) + string("_planning/swarm_trajs");
    swarm_trajs_pub_ = nh->create_publisher<traj_utils::msg::MultiBsplines>(pub_traj_topic_name, 10);
  }

  other_odoms_sub_ = nh->create_subscription<nav_msgs::msg::Odometry>("~/my_odom", 10, odom_sub_udp_cb);
  other_odoms_pub_ = nh->create_publisher<nav_msgs::msg::Odometry>("/others_odom", 10);

  //emergency_stop_sub_ = nh->create_subscription<std_msgs::msg::Empty>("~/emergency_stop_broadcast", 10, emergency_stop_sub_udp_cb);
  //emergency_stop_pub_ = nh->create_publisher<std_msgs::msg::Empty>("~/emergency_stop_recv", 10);

  one_traj_sub_ = nh->create_subscription<traj_utils::msg::Bspline>("/broadcast_bspline", 100, one_traj_sub_udp_cb);
  one_traj_pub_ = nh->create_publisher<traj_utils::msg::Bspline>("/broadcast_bspline2", 100);

  std::thread recv_thd(server_fun);
  recv_thd.detach();
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  std::thread udp_recv_thd(udp_recv_fun);
  udp_recv_thd.detach();
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  // TCP connect
  send_sock_ = connect_to_next_drone(tcp_ip_.c_str(), PORT);

  // UDP connect
  udp_send_fd_ = init_broadcast(udp_ip_.c_str(), UDP_PORT);

  cout << "[rosmsg_tcp_bridge] start running" << endl;

  rclcpp::spin(node_);

  close(send_sock_);
  close(recv_sock_);
  close(server_fd_);
  close(udp_server_fd_);
  close(udp_send_fd_);

  rclcpp::shutdown();
  return 0;
}
