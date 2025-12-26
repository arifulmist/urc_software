#include <Arduino.h>
#include <FlexCAN_T4.h>
#ifdef CAN_ERROR_BUS_OFF
#undef CAN_ERROR_BUS_OFF
#endif
#include "ODriveCAN.h"
#include "ODriveFlexCAN.hpp"
 
//ros 2 include
#include <micro_ros_platformio.h>
 
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
 
#include <std_msgs/msg/string.h>
#include<bits/stdc++.h>
 
 
#define CAN_BAUDRATE   1000000
#define ODRV0_NODE_ID0 0  // motor/controller with ID 0
#define ODRV0_NODE_ID1 1  // motor/controller with ID 1

enum states{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

 
// CAN interface on Teensy 4.x (same as in your original code)
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can_intf;
 
// Single ODrive on CAN
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID0);
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV0_NODE_ID1);
 
 
// ROS entities
rcl_subscription_t subscriber;
rcl_publisher_t publisher;

 std_msgs__msg__String msgss;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;


#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            error_loop();            \
        }                            \
    }


void error_loop()
{
    while (1)
    {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);
    }
}



// Callback function
void subscription_callback(const void * msgin)
{
    //publish to 
    // msg.data.data 
    // std_msgs__msg__String tmp ;

    // tmp.data.data = (char*) malloc(256);
    //         tmp.data.size = 0;
    //         tmp.data.capacity = 256;

    // strcpy(tmp.data.data,"heello");
    // tmp.data.size = 6;
    // rcl_publish(&publisher, &tmp, NULL);


  const std_msgs__msg__String * msg =
    (const std_msgs__msg__String *)msgin;
  
//   //data check
  bool wrong_data = false;
  if(msg->data.data[0] != '['){
    wrong_data = true;
    // std_msgs__msg__String tmp ;

    // tmp.data.data = (char*) malloc(256);
    //         tmp.data.size = 0;
    //         tmp.data.capacity = 256;

    // strcpy(tmp.data.data,"issue ase");
    // tmp.data.size = 6;
    // rcl_publish(&publisher, &tmp, NULL);
    // Serial.println("Receiving wrong data");
  }

  //decooding incoming mag
    std::string copy_msg = msg->data.data, tmp_msg;
    std::stringstream ss(copy_msg.substr(1, copy_msg.size()-2));
    int arr[25] = {-1, -1};

    int idx = 0;
    while (std::getline(ss, tmp_msg, ',')) {
        arr[idx] = stoi(tmp_msg);
        idx++;
    }

    int left = arr[0], right = arr[1];
  if(left == -1)  left = 1500;
  if(right == -1) right = 1500;

  std::string tmpss=std::to_string(left);//debug 
  std_msgs__msg__String tmp1 ;
  tmp1.data.data=(char*)malloc(256);

  tmp1.data.size=std::strlen(tmpss.c_str());//string len
  tmp1.data.capacity=256;
  strcpy(tmp1.data.data,tmpss.c_str());//publisher jnno
  rcl_publish(&publisher, &tmp1, NULL);

  //data check before setting velocity
  if(left<1000 || left > 2000 || right < 1000 || right>2000){
    Serial.println("Receiving wrong data");
    wrong_data = true;
  }
  //setting velocity 
  if(!wrong_data){
    odrv0.setVelocity(map(left, 1000, 2000, -53, 53));
    odrv1.setVelocity(map(right, 1000, 2000, -53, 53));
  }
  else{
    odrv0.setVelocity(map(1500, 1000, 2000, -53, 53));
    odrv1.setVelocity(map(1500, 1000, 2000, -53, 53));
  }
  

//   digitalWrite(LED_BUILTIN, HIGH);
}
 


#define EXECUTE_EVERY_N_MS(MS, X)          \
    do                                     \
    {                                      \
        static volatile int64_t init = -1; \
        if (init == -1)                    \
        {                                  \
            init = millis();               \
        }                                  \
        if (millis() - init > MS)          \
        {                                  \
            X;                             \
            init = millis();               \
        }                                  \
    } while (0)

bool create_ros_entities(){
    allocator = rcl_get_default_allocator();
 
  // Init support
  //rclc_support_init(&support, 0, NULL, &allocator);
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
 
  // Create node
  //rclc_node_init_default(&node, "teensy_subscriber", "", &support);
    RCCHECK(rclc_node_init_default(&node, "teensy_subscriber", "", &support));
  
  
  
 
  // Create subscriber
//   rclc_subscription_init_default(
//         &subscriber,
//         &node,
//         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
//         "control_word"
//         );
  RCCHECK(
        rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "control_word"
        )
  );

  //create publisher
//   rclc_publisher_init_default(
//         &publisher,
//         &node,
//         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
//         "/teensy_debug"
        
//         );
  RCCHECK(
        rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/teensy_debug"
        
        )
  );
  
 
  // Create executor
//   rclc_executor_init(&executor, &support.context, 10, &allocator);
//   rclc_executor_add_subscription(
//         &executor,
//         &subscriber,
//         &msgss,
//         subscription_callback,
//         ON_NEW_DATA
//         );

  RCCHECK(rclc_executor_init(&executor, &support.context, 10, &allocator));
  RCCHECK(
        rclc_executor_add_subscription(
        &executor,
        &subscriber,
        &msgss,
        subscription_callback,
        ON_NEW_DATA
        )
  );

  return true;
}

void destroy_entities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_subscription_fini(&subscriber, &node);
    rcl_publisher_fini(&publisher, &node);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

 
 
// Simple user-data holder for heartbeat
struct ODriveUserData
{
    Heartbeat_msg_t last_heartbeat;
    bool received_heartbeat = false;
};
 
ODriveUserData odrv0_user_data;
ODriveUserData odrv1_user_data;
 
// Heartbeat callback
void onHeartbeat(Heartbeat_msg_t &msg, void *user_data)
{
    auto *ud = static_cast<ODriveUserData *>(user_data);
    ud->last_heartbeat = msg;
    ud->received_heartbeat = true;
}
 
// Global CAN receive handler
void onCanMessage(const CanMsg &msg)
{
    onReceive(msg, odrv0);//motor 0
    onReceive(msg, odrv1);//motor 1
}
 
// Basic CAN setup
bool setupCan()
{
    can_intf.begin();
    can_intf.setBaudRate(CAN_BAUDRATE);
    can_intf.setMaxMB(16);
    can_intf.enableFIFO();
    can_intf.enableFIFOInterrupt();
    can_intf.onReceive(onCanMessage);
    return true;
}



 
void setup()
{
    Serial.begin(115200);
    delay(100);
 
    //
    state = WAITING_AGENT;

    msgss.data.data = (char*) malloc(256);
            msgss.data.size = 0;
            msgss.data.capacity = 256;
 
    // Configure micro-ROS transport (USB)
    set_microros_serial_transports(Serial);
 

 
 
 
    // Register status (heartbeat) callback
    odrv0.onStatus(onHeartbeat, &odrv0_user_data);
    odrv1.onStatus(onHeartbeat, &odrv1_user_data);
 
    if (!setupCan())
    {
        Serial.println("CAN setup failed");
        while (true)
        {
        }
    }
 
    // Wait for heartbeat from ODrive 0
    const unsigned long heartbeat_timeout = 3000;
    unsigned long start = millis();
    while (!odrv0_user_data.received_heartbeat && millis() - start < heartbeat_timeout)
    {
        pumpEvents(can_intf);
        delay(5);
    }
 
    if (!odrv0_user_data.received_heartbeat)
    {
        Serial.println("No heartbeat from ODrive 0");
        while (true)
        {
            pumpEvents(can_intf);
            delay(10);
        }
    }

    while (!odrv1_user_data.received_heartbeat && millis() - start < heartbeat_timeout)
    {
        pumpEvents(can_intf);
        delay(5);
    }
 
    if (!odrv1_user_data.received_heartbeat)
    {
        Serial.println("No heartbeat from ODrive 0");
        while (true)
        {
            pumpEvents(can_intf);
            delay(10);
        }
    }
 
    Serial.println("Heartbeat received from ODrive 0, 1");
 
    // Clear any errors and enter closed loop
    odrv0.clearErrors();
    delay(10);
    odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    delay(50);

    odrv1.clearErrors();
    delay(10);
    odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    delay(50);
 
    // Velocity control mode
    odrv0.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
    odrv1.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP);
}
 
void loop()
{   
    
    switch (state)
    {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
    case AGENT_AVAILABLE:
        state=(create_ros_entities() ? AGENT_CONNECTED: WAITING_AGENT);
        if (state == AGENT_CONNECTED){

            
        }
        else{
            //memory free
            //destroy created ros entities
            destroy_entities();
        }
        break;
    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED);

        if(state == AGENT_CONNECTED){
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

            
        }
        
        break;
    case AGENT_DISCONNECTED:
        destroy_entities();
        state = WAITING_AGENT;
        break;
    default:
        break;
    }

    digitalWrite(LED_BUILTIN,  state == WAITING_AGENT ? LOW : HIGH );

    // Process incoming CAN frames
    pumpEvents(can_intf);
 
    delay(10);
}