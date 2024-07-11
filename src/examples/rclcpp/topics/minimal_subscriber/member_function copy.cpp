#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <thread>

#include "ecrt.h"
#include <stdio.h>
#include <unistd.h>
#include <sched.h>
#include <sys/mman.h>
#include <errno.h>
#include <string.h>
#include <sys/resource.h>
#include <time.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/types.h>
#include <malloc.h>
#include <inttypes.h>

#include <iostream>
#include <vector>

static unsigned int counter = 0;
static unsigned int sync_ref_counter = 0;
static struct timespec apptime;

#define TASK_FREQUENCY  125  /* Hz */
#define CLOCK_TO_USE CLOCK_REALTIME
#define TIMEOUT_CLEAR_ERROR  (1*TASK_FREQUENCY) /* clearing error timeout */

/*Time calculation*/
#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / TASK_FREQUENCY)
#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC +   \
                       (B).tv_nsec - (A).tv_nsec)
#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)


// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};
int pos = 0;
int toq = 0;
int vel = 0;
int ii = 0;
int torque_from_ros = 0;
int p_flag = 0;

/****************************************************************************/

// process data
static uint8_t *domain1_pd = NULL;


#define JiHua    0x00000130, 0x01300007
#define Junction 0x00494350, 0x000009d3

#define Inovance_IS620N_DC_SYNC_SHIFT 4400000

// offsets for PDO entries
static unsigned int ctrl_word   ;
static unsigned int mode  ;
static unsigned int tar_torq    ;
static unsigned int max_torq    ;
static unsigned int tar_pos    ;
static unsigned int digital_output;
static unsigned int max_speed  ;
static unsigned int touch_probe_func ;
static unsigned int tar_vel ;
static unsigned int error_code  ;
static unsigned int status_word;
static unsigned int mode_display ;
static unsigned int pos_act;
static unsigned int vel_act;
static unsigned int torq_act;
static unsigned int pos_gap_act;
static unsigned int touch_probe_status;
static unsigned int touch_probe_pos;
static unsigned int touch_probe_pos2;
static unsigned int digital_input;

// offsets for PDO entries
static int off_dig_out;
static int off_counter_in;
static int off_counter_out;


static unsigned int blink = 0;

const struct timespec cycletime = {0, PERIOD_NS};

static ec_sdo_request_t *status_request ;
static ec_sdo_request_t *control_word_request ;
static ec_sdo_request_t *position_request ;
static ec_sdo_request_t *position_current_request ;

int enabled[19] =    {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0};
int positions[19] =  {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0};
int velocities[19] = {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0};
int torques[19] =    {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0};

int positions_to_robot[19] = {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0};
bool arrived[19] = {true,true,true,true,true, true,true,true,true,true,
true,true,true,true,true, true,true,true,true};
auto message_arrived = std_msgs::msg::Int32();
int zone = 500;
int speed = 1000;

static struct {
    unsigned int ctrl_word;
    unsigned int target_position;
    unsigned int target_torque;
    unsigned int target_velocity;
    unsigned int max_torque;
    unsigned int DO;
    unsigned int DI;
    unsigned int act_DI;
    
    unsigned int offset_velocity ;
    unsigned int offset_torque ;
    unsigned int offset_position ;

    unsigned int error_code;
    unsigned int status_word ;
    unsigned int act_position;
    unsigned int act_torque;

    unsigned int disp_mode;
    unsigned int actual_ferr;
    unsigned int probe_status;
    unsigned int probe1_pos;
    
    unsigned int probe2_pos;
    unsigned int act_velocity;
    unsigned int mode_Of_Operation;
    unsigned int mode_Of_Operation_dsiplay;
} offset[19];


const static ec_pdo_entry_reg_t domain1_regs[] = {
    //slave 0
    //RxPDO
    { 0,4, JiHua, 0x6040, 0, &offset[4].ctrl_word },
    //{ 0,0, JiHua, 0x6060, 0, &offset[4].mode_Of_Operation },
    { 0,4, JiHua, 0x6071, 0, &offset[4].target_torque },
    { 0,4, JiHua, 0x607a, 0, &offset[4].target_position },
    { 0,4, JiHua, 0x60b1, 0, &offset[4].offset_velocity },
    { 0,4, JiHua, 0x60b2, 0, &offset[4].offset_torque },
    { 0,4, JiHua, 0x60ff, 0, &offset[4].target_velocity },

    //TxPDO
    { 0,4, JiHua, 0x6041, 0, &offset[4].status_word },      
    { 0,4, JiHua, 0x6064, 0, &offset[4].act_position},
    { 0,4, JiHua, 0x606c, 0, &offset[4].act_velocity},    
    { 0,4, JiHua, 0x6077, 0, &offset[4].act_torque },
    { 0,4, JiHua, 0x6061, 0, &offset[4].mode_Of_Operation_dsiplay },      
      

    { 0,5, JiHua, 0x6040, 0, &offset[5].ctrl_word },
    { 0,5, JiHua, 0x6071, 0, &offset[5].target_torque },
    { 0,5, JiHua, 0x607a, 0, &offset[5].target_position },
    { 0,5, JiHua, 0x60b1, 0, &offset[5].offset_velocity },
    { 0,5, JiHua, 0x60b2, 0, &offset[5].offset_torque },
    { 0,5, JiHua, 0x60ff, 0, &offset[5].target_velocity },
    
    { 0,5, JiHua, 0x6041, 0, &offset[5].status_word },      
    { 0,5, JiHua, 0x6064, 0, &offset[5].act_position},
    { 0,5, JiHua, 0x606c, 0, &offset[5].act_velocity},    
    { 0,5, JiHua, 0x6077, 0, &offset[5].act_torque },
    { 0,5, JiHua, 0x6061, 0, &offset[5].mode_Of_Operation_dsiplay },   
    
       
    { 0,6, JiHua, 0x6040, 0, &offset[6].ctrl_word },
    { 0,6, JiHua, 0x6071, 0, &offset[6].target_torque },
    { 0,6, JiHua, 0x607a, 0, &offset[6].target_position },
    { 0,6, JiHua, 0x60b1, 0, &offset[6].offset_velocity },
    { 0,6, JiHua, 0x60b2, 0, &offset[6].offset_torque },
    { 0,6, JiHua, 0x60ff, 0, &offset[6].target_velocity },

    { 0,6, JiHua, 0x6041, 0, &offset[6].status_word },      
    { 0,6, JiHua, 0x6064, 0, &offset[6].act_position},
    { 0,6, JiHua, 0x606c, 0, &offset[6].act_velocity},    
    { 0,6, JiHua, 0x6077, 0, &offset[6].act_torque },
    { 0,6, JiHua, 0x6061, 0, &offset[6].mode_Of_Operation_dsiplay },  
    
    
    { 0,7, JiHua, 0x6040, 0, &offset[7].ctrl_word },
    { 0,7, JiHua, 0x6071, 0, &offset[7].target_torque },
    { 0,7, JiHua, 0x607a, 0, &offset[7].target_position },
    { 0,7, JiHua, 0x60b1, 0, &offset[7].offset_velocity },
    { 0,7, JiHua, 0x60b2, 0, &offset[7].offset_torque },
    { 0,7, JiHua, 0x60ff, 0, &offset[7].target_velocity },

    { 0,7, JiHua, 0x6041, 0, &offset[7].status_word },      
    { 0,7, JiHua, 0x6064, 0, &offset[7].act_position},
    { 0,7, JiHua, 0x606c, 0, &offset[7].act_velocity},    
    { 0,7, JiHua, 0x6077, 0, &offset[7].act_torque },
    { 0,7, JiHua, 0x6061, 0, &offset[7].mode_Of_Operation_dsiplay },   
       
    
    { 0,8, JiHua, 0x6040, 0, &offset[8].ctrl_word },
    { 0,8, JiHua, 0x6071, 0, &offset[8].target_torque },
    { 0,8, JiHua, 0x607a, 0, &offset[8].target_position },
    { 0,8, JiHua, 0x60b1, 0, &offset[8].offset_velocity },
    { 0,8, JiHua, 0x60b2, 0, &offset[8].offset_torque },
    { 0,8, JiHua, 0x60ff, 0, &offset[8].target_velocity },

    { 0,8, JiHua, 0x6041, 0, &offset[8].status_word },      
    { 0,8, JiHua, 0x6064, 0, &offset[8].act_position},
    { 0,8, JiHua, 0x606c, 0, &offset[8].act_velocity},    
    { 0,8, JiHua, 0x6077, 0, &offset[8].act_torque },
    { 0,8, JiHua, 0x6061, 0, &offset[8].mode_Of_Operation_dsiplay },  
    
    
    { 0,9, JiHua, 0x6040, 0, &offset[9].ctrl_word },
    { 0,9, JiHua, 0x6071, 0, &offset[9].target_torque },
    { 0,9, JiHua, 0x607a, 0, &offset[9].target_position },
    { 0,9, JiHua, 0x60b1, 0, &offset[9].offset_velocity },
    { 0,9, JiHua, 0x60b2, 0, &offset[9].offset_torque },
    { 0,9, JiHua, 0x60ff, 0, &offset[9].target_velocity },

    { 0,9, JiHua, 0x6041, 0, &offset[9].status_word },      
    { 0,9, JiHua, 0x6064, 0, &offset[9].act_position},
    { 0,9, JiHua, 0x606c, 0, &offset[9].act_velocity},    
    { 0,9, JiHua, 0x6077, 0, &offset[9].act_torque },
    { 0,9, JiHua, 0x6061, 0, &offset[9].mode_Of_Operation_dsiplay },  
    
    
    { 0,10, JiHua, 0x6040, 0, &offset[10].ctrl_word },
    { 0,10, JiHua, 0x6071, 0, &offset[10].target_torque },
    { 0,10, JiHua, 0x607a, 0, &offset[10].target_position },
    { 0,10, JiHua, 0x60b1, 0, &offset[10].offset_velocity },
    { 0,10, JiHua, 0x60b2, 0, &offset[10].offset_torque },
    { 0,10, JiHua, 0x60ff, 0, &offset[10].target_velocity },

    { 0,10, JiHua, 0x6041, 0, &offset[10].status_word },      
    { 0,10, JiHua, 0x6064, 0, &offset[10].act_position},
    { 0,10, JiHua, 0x606c, 0, &offset[10].act_velocity},    
    { 0,10, JiHua, 0x6077, 0, &offset[10].act_torque },
    { 0,10, JiHua, 0x6061, 0, &offset[10].mode_Of_Operation_dsiplay },     
    
    
    { 0,11, JiHua, 0x6040, 0, &offset[11].ctrl_word },
    { 0,11, JiHua, 0x6071, 0, &offset[11].target_torque },
    { 0,11, JiHua, 0x607a, 0, &offset[11].target_position },
    { 0,11, JiHua, 0x60b1, 0, &offset[11].offset_velocity },
    { 0,11, JiHua, 0x60b2, 0, &offset[11].offset_torque },
    { 0,11, JiHua, 0x60ff, 0, &offset[11].target_velocity },

    { 0,11, JiHua, 0x6041, 0, &offset[11].status_word },      
    { 0,11, JiHua, 0x6064, 0, &offset[11].act_position},
    { 0,11, JiHua, 0x606c, 0, &offset[11].act_velocity},    
    { 0,11, JiHua, 0x6077, 0, &offset[11].act_torque },
    { 0,11, JiHua, 0x6061, 0, &offset[11].mode_Of_Operation_dsiplay },  
    
    
    { 0,12, JiHua, 0x6040, 0, &offset[12].ctrl_word },
    { 0,12, JiHua, 0x6071, 0, &offset[12].target_torque },
    { 0,12, JiHua, 0x607a, 0, &offset[12].target_position },
    { 0,12, JiHua, 0x60b1, 0, &offset[12].offset_velocity },
    { 0,12, JiHua, 0x60b2, 0, &offset[12].offset_torque },
    { 0,12, JiHua, 0x60ff, 0, &offset[12].target_velocity },

    { 0,12, JiHua, 0x6041, 0, &offset[12].status_word },      
    { 0,12, JiHua, 0x6064, 0, &offset[12].act_position},
    { 0,12, JiHua, 0x606c, 0, &offset[12].act_velocity},    
    { 0,12, JiHua, 0x6077, 0, &offset[12].act_torque },
    { 0,12, JiHua, 0x6061, 0, &offset[12].mode_Of_Operation_dsiplay },      
    
    
    { 0,13, JiHua, 0x6040, 0, &offset[13].ctrl_word },
    { 0,13, JiHua, 0x6071, 0, &offset[13].target_torque },
    { 0,13, JiHua, 0x607a, 0, &offset[13].target_position },
    { 0,13, JiHua, 0x60b1, 0, &offset[13].offset_velocity },
    { 0,13, JiHua, 0x60b2, 0, &offset[13].offset_torque },
    { 0,13, JiHua, 0x60ff, 0, &offset[13].target_velocity },

    { 0,13, JiHua, 0x6041, 0, &offset[13].status_word },      
    { 0,13, JiHua, 0x6064, 0, &offset[13].act_position},
    { 0,13, JiHua, 0x606c, 0, &offset[13].act_velocity},    
    { 0,13, JiHua, 0x6077, 0, &offset[13].act_torque },
    { 0,13, JiHua, 0x6061, 0, &offset[13].mode_Of_Operation_dsiplay },  
    
    
    { 0,14, JiHua, 0x6040, 0, &offset[14].ctrl_word },
    { 0,14, JiHua, 0x6071, 0, &offset[14].target_torque },
    { 0,14, JiHua, 0x607a, 0, &offset[14].target_position },
    { 0,14, JiHua, 0x60b1, 0, &offset[14].offset_velocity },
    { 0,14, JiHua, 0x60b2, 0, &offset[14].offset_torque },
    { 0,14, JiHua, 0x60ff, 0, &offset[14].target_velocity },

    { 0,14, JiHua, 0x6041, 0, &offset[14].status_word },      
    { 0,14, JiHua, 0x6064, 0, &offset[14].act_position},
    { 0,14, JiHua, 0x606c, 0, &offset[14].act_velocity},    
    { 0,14, JiHua, 0x6077, 0, &offset[14].act_torque },
    { 0,14, JiHua, 0x6061, 0, &offset[14].mode_Of_Operation_dsiplay },  
    
    
    { 0,15, JiHua, 0x6040, 0, &offset[15].ctrl_word },
    { 0,15, JiHua, 0x6071, 0, &offset[15].target_torque },
    { 0,15, JiHua, 0x607a, 0, &offset[15].target_position },
    { 0,15, JiHua, 0x60b1, 0, &offset[15].offset_velocity },
    { 0,15, JiHua, 0x60b2, 0, &offset[15].offset_torque },
    { 0,15, JiHua, 0x60ff, 0, &offset[15].target_velocity },

    { 0,15, JiHua, 0x6041, 0, &offset[15].status_word },      
    { 0,15, JiHua, 0x6064, 0, &offset[15].act_position},
    { 0,15, JiHua, 0x606c, 0, &offset[15].act_velocity},    
    { 0,15, JiHua, 0x6077, 0, &offset[15].act_torque },
    { 0,15, JiHua, 0x6061, 0, &offset[15].mode_Of_Operation_dsiplay },  


    { 0,16, JiHua, 0x6040, 0, &offset[16].ctrl_word },
    { 0,16, JiHua, 0x6071, 0, &offset[16].target_torque },
    { 0,16, JiHua, 0x607a, 0, &offset[16].target_position },
    { 0,16, JiHua, 0x60b1, 0, &offset[16].offset_velocity },
    { 0,16, JiHua, 0x60b2, 0, &offset[16].offset_torque },
    { 0,16, JiHua, 0x60ff, 0, &offset[16].target_velocity },

    { 0,16, JiHua, 0x6041, 0, &offset[16].status_word },      
    { 0,16, JiHua, 0x6064, 0, &offset[16].act_position},
    { 0,16, JiHua, 0x606c, 0, &offset[16].act_velocity},    
    { 0,16, JiHua, 0x6077, 0, &offset[16].act_torque },
    { 0,16, JiHua, 0x6061, 0, &offset[16].mode_Of_Operation_dsiplay },  


    { 0,17, JiHua, 0x6040, 0, &offset[17].ctrl_word },
    { 0,17, JiHua, 0x6071, 0, &offset[17].target_torque },
    { 0,17, JiHua, 0x607a, 0, &offset[17].target_position },
    { 0,17, JiHua, 0x60b1, 0, &offset[17].offset_velocity },
    { 0,17, JiHua, 0x60b2, 0, &offset[17].offset_torque },
    { 0,17, JiHua, 0x60ff, 0, &offset[17].target_velocity },

    { 0,17, JiHua, 0x6041, 0, &offset[17].status_word },      
    { 0,17, JiHua, 0x6064, 0, &offset[17].act_position},
    { 0,17, JiHua, 0x606c, 0, &offset[17].act_velocity},    
    { 0,17, JiHua, 0x6077, 0, &offset[17].act_torque },
    { 0,17, JiHua, 0x6061, 0, &offset[17].mode_Of_Operation_dsiplay },  


    { 0,18, JiHua, 0x6040, 0, &offset[18].ctrl_word },
    { 0,18, JiHua, 0x6071, 0, &offset[18].target_torque },
    { 0,18, JiHua, 0x607a, 0, &offset[18].target_position },
    { 0,18, JiHua, 0x60b1, 0, &offset[18].offset_velocity },
    { 0,18, JiHua, 0x60b2, 0, &offset[18].offset_torque },
    { 0,18, JiHua, 0x60ff, 0, &offset[18].target_velocity },

    { 0,18, JiHua, 0x6041, 0, &offset[18].status_word },      
    { 0,18, JiHua, 0x6064, 0, &offset[18].act_position},
    { 0,18, JiHua, 0x606c, 0, &offset[18].act_velocity},    
    { 0,18, JiHua, 0x6077, 0, &offset[18].act_torque },
    { 0,18, JiHua, 0x6061, 0, &offset[18].mode_Of_Operation_dsiplay },  
             
    {}
};


ec_pdo_entry_info_t IS620N_pdo_entries[] = {
    /* RxPdo 0x1602 */
    { 0x6040, 0x00, 16 }, 
    //{ 0x6060, 0x00, 8 },      
    { 0x6071, 0x00, 16 }, 
    { 0x607a, 0x00, 32 },  
    { 0x60b1, 0x00, 32 }, 
    { 0x60b2, 0x00, 16 },  
    { 0x60ff, 0x00, 32 },  

    /* TxPDO 0x1a02 */
    { 0x6041, 0x00, 16 },
    { 0x6064, 0x00, 32 }, 
    { 0x606c, 0x00, 32 },
    { 0x6077, 0x00, 16 },
    { 0x6061, 0x00, 8 },
};

ec_pdo_info_t IS620N_pdos[] = {
    { 0x1600, 6, IS620N_pdo_entries + 0 },
    { 0x1a00, 5, IS620N_pdo_entries + 6 },
   // { 0x1701, 4, IS620N_pdo_entries + 0 },
   // { 0x1B04, 10, IS620N_pdo_entries + 4 },

};

ec_sync_info_t IS620N_syncs[] = {
    { 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
    { 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
    { 2, EC_DIR_OUTPUT, 1, IS620N_pdos + 0, EC_WD_DEFAULT },
    { 3, EC_DIR_INPUT, 1, IS620N_pdos + 1, EC_WD_DEFAULT },
    { 0xFF }
};

/*****************************************************************************/

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
    struct timespec result;

    if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    } else {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }

    return result;
}

rclcpp::Node::SharedPtr g_node;
rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr g_pub_positions;
rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr g_pub_velocities;
rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr g_pub_torques;
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr g_pub_arrived;

auto message_torques = std_msgs::msg::Int32MultiArray();
auto message_positions = std_msgs::msg::Int32MultiArray();
auto message_velocities = std_msgs::msg::Int32MultiArray();

void ec_igh_write();
void is620n_receive();
void ec_igh_init();
void ec_igh_master_activate();
/*****************************************************************************/

class Node {
public:
    std::vector<int> data;
    Node* next;

    Node(const std::vector<int>& value) : data(value), next(nullptr) {}
};

class Queue {
private:
    Node* front;
    Node* rear;

public:
    Queue() : front(nullptr), rear(nullptr) {}

    ~Queue() {
        while (!isEmpty()) {
            dequeue();
        }
    }

    void enqueue(const std::vector<int>& value) {
        Node* newNode = new Node(value);
        if (rear) {
            rear->next = newNode;
        }
        rear = newNode;
        if (!front) {
            front = newNode;
        }
    }

    void dequeue() {
        if (isEmpty()) {
            std::cout << "Queue is empty, cannot dequeue." << std::endl;
            return;
        }
        Node* temp = front;
        front = front->next;
        if (!front) {
            rear = nullptr;
        }
        delete temp;
    }

    std::vector<int> getFront() const {
        if (isEmpty()) {
            std::cout << "Queue is empty." << std::endl;
            return std::vector<int>();  // 或者抛出异常
        }
        return front->data;
    }

    bool isEmpty() const {
        return front == nullptr;
    }
};

Queue positions_queue;
std::vector<int> position_current = {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0};



void check_domain1_state(void)
{
    ec_domain_state_t ds;
    ecrt_domain_state(domain1, &ds);
    if (ds.working_counter != domain1_state.working_counter)
        printf("Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain1_state.wc_state)
        printf("Domain1: State %u.\n", ds.wc_state);

    domain1_state = ds;
}

/*****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;
    ecrt_master_state(master, &ms);
    if (ms.slaves_responding != master_state.slaves_responding)
        printf("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printf("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printf("Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}

/****************************************************************************/

void cyclic_task()
{
    struct timespec wakeupTime, time;
    // get current time
    //clock_gettime(CLOCK_TO_USE, &wakeupTime);
    pos = 1;
    int dir = 1;
    
    
        //wakeupTime = timespec_add(wakeupTime, cycletime);
        //clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);
        is620n_receive();        

        p_flag ++;
        for(ii=4;ii<=18;ii++){
        
          uint16_t ss = EC_READ_U16(domain1_pd + offset[ii].status_word);
          if(p_flag>=120){
            printf("status %d : 0x%04x \n",ii,ss);
          }
          if((ss&0xFF)==0x50){
              EC_WRITE_U16( domain1_pd + offset[ii].ctrl_word, 0x06 );
              printf("0x%04x \n",0x06);
          }
          else if((ss&0xFF)==0x31){
              EC_WRITE_U16( domain1_pd + offset[ii].ctrl_word, 0x07 );
          }  
          else if((ss&0xFF)==0x33){
              EC_WRITE_U16( domain1_pd + offset[ii].ctrl_word, 0x0F );
              
          }      
          else if( (ss&0xFF) ==0x37){ 
              enabled[ii]=1;              
                pos = EC_READ_S32(domain1_pd + offset[ii].act_position);
                toq = EC_READ_S16(domain1_pd + offset[ii].act_torque);
                vel = EC_READ_S32(domain1_pd + offset[ii].act_velocity);

                auto message = std_msgs::msg::Int32MultiArray();

                message_velocities.data[ii] = vel;
                message_torques.data[ii] = toq;
                message_positions.data[ii] = pos;

              //printf("position %d : 0x%04x \n",ii,pos);
                    
              if( (enabled[4]==1) && (enabled[5]==1)&& (enabled[6]==1)&& (enabled[7]==1)
              && (enabled[8]==1)&& (enabled[9]==1)&& (enabled[10]==1)&& (enabled[11]==1)
              && (enabled[12]==1)&& (enabled[13]==1)&& (enabled[14]==1)&& (enabled[15]==1)
              && (enabled[16]==1)&& (enabled[17]==1)&& (enabled[18]==1)
              && (!positions_queue.isEmpty())
              ){
                
                if(ii==4){
                    position_current = positions_queue.getFront();
                    positions_queue.dequeue();
                }

                int target = position_current[ii];
                // int moveTo = target;
                // if(target > pos+zone){
                //     moveTo = pos+speed;
                // }
                // else if(target<pos-zone){
                //     moveTo = pos-speed;
                // }
                // else{
                //     arrived[ii] = true;
                // }
                // printf("position %d 0x%04x to 0x%04x \n",ii,pos,moveTo);
                // EC_WRITE_S32( domain1_pd + offset[ii].target_position,moveTo );   
                printf("position %d 0x%04x to 0x%04x \n",ii,pos,target);
                EC_WRITE_S32( domain1_pd + offset[ii].target_position,target );                  
                
              }
              else{
                EC_WRITE_S32( domain1_pd + offset[ii].target_position, pos );        
              }
          }
          else{
            enabled[ii]=0;
          }
        }

        if(p_flag>=120){
            p_flag = 0;
        }


        g_pub_torques->publish(message_torques);
        g_pub_velocities->publish(message_velocities);
        g_pub_positions->publish(message_positions);

        if( (arrived[4]==1) && (arrived[5]==1)&& (arrived[6]==1)&& (arrived[7]==1)
        && (arrived[8]==1)&& (arrived[9]==1)&& (arrived[10]==1)&& (arrived[11]==1)
        && (arrived[12]==1)&& (arrived[13]==1)&& (arrived[14]==1)&& (arrived[15]==1)
        && (enabled[16]==1)&& (enabled[17]==1)&& (enabled[18]==1)
        && (positions_queue.isEmpty())
        ){
            message_arrived.data = 1;
            g_pub_arrived->publish(message_arrived);
        } 
        
        ec_igh_write();
}

void ec_igh_write()
{
    clock_gettime(CLOCK_TO_USE, &apptime);

    ecrt_master_application_time(master, TIMESPEC2NS(apptime));
    if (sync_ref_counter) {
        sync_ref_counter--;
    } else {
        sync_ref_counter = 1; // sync every cycle
        ecrt_master_sync_reference_clock(master); //DC reference clock drift compensation
    }
    ecrt_master_sync_slave_clocks(master);  //DC clock drift compensation     
    // queue process data
    ecrt_domain_queue(domain1);            
    // send EtherCAT data
    ecrt_master_send(master);
}

void is620n_receive()
{
    /* receive process data */
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);
}

void ec_igh_master_activate()
{
    printf("......Activating master......\n");
    if ( ecrt_master_activate( master ) ) {
        exit( EXIT_FAILURE );
    }
    if ( !(domain1_pd = ecrt_domain_data( domain1 )) ) {
        exit( EXIT_FAILURE );
    }

    printf("......Master  Activated.......\n");

}

/****************************************************************************/


void ec_igh_init()
{
    ec_master_info_t master_info;
    ec_slave_info_t slave_info;
    int ret;
    int slavecnt;
    
   
 //   uint8_t* data = (uint8_t*)malloc(4*sizeof(uint8_t));
    int32_t sdo_data;
    //act_pos =(int32_t*)malloc(6*sizeof(int32_t));
   
    //uint32_t  abort_code;
   // size_t rb;
    int i = 0;
    master = ecrt_request_master( 0 );
    if ( !master ) {
        exit( EXIT_FAILURE );
    }
   
    domain1 = ecrt_master_create_domain( master );
    if ( !domain1 ) {
        exit( EXIT_FAILURE );
    }
    
    //---------get master / salve info----------------------
    ret = ecrt_master(master,&master_info);
    slavecnt = master_info.slave_count;
    printf("ret = %d, slavecnt = %d, apptime = %"PRIu64"\n",ret, master_info.slave_count,master_info.app_time);
    ret = ecrt_master_get_slave(master,0,&slave_info);
    printf("ret = %d,spos = %d, pcode = %d\n",ret,slave_info.position,slave_info.product_code);
  
    printf("servo %d  begin init! \n",i);
        
    for(ii=4;ii<=18;ii++){
        ec_slave_config_t *sc;
		
        if (!(sc = ecrt_master_slave_config( master,0,ii ,JiHua)) ) {
            fprintf(stderr, "Failed to get slave configuration for IS620N.\n");
            exit( EXIT_FAILURE );
        }

        printf("Configuring PDOs...\n");
        if ( ecrt_slave_config_pdos( sc, EC_END, IS620N_syncs ) ) {
            fprintf( stderr, "Failed to configure IS620N PDOs.\n" );
            exit( EXIT_FAILURE );
        }

        // configure SYNC signals for this slave
        ecrt_slave_config_sdo8( sc, 0x6060, 0,8 );
        //ecrt_slave_config_sdo16( sc, 0x1C32, 1, 2 );
        //ecrt_slave_config_sdo16( sc, 0x1C33, 1, 2 );
        //ecrt_slave_config_sdo8( sc, 0x60C2, 1, 8 );
        //ecrt_slave_config_sdo8( sc, 0x60C2, 2, -3 );     
        
        ecrt_slave_config_dc(sc, 0x0300, PERIOD_NS, (PERIOD_NS/4) , 0, 0);
    }

    if ( ecrt_domain_reg_pdo_entry_list( domain1, domain1_regs ) ) {
        fprintf( stderr, "PDO entry registration failed!\n" );
        exit( EXIT_FAILURE );
    }

}




class MySubscriberNode : public rclcpp::Node
{
public:
    MySubscriberNode()
        : Node("my_subscriber_node")
    {

        g_pub_torques = this->create_publisher<std_msgs::msg::Int32MultiArray>("torques", 10);
        g_pub_positions = this->create_publisher<std_msgs::msg::Int32MultiArray>("positions", 10);
        g_pub_velocities = this->create_publisher<std_msgs::msg::Int32MultiArray>("velocities", 10);
        g_pub_arrived = this->create_publisher<std_msgs::msg::Int32>("arrived", 10);

        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "positions_to_robot", 10, std::bind(&MySubscriberNode::topic_callback_positions, this, std::placeholders::_1));
        sub_zone = this->create_subscription<std_msgs::msg::Int32>(
            "set_zone", 10, std::bind(&MySubscriberNode::topic_callback_zone, this, std::placeholders::_1));
        sub_speed = this->create_subscription<std_msgs::msg::Int32>(
            "set_speed", 10, std::bind(&MySubscriberNode::topic_callback_speed, this, std::placeholders::_1));                        


        // Start the additional thread with a timer
        worker_thread_ = std::thread(std::bind(&MySubscriberNode::worker_thread_function, this));
    }

    ~MySubscriberNode()
    {
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }

private:
    void topic_callback_positions(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        std::vector<int> myVector = {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0};
        RCLCPP_INFO(this->get_logger(), "Received: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d]",
                    msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4],
                     msg->data[6], msg->data[7], msg->data[8], msg->data[9], msg->data[10],
                      msg->data[11], msg->data[12], msg->data[13], msg->data[14], msg->data[15], msg->data[16]);
        for(int i=0;i<msg->data.size();i++){
            positions_to_robot[i] = msg->data[i];
            arrived[i] = false;

            myVector[i] = msg->data[i];
        }
        
        positions_queue.enqueue(myVector);
        message_arrived.data = 0;
        g_pub_arrived->publish(message_arrived);
    }

    void topic_callback_zone(const std_msgs::msg::Int32::SharedPtr msg)
    {
       zone = msg->data;
    }   

    void topic_callback_speed(const std_msgs::msg::Int32::SharedPtr msg)
    {
       speed = msg->data;
    }      

    void worker_thread_function()
    {
        rclcpp::Rate rate(125); // 100 Hz -> 10 ms
        while (rclcpp::ok())
        {
            cyclic_task();
            rate.sleep();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_zone;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_speed;
    std::thread worker_thread_;
};

int main(int argc, char *argv[])
{
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
		perror("mlockall failed");
		return -1;
	}

    ec_igh_init();
    ec_igh_master_activate(); 
    message_torques.data =    {0,0,0,0,0 ,0,0,0,0,0 ,0,0,0,0,0 ,0,0,0,0};
    message_positions.data =  {0,0,0,0,0 ,0,0,0,0,0 ,0,0,0,0,0 ,0,0,0,0};
    message_velocities.data = {0,0,0,0,0, 0,0,0,0,0 ,0,0,0,0,0 ,0,0,0,0};
    rclcpp::init(argc, argv);
    g_node = std::make_shared<MySubscriberNode>();
    rclcpp::spin(g_node);
    rclcpp::shutdown();
    return 0;
}