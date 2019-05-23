#define   GATE_2_NODE       1
#define   NODE_2_GATE       GATE_2_NODE
#define   GATE_2_APP_BLE    2
#define   APP_BLE_2_GATE    GATE_2_APP_BLE
#define   GATE_2_GATE       4

#define HEADER_PACKET     0x36
#define LEN_ID_APP        2
#define LEN_DIRECTION     1
#define LEN_LEN_PAYLOAD   2
#define LEN_PAYLOAD       20
#define LEN_PACKET    (LEN_ID_APP + LEN_DIRECTION + LEN_LEN_PAYLOAD + LEN_PAYLOAD)

#define   CMD_UPDATE_NODE_STATUS        12
#define   CMD_UPDATE_GROUP_STATUS       13
#define   CMD_UPDATE_SCENE_STATUS       21

// cmd of node:
#define   CMD_GET_STATUS                  0x21
#define   CMD_UPDATE_DEVICE_STATUS_REPLY  14
#define   CMD_UPDATE_SCENE_STATUS_REPLY   33

// config
#define CMD_STATUS_APP    0x37
#define CMD_STATUS_NODE   0x36      // Node: trans-receive

#define CMD_CHANGE_NAME   10
#define CMD_ADD_NODE   3
#define CMD_DELETE_NODE   5



