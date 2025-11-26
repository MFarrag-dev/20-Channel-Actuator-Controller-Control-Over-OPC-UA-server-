/* main.c
   ESP-IDF project main for:
   - driving 3x74HC595 -> 3xULN2003 -> 20 relays
   - OPC UA server (open62541) with 20 boolean nodes
   - W5500 (WIZ820io) Ethernet via esp_eth
   - watchdog + network auto-reconnect + LED indicator
*/

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_task_wdt.h"

#include "open62541.h" // needs open62541 component added

// --- Configuration (pins as given) ---
#define PIN_LED_INDICATOR   GPIO_NUM_2

#define PIN_SR_DATA         GPIO_NUM_13
#define PIN_SR_CLK          GPIO_NUM_14
#define PIN_SR_LATCH        GPIO_NUM_25
#define PIN_SR_ENABLE       GPIO_NUM_33  // optional (Normally Low Active High)
#define PIN_SR_CLEAR        GPIO_NUM_26  // optional (Normally High Active Low)

// WIZ820IO (W5500) SPI pins (esp_eth/w5500 expects SPI bus; tune in menuconfig or init)
#define PIN_SPI_SCK         GPIO_NUM_18
#define PIN_SPI_MISO        GPIO_NUM_19
#define PIN_SPI_MOSI        GPIO_NUM_23
#define PIN_SPI_CS          GPIO_NUM_5
#define PIN_WIZ_RESET       GPIO_NUM_21
#define PIN_WIZ_INT         GPIO_NUM_27

// Other defines
#define TAG "OPCUA_RELAY"
#define NUM_RELAYS 20
#define WDT_TIMEOUT_SECONDS 6

// Relay state storage
static bool relays[NUM_RELAYS];
static SemaphoreHandle_t relays_mutex = NULL;

// Network connected flag
static bool net_connected = false;
static SemaphoreHandle_t net_mutex = NULL;

// OPC UA server pointer
static UA_Server *ua_server = NULL;

// Watchdog: we will register main tasks and pet periodically
#define WDT_TASK_NAME "main_task"

// Forward declarations
static void update_shift_registers_from_relays(void);
static void set_relay_index(int idx, bool value);
static bool get_relay_index(int idx);
static void led_task(void *arg);
static void network_monitor_task(void *arg);
static void opcua_server_task(void *arg);
static void wdt_pet_task(void *arg);

// Utility: atomic relay access
static void set_relay_index(int idx, bool value) {
    if (idx < 0 || idx >= NUM_RELAYS) return;
    xSemaphoreTake(relays_mutex, portMAX_DELAY);
    relays[idx] = value;
    // push to hardware
    update_shift_registers_from_relays();
    xSemaphoreGive(relays_mutex);
}

static bool get_relay_index(int idx) {
    bool v = false;
    if (idx < 0 || idx >= NUM_RELAYS) return false;
    xSemaphoreTake(relays_mutex, portMAX_DELAY);
    v = relays[idx];
    xSemaphoreGive(relays_mutex);
    return v;
}

/* ------------------- shift register code (bitbang) --------------------
   We use GPIO toggling to shift out 24 bits (3x74HC595). This is simple,
   deterministic and fast enough. Toggling time is microseconds-level; updating
   24 bits is < 1ms so it easily meets 50ms latency.
*/
static inline void sr_pulse_pin(gpio_num_t pin) {
    gpio_set_level(pin, 1);
    asm volatile("nop\nnop\nnop\n" :::);
    gpio_set_level(pin, 0);
}

static void sr_init_pins(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL<<PIN_SR_DATA) | (1ULL<<PIN_SR_CLK) | (1ULL<<PIN_SR_LATCH)
    };
    gpio_config(&io_conf);

    // optional pins
    gpio_set_direction(PIN_SR_ENABLE, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_SR_CLEAR, GPIO_MODE_OUTPUT);

    // default states
    gpio_set_level(PIN_SR_ENABLE, 0); // active low enable -> enable outputs
    gpio_set_level(PIN_SR_CLEAR, 1);  // active low clear -> not clearing
    gpio_set_level(PIN_SR_CLK, 0);
    gpio_set_level(PIN_SR_LATCH, 0);
    gpio_set_level(PIN_SR_DATA, 0);
}

static void sr_shift_out_24bits(uint32_t data24) {
    // send MSB first or LSB first depending on wiring. Here assume MSB first
    for (int b = 23; b >= 0; --b) {
        int bit = (data24 >> b) & 1;
        gpio_set_level(PIN_SR_DATA, bit);
        // pulse clock
        gpio_set_level(PIN_SR_CLK, 1);
        asm volatile("nop\nnop\n");
        gpio_set_level(PIN_SR_CLK, 0);
    }
    // latch
    gpio_set_level(PIN_SR_LATCH, 1);
    asm volatile("nop\nnop\n");
    gpio_set_level(PIN_SR_LATCH, 0);
}

// Map relays[0..19] to shift register bits.
// According to your mapping: first 7 bits of first two SRs, first 6 bits of last.
// We'll pack them into 24-bit word with the following assumed mapping:
// SR chain order: SR1 (first in chain) -> SR2 -> SR3 (last in chain).
// Common wiring: first bit shifted out ends up at Q7 of last SR depending on wiring.
// You should check and adjust bit ordering to match your actual hardware wiring.
// For now we assume: relay 0 -> SR1 bit0, ... sequential.
// We'll produce clear mapping table for easy changes.
static const int relay_to_sr_bit[NUM_RELAYS] = {
    // This mapping assumes simple sequential mapping; adapt if necessary
    // relay 0..6 -> first SR bits 0..6
    0,1,2,3,4,5,6,
    // relay 7..13 -> second SR bits 0..6
    8,9,10,11,12,13,14,
    // relay 14..19 -> third SR bits 0..5
    16,17,18,19,20,21
};

static void update_shift_registers_from_relays(void) {
    uint32_t data24 = 0;
    // Build 24-bit shift register data word. Bit numbering below is arbitrary
    // and must match wiring: adjust relay_to_sr_bit[] as needed.
    for (int i = 0; i < NUM_RELAYS; ++i) {
        int bitpos = relay_to_sr_bit[i];
        if (bitpos < 0 || bitpos >= 24) continue;
        if (relays[i]) data24 |= (1UL << bitpos);
    }
    // shift out
    sr_shift_out_24bits(data24);
}

/* ------------------- OPC UA callbacks / data source --------------------
   We expose each relay as a UA variable node (Boolean) and implement writing
   callbacks that toggle relays quickly.
*/
typedef struct {
    int idx;
} RelayNodeCtx;

static UA_StatusCode writeRelayCallback(UA_Server *server,
                                        const UA_NodeId *sessionId,
                                        void *sessionContext,
                                        const UA_NodeId *nodeId,
                                        void *nodeContext,
                                        const UA_NumericRange *range,
                                        const UA_DataValue *data) {
    (void)server; (void)sessionId; (void)sessionContext; (void)nodeId;
    (void)nodeContext; (void)range;
    if (!data || UA_Variant_isEmpty(&data->value)) return UA_STATUSCODE_BADINTERNALERROR;
    if (data->value.type != &UA_TYPES[UA_TYPES_BOOLEAN]) return UA_STATUSCODE_BADTYPEMISMATCH;
    UA_Boolean v = *(UA_Boolean*)data->value.data;
    RelayNodeCtx *ctx = (RelayNodeCtx*)nodeContext;
    if (!ctx) return UA_STATUSCODE_BADINTERNALERROR;
    set_relay_index(ctx->idx, (bool)v);
    // echo status can be handled by UA variable automatically because we update internal state
    return UA_STATUSCODE_GOOD;
}

static UA_DataSource relay_datasource_factory(int idx) {
    UA_DataSource ds;
    ds.read = NULL; // let UA variable read directly from underlying stored value via callback read below
    // create a small closure-like struct via new; server will hold pointer - we must free on shutdown in production
    RelayNodeCtx *ctx = malloc(sizeof(RelayNodeCtx));
    ctx->idx = idx;
    ds.write = (UA_DataSourceWriteCallback)writeRelayCallback;
    // we'll pass ctx when creating variable node
    return ds;
}

static UA_StatusCode readRelay(UA_Server *server, const UA_NodeId *sessionId, void *sessionContext,
                               const UA_NodeId *nodeId, void *nodeContext, const UA_NumericRange *range,
                               UA_DataValue *data) {
    (void)server; (void)sessionId; (void)sessionContext; (void)nodeId; (void)range;
    RelayNodeCtx *ctx = (RelayNodeCtx*)nodeContext;
    if (!ctx) return UA_STATUSCODE_BADINTERNALERROR;
    bool val = get_relay_index(ctx->idx);
    UA_Boolean uaVal = val ? UA_TRUE : UA_FALSE;
    UA_Variant_setScalarCopy(&data->value, &uaVal, &UA_TYPES[UA_TYPES_BOOLEAN]);
    data->hasValue = true;
    data->serverTimestamp = UA_DateTime_now();
    data->sourceTimestamp = UA_DateTime_now();
    return UA_STATUSCODE_GOOD;
}

/* ------------------- OPC UA server setup --------------------
   We'll create UA variables under folder /Actuators (and DeviceInfo)
*/
static void create_opcua_nodes(UA_Server *server) {
    // Create Objects folder "Actuators"
    UA_NodeId actuatorsFolderId = UA_NODEID_STRING(1, "Actuators");
    UA_ObjectAttributes objAttr = UA_ObjectAttributes_default;
    objAttr.displayName = UA_LOCALIZEDTEXT("en-US", "Actuators");
    UA_Server_addObjectNode(server, actuatorsFolderId,
                            UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
                            UA_QUALIFIEDNAME(1, "Actuators"),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE),
                            objAttr, NULL, NULL);

    // Create 20 boolean nodes
    for (int i = 0; i < NUM_RELAYS; ++i) {
        char nodename[32];
        char browseName[32];
        snprintf(nodename, sizeof(nodename), "Ch%02d", i+1);
        snprintf(browseName, sizeof(browseName), "Ch%02d", i+1);

        UA_VariableAttributes attr = UA_VariableAttributes_default;
        UA_Boolean initial = get_relay_index(i) ? UA_TRUE : UA_FALSE;
        UA_Variant_setScalarCopy(&attr.value, &initial, &UA_TYPES[UA_TYPES_BOOLEAN]);
        attr.displayName = UA_LOCALIZEDTEXT("en-US", nodename);
        attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;

        UA_NodeId nodeId = UA_NODEID_STRING_ALLOC(1, (char*)nodename);
        UA_QualifiedName qname = UA_QUALIFIEDNAME_ALLOC(1, (char*)browseName);

        // create context
        RelayNodeCtx *ctx = malloc(sizeof(RelayNodeCtx));
        ctx->idx = i;

        UA_Server_addVariableNode(server, nodeId,
                                  actuatorsFolderId,
                                  UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                                  qname,
                                  UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                                  attr, NULL, NULL);

        // Add data source callbacks for read & write
        UA_NodeId varNodeId = nodeId;
        UA_DataSource ds;
        ds.read = (UA_DataSourceReadCallback)readRelay;
        ds.write = (UA_DataSourceWriteCallback)writeRelayCallback;
        // Attach context pointer by variable node's user context using method below:
        UA_Server_setNodeContext(server, varNodeId, ctx);
        UA_Server_setVariableNode_dataSource(server, varNodeId, ds);

        UA_NodeId_clear(&nodeId);
        UA_QualifiedName_clear(&qname);
        UA_VariableAttributes_clear(&attr);
    }

    // DeviceInfo object
    UA_NodeId devInfoId = UA_NODEID_STRING(1, "DeviceInfo");
    UA_ObjectAttributes devAttr = UA_ObjectAttributes_default;
    devAttr.displayName = UA_LOCALIZEDTEXT("en-US", "DeviceInfo");
    UA_Server_addObjectNode(server, devInfoId,
                            UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
                            UA_QUALIFIEDNAME(1, "DeviceInfo"),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE),
                            devAttr, NULL, NULL);

    // Name & FW & Uptime variables
    const char *name = "ESP32-RelayController";
    const char *fw = "v1.0.0";

    UA_VariableAttributes aName = UA_VariableAttributes_default;
    UA_Variant_setScalarCopy(&aName.value, &((UA_String){.length = (UA_UInt16)strlen(name), .data = (UA_Byte*)name}), &UA_TYPES[UA_TYPES_STRING]);
    aName.displayName = UA_LOCALIZEDTEXT("en-US","Name");
    UA_NodeId nameId = UA_NODEID_STRING(1, "DeviceName");
    UA_Server_addVariableNode(server, nameId,
                              devInfoId,
                              UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                              UA_QUALIFIEDNAME(1, "Name"),
                              UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                              aName, NULL, NULL);

    UA_VariableAttributes aFW = UA_VariableAttributes_default;
    UA_Variant_setScalarCopy(&aFW.value, &((UA_String){.length = (UA_UInt16)strlen(fw), .data = (UA_Byte*)fw}), &UA_TYPES[UA_TYPES_STRING]);
    aFW.displayName = UA_LOCALIZEDTEXT("en-US","FW");
    UA_NodeId fwId = UA_NODEID_STRING(1, "Firmware");
    UA_Server_addVariableNode(server, fwId,
                              devInfoId,
                              UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                              UA_QUALIFIEDNAME(1, "FW"),
                              UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                              aFW, NULL, NULL);

    // Uptime variable (dynamic; we will update periodically in server loop)
    UA_VariableAttributes aUp = UA_VariableAttributes_default;
    UA_Double uptime_init = 0.0;
    UA_Variant_setScalarCopy(&aUp.value, &uptime_init, &UA_TYPES[UA_TYPES_DOUBLE]);
    aUp.displayName = UA_LOCALIZEDTEXT("en-US","Uptime");
    aUp.accessLevel = UA_ACCESSLEVELMASK_READ;
    UA_NodeId upId = UA_NODEID_STRING(1, "Uptime");
    UA_Server_addVariableNode(server, upId,
                              devInfoId,
                              UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                              UA_QUALIFIEDNAME(1, "Uptime"),
                              UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                              aUp, NULL, NULL);

    UA_VariableAttributes_clear(&aName);
    UA_VariableAttributes_clear(&aFW);
    UA_VariableAttributes_clear(&aUp);
}

/* OPC UA server task */
static void opcua_server_task(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "Starting OPC UA server (open62541)...");
    UA_ServerConfig *config = UA_ServerConfig_new_default();
    ua_server = UA_Server_new(config);

    // create nodes
    create_opcua_nodes(ua_server);

    // run server (blocking) in this task
    // We will run the server step in a loop to allow other tasks to run
    const int sleep_ms = 50;
    uint32_t loopcounter = 0;
    while (1) {
        UA_Server_run_iterate(ua_server, true);
        // update uptime every second
        if ((loopcounter++ % (1000/sleep_ms)) == 0) {
            time_t now = time(NULL);
            static time_t start = 0;
            if (!start) start = now;
            double uptime = difftime(now, start);
            UA_Double ua_uptime = uptime;
            UA_NodeId upId = UA_NODEID_STRING(1, "Uptime");
            UA_Variant val;
            UA_Variant_setScalarCopy(&val, &ua_uptime, &UA_TYPES[UA_TYPES_DOUBLE]);
            UA_Server_writeValue(ua_server, upId, val);
            UA_Variant_clear(&val);
            UA_NodeId_clear(&upId);
        }
        vTaskDelay(pdMS_TO_TICKS(sleep_ms));
    }

    UA_Server_run_shutdown(ua_server);
    UA_Server_delete(ua_server);
    UA_ServerConfig_delete(config);
    vTaskDelete(NULL);
}

/* ------------------- Ethernet & network handling --------------------
   We'll initialize NVS, TCP/IP stack and Ethernet (W5500) using esp_eth examples.
   For brevity the example uses the esp_eth default init procedure; adapt pins in
   your SDK config or driver init as appropriate.
*/
static void eth_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data) {
    (void)arg; (void)event_data;
    if (event_id == ETHERNET_EVENT_CONNECTED) {
        xSemaphoreTake(net_mutex, portMAX_DELAY);
        net_connected = true;
        xSemaphoreGive(net_mutex);
        ESP_LOGI(TAG, "Ethernet connected");
    } else if (event_id == ETHERNET_EVENT_DISCONNECTED) {
        xSemaphoreTake(net_mutex, portMAX_DELAY);
        net_connected = false;
        xSemaphoreGive(net_mutex);
        ESP_LOGW(TAG, "Ethernet disconnected");
    } else if (event_id == IP_EVENT_ETH_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static void start_ethernet(void) {
    ESP_LOGI(TAG, "Init network (NVS, TCP/IP, ETH)...");
    // initialize NVS and netif
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();

    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);

    // NOTE: use the esp_eth W5500 driver or configure as required.
    // For simplicity, here we call the built-in W5500 example init
    eth_mac_t *mac = NULL;
    eth_phy_t *phy = NULL;
    esp_eth_handle_t eth_handle = NULL;

    // The canonical esp-idf example uses esp_eth_mac_new_esp32() for RMII
    // and a w5500 driver for external chip. Integrate the W5500 driver component
    // and call the driver's init routine here.
    //
    // Because driver setup varies by esp-idf version, please follow the esp-idf
    // W5500 example and set correct SPI pins (SCK/MISO/MOSI/CS) or use menuconfig.
    //
    // For guidance, see: ESP-IDF ethernet docs / W5500 example. :contentReference[oaicite:3]{index=3}

    // Register Ethernet event handlers
    esp_event_handler_instance_t eth_event_inst;
    esp_event_handler_instance_t ip_event_inst;
    esp_event_handler_instance_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL, &eth_event_inst);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &eth_event_handler, NULL, &ip_event_inst);

    // TODO: initialize W5500 driver here and attach to eth_netif
    // Example code is found in esp-idf/examples/protocols/ethernet or W5500 driver repos.
    //
    // For now, log and rely on user to wire driver in project.
    ESP_LOGI(TAG, "Please ensure W5500 driver component is added and initialized.");
}

/* ------------------- LED indicator -------------------- */
static void led_task(void *arg) {
    (void)arg;
    const TickType_t delay_connected = pdMS_TO_TICKS(1000);
    const TickType_t delay_blink = pdMS_TO_TICKS(250);
    while (1) {
        xSemaphoreTake(net_mutex, portMAX_DELAY);
        bool connected = net_connected;
        xSemaphoreGive(net_mutex);
        if (connected) {
            gpio_set_level(PIN_LED_INDICATOR, 1);
            vTaskDelay(delay_connected);
        } else {
            // blink
            gpio_set_level(PIN_LED_INDICATOR, 1);
            vTaskDelay(delay_blink);
            gpio_set_level(PIN_LED_INDICATOR, 0);
            vTaskDelay(delay_blink);
        }
    }
}

/* ------------------- Watchdog pet & monitoring -------------------- */
static void wdt_pet_task(void *arg) {
    (void)arg;
    while (1) {
        esp_task_wdt_reset(); // pet WDT
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ------------------- Network monitor (auto reconnect) --------------------
   If not connected, attempt to re-init or restart ethernet periodically.
*/
static void network_monitor_task(void *arg) {
    (void)arg;
    while (1) {
        xSemaphoreTake(net_mutex, portMAX_DELAY);
        bool connected = net_connected;
        xSemaphoreGive(net_mutex);

        if (!connected) {
            ESP_LOGW(TAG, "Network not connected; attempting to reinit...");
            // In practice: reinitialize ethernet driver or call esp_eth_stop / start
            // Here we just log; integrate W5500-specific reset if needed:
            gpio_set_level(PIN_WIZ_RESET, 0);
            vTaskDelay(pdMS_TO_TICKS(50));
            gpio_set_level(PIN_WIZ_RESET, 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
            // Attempt to restart ethernet stack if implemented
            // TODO: call esp_eth_stop/esp_eth_start on your eth_handle
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* ------------------- app_main -------------------- */
void app_main(void) {
    ESP_LOGI(TAG, "Startup, checking reset reason...");
    esp_reset_reason_t reason = esp_reset_reason();
    if (reason == ESP_RST_BROWNOUT) {
        ESP_LOGW(TAG, "Last reset was a brownout!");
    }

    // initalize nvs (needed for network etc)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // initialize global mutexes
    relays_mutex = xSemaphoreCreateMutex();
    net_mutex = xSemaphoreCreateMutex();

    // initialize relay array
    xSemaphoreTake(relays_mutex, portMAX_DELAY);
    for (int i = 0; i < NUM_RELAYS; ++i) relays[i] = false;
    xSemaphoreGive(relays_mutex);

    // init pins
    gpio_pad_select_gpio(PIN_LED_INDICATOR);
    gpio_set_direction(PIN_LED_INDICATOR, GPIO_MODE_OUTPUT);
    sr_init_pins();

    // WIZ reset pin
    gpio_pad_select_gpio(PIN_WIZ_RESET);
    gpio_set_direction(PIN_WIZ_RESET, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_WIZ_RESET, 1);

    // start ethernet (user must integrate actual W5500 init)
    start_ethernet();

    // create tasks
    // 1) LED indicator
    xTaskCreatePinnedToCore(led_task, "led_task", 2048, NULL, 5, NULL, 1);

    // 2) network monitor
    xTaskCreate(network_monitor_task, "net_mon", 4096, NULL, 5, NULL);

    // 3) Start OPC UA server task
    xTaskCreate(opcua_server_task, "opcua", 16*1024, NULL, 5, NULL);

    // 4) Watchdog: register and start pet task
    esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
    // register current (main) task and create a dedicated WDT pet task
    esp_task_wdt_add(NULL); // register current
    xTaskCreate(wdt_pet_task, "wdt_pet", 2048, NULL, 10, NULL);

    ESP_LOGI(TAG, "Main tasks created. System running.");
}
