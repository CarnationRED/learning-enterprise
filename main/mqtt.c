#ifndef __MQTT_C__
#define __MQTT_C__

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "esp_wifi.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "mqtt.h"

static const char *TAG = "MQTT_EXAMPLE";
#define tup "/topic/up"
#define tdown "/topic/down"
#define trprt "/topic/rprt"
#define tctrl "/topic/ctrl"

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}
static esp_mqtt_client_handle_t client = NULL;
bool subscribed = false;
bool mqttconnected = false;
void (*dataDownHandler)(int len, uint8_t *data) = 0;
void (*dataCtrlHandler)(int len, uint8_t *data) = 0;
/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
    mqttconnected=true;
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, trprt, "VCI logged on", 0, 2, 1);
        ESP_LOGI(TAG, "sent publish, msg_id=%d", msg_id);

        // msg_id = esp_mqtt_client_subscribe(client, trprt, 2);
        // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, tctrl, 2);
        ESP_LOGI(TAG, "sent subscribe, msg_id=%d", msg_id);

        // msg_id = esp_mqtt_client_subscribe(client, tup, 2);
        // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, tdown, 2);
        ESP_LOGI(TAG, "sent subscribe, msg_id=%d", msg_id);

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        // esp_mqtt_client_disconnect(client);
        esp_wifi_disconnect();
        esp_wifi_connect();
        //  esp_mqtt_client_stop(client);
        //  esp_mqtt_client_start(client);
        subscribed = false;
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, trprt, "VCI topic subscribed", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish, msg_id=%d", msg_id);
        subscribed = true;
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        subscribed = false;
        break;
    case MQTT_EVENT_PUBLISHED:
        // ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        if (strcmp(event->topic, tdown)==0 && dataDownHandler != NULL)
        {
            dataDownHandler(event->data_len, (uint8_t *)event->data);
        }
        else if (strcmp(event->topic, tctrl)==0 && dataCtrlHandler != NULL)
        {
            dataCtrlHandler(event->data_len, (uint8_t *)event->data);
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}
static void mqtt_app_start(void)
{
    if (client)
    {
        esp_mqtt_client_stop(client);
        esp_mqtt_client_destroy(client);
    }
    subscribed = false;
    esp_mqtt_client_config_t mqtt_cfg = {
        .session.protocol_ver = MQTT_PROTOCOL_V_3_1_1,
        .broker.address.uri = "mqtt://esp32s3:pwd@192.168.4.2:2222",
        // .session.keepalive = 2, // 1s
        .session.disable_keepalive = false, // 1s
        // .broker.address.port = 2222,
        .buffer.size = 4096,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void (*mqtt_start)(void) = &mqtt_app_start;

void mqtt_dataUp_pulish(char *data, int len)
{
    int msg_id;
    msg_id = esp_mqtt_client_publish(client, tup, data, len, 2, 0);
    //ESP_LOGI(TAG, "sent publish, msg_id=%d", msg_id);
}

void mqtt_report_pulish(char *data, int len)
{
    int msg_id;
    msg_id = esp_mqtt_client_publish(client, trprt, data, len, 2, 0);
    //ESP_LOGI(TAG, "sent publish, msg_id=%d", msg_id);
}

#endif // !__MQTT_C__