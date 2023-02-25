#include <driver/gpio.h>
#include <driver/uart.h>
#include <errno.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <inttypes.h>
#include <lwip/api.h>
#include <lwip/err.h>
#include <lwip/netdb.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <nvs_flash.h>
#include <sdkconfig.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define WIFI_SSID CONFIG_PSX_WIFI_SSID
#define WIFI_PASS CONFIG_PSX_WIFI_PASS

#define BUFFER_SIZE (10 * 1024)

#if defined(CONFIG_PSX_SERIAL_PC_UART_0)
#define PC_UART UART_NUM_0
#elif defined(CONFIG_PSX_SERIAL_PC_UART_1)
#define PC_UART UART_NUM_1
#elif defined(CONFIG_PSX_SERIAL_PC_UART_2)
#define PC_UART UART_NUM_2
#endif

#if defined(CONFIG_PSX_SERIAL_PSX_UART_1)
#define PSX_UART UART_NUM_1
#elif defined(CONFIG_PSX_SERIAL_PSX_UART_2)
#define PSX_UART UART_NUM_2
#endif

#if PC_UART == PSX_UART
#error "You can't use the same uart port for both the pc and psx"
#endif

#if CONFIG_PSX_TCP_PORT_LOGS == CONFIG_PSX_TCP_PORT
#error "You can't use the same tcp port for both forwarding uart and esp debug"
#endif

#if defined(CONFIG_PSX_SERIAL_PC_BAUD_9600)
#define PC_BAUD 9600
#elif defined(CONFIG_PSX_SERIAL_PC_BAUD_115200)
#define PC_BAUD 115200
#elif defined(CONFIG_PSX_SERIAL_PC_BAUD_460800)
#define PC_BAUD 460800
#elif defined(CONFIG_PSX_SERIAL_PC_BAUD_510000)
#define PC_BAUD 510000
#elif defined(CONFIG_PSX_SERIAL_PC_BAUD_921600)
#define PC_BAUD 921600
#elif defined(PSX_SERIAL_PC_BAUD_2000000)
#define PC_BAUD 20000000
#else
#error "Bad pc baudrate"
#endif

#ifdef CONFIG_PSX_ESP_DEBUG_LOGS
#if defined(CONFIG_PSX_ESP_DEBUG_LOGS_WARNING)
#define TCP_LOG_LEVEL ESP_LOG_WARN
#elif defined(CONFIG_PSX_ESP_DEBUG_LOGS_ERROR)
#define TCP_LOG_LEVEL ESP_LOG_ERROR
#elif defined(CONFIG_PSX_ESP_DEBUG_LOGS_INFO)
#define TCP_LOG_LEVEL ESP_LOG_INFO
#elif defined(CONFIG_PSX_ESP_DEBUG_LOGS_DEBUG)
#define TCP_LOG_LEVEL ESP_LOG_DEBUG
#elif defined(CONFIG_PSX_ESP_DEBUG_LOGS_VERBOSE)
#define TCP_LOG_LEVEL ESP_LOG_VERBOSE
#else
#error "Bad log level"
#endif
#endif

#if defined(CONFIG_PSX_SERIAL_PSX_BAUD_9600)
#define PSX_BAUD 9600
#elif defined(CONFIG_PSX_SERIAL_PSX_BAUD_115200)
#define PSX_BAUD 115200
#elif defined(CONFIG_PSX_SERIAL_PSX_BAUD_518400)
#define PSX_BAUD 518400
#else
#error "Bad psx baudrate"
#endif

#if CONFIG_PSX_SERIAL_PC_STOP_BITS == 1
#define PC_STOP_BITS UART_STOP_BITS_1
#elif CONFIG_PSX_SERIAL_PC_STOP_BITS == 2
#define PC_STOP_BITS UART_STOP_BITS_2
#else
#error "Bad pc stop bits"
#endif

#if CONFIG_PSX_SERIAL_PSX_STOP_BITS == 1
#define PSX_STOP_BITS UART_STOP_BITS_1
#elif CONFIG_PSX_SERIAL_PSX_STOP_BITS == 2
#define PSX_STOP_BITS UART_STOP_BITS_2
#else
#error "Bad psx stop bits"
#endif

static const char* TAG = "ESP_PSX_TOOLS";

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && (event_id == WIFI_EVENT_STA_DISCONNECTED || event_id == WIFI_EVENT_STA_START)) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static void setup_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void setup_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

static void setup_serial(void)
{
    uart_config_t uart_config_pc = {
        .baud_rate = PC_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = PC_STOP_BITS,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_param_config(PC_UART, &uart_config_pc);
    uart_driver_install(PC_UART, BUFFER_SIZE, BUFFER_SIZE, 0, NULL, ESP_INTR_FLAG_IRAM);
    uart_set_pin(PC_UART, CONFIG_PSX_SERIAL_PC_TX, CONFIG_PSX_SERIAL_PC_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_config_t uart_config_psx = {
        .baud_rate = PSX_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = PSX_STOP_BITS,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_param_config(PSX_UART, &uart_config_psx);
    uart_driver_install(PSX_UART, BUFFER_SIZE, BUFFER_SIZE, 0, NULL, ESP_INTR_FLAG_IRAM);
    uart_set_pin(PSX_UART, CONFIG_PSX_SERIAL_PSX_TX, CONFIG_PSX_SERIAL_PSX_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void set_led_brightness(size_t len)
{
#if CONFIG_PSX_SERIAL_LED != -1
    gpio_set_level(CONFIG_PSX_SERIAL_LED, !len ^ CONFIG_PSX_SERIAL_LED_INVERT);
#endif
}

static void listen_tcp_non_blocking(int* return_listen_sock, size_t port)
{
    struct sockaddr_storage dest_addr;
    struct sockaddr_in* dest_addr_ip4 = (struct sockaddr_in*)&dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(port);

    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    assert(listen_sock > 0);
    int flags = fcntl(listen_sock, F_GETFL);
    if (fcntl(listen_sock, F_SETFL, flags | O_NONBLOCK) == -1) {
        assert(false);
    }

    int err = bind(listen_sock, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
    assert(err == 0);
    err = listen(listen_sock, 1);
    assert(err == 0);
    *return_listen_sock = listen_sock;
}

static void tcp_accept_new_connections(int listensock, int* sock)
{
    if (*sock != -1) {
        return;
    }

    int keepAlive = 1;
    int keepIdle = 4;
    int keepInterval = 4;
    int keepCount = 4;
    struct sockaddr_storage source_addr;
    socklen_t addr_len = sizeof(source_addr);
    *sock = accept(listensock, (struct sockaddr*)&source_addr, &addr_len);
    if (*sock > 0) {
        setsockopt(*sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(*sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(*sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(*sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        ESP_LOGI(TAG, "accept successful");
    }
}

static void tcp_write_bytes(int* sock, const uint8_t* buf, size_t len)
{
    if (*sock == -1) {
        return;
    }

    int to_write = len;
    while (to_write > 0) {
        int written = send(*sock, buf + (len - to_write), to_write, 0);
        if (written < 0 && errno != EINPROGRESS && errno != EAGAIN && errno != EWOULDBLOCK) {
            *sock = -1;
            return;
        }
        to_write -= written;
    }
}

static size_t tcp_read_bytes(int* sock, uint8_t* buf, size_t maxlen)
{
    if (*sock == -1) {
        return 0;
    }
    int len = recv(*sock, buf, maxlen, MSG_DONTWAIT);
    if (len < 0) {
        if (errno == EINPROGRESS || errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        *sock = -1;
        return 0;
    }
    return len;
}

static void write_uart(uart_port_t uart_port, const void* buf, size_t len)
{
    size_t to_write = len;
    while (to_write > 0) {
        to_write -= uart_write_bytes(uart_port, buf + (len - to_write), to_write);
    }
}

#ifdef CONFIG_PSX_ESP_DEBUG_LOGS
static const char* TRUNCATE_TAIL = "...\n";
static int sock_debug = -1;

static int tcplogger(const char* message, va_list fmt)
{
    if (sock_debug == -1) {
        return 0;
    }
    char buff[256];
    int written = vsnprintf(buff, sizeof(buff), message, fmt);

    if (written >= sizeof(buff)) {
        char* tail_begin = buff + sizeof(buff) - strlen(TRUNCATE_TAIL) - 1;
        memcpy(tail_begin, TRUNCATE_TAIL, strlen(TRUNCATE_TAIL));
        written = sizeof(buff) - 1;
    }
    tcp_write_bytes(&sock_debug, (uint8_t*)buff, written);
    return written;
}
#endif

static void debug_data(const char* source, const uint8_t* data, size_t len)
{
#ifdef CONFIG_PSX_ESP_DEBUG_LOGS
    if (len) {
        ESP_LOGI(TAG, "from %s len %d", source, len);
        ESP_LOG_BUFFER_HEX(TAG, data, len);
    }
#endif
}

static void handle_debug_input()
{
#ifdef CONFIG_PSX_ESP_DEBUG_LOGS
    uint8_t t;
    if (tcp_read_bytes(&sock_debug, &t, 1)) {
        switch (t) {
        case 'E':
        case 'e':
            esp_log_level_set("*", ESP_LOG_ERROR);
            break;
        case 'W':
        case 'w':
            esp_log_level_set("*", ESP_LOG_WARN);
            break;
        case 'I':
        case 'i':
            esp_log_level_set("*", ESP_LOG_INFO);
            break;
        case 'D':
        case 'd':
            esp_log_level_set("*", ESP_LOG_DEBUG);
            break;
        case 'V':
        case 'v':
            esp_log_level_set("*", ESP_LOG_DEBUG);
            break;
        default:
            ESP_LOGI(TAG, "Bad data %c", t);
            break;
        }
    }
#endif
}

static void mainloop(void)
{
    uint8_t* output_buffer = malloc(BUFFER_SIZE);
    size_t len;
    int listensock = -1;
    int sock = -1;
    listen_tcp_non_blocking(&listensock, CONFIG_PSX_TCP_PORT);
    assert(listensock != -1);
#ifdef CONFIG_PSX_ESP_DEBUG_LOGS
    int listensock_debug = -1;
    listen_tcp_non_blocking(&listensock_debug, CONFIG_PSX_TCP_PORT_LOGS);
    assert(listensock_debug != -1);
#endif

    while (1) {

        len = uart_read_bytes(PC_UART, output_buffer, BUFFER_SIZE, 10 / portTICK_RATE_MS);
        debug_data("pc uart", output_buffer, len);
        set_led_brightness(len);
        write_uart(PSX_UART, output_buffer, len);

        len = tcp_read_bytes(&sock, output_buffer, BUFFER_SIZE);
        debug_data("tcp", output_buffer, len);
        set_led_brightness(len);
        write_uart(PSX_UART, output_buffer, len);

        len = uart_read_bytes(PSX_UART, output_buffer, BUFFER_SIZE, 10 / portTICK_RATE_MS);
        debug_data("psx uart", output_buffer, len);
        set_led_brightness(len);
        write_uart(PC_UART, output_buffer, len);
        tcp_write_bytes(&sock, output_buffer, len);

        tcp_accept_new_connections(listensock, &sock);
#ifdef CONFIG_PSX_ESP_DEBUG_LOGS
        tcp_accept_new_connections(listensock_debug, &sock_debug);
        handle_debug_input();
#endif
    }
}

static void setup_led(void)
{
#if CONFIG_PSX_SERIAL_LED != -1
    gpio_pad_select_gpio(CONFIG_PSX_SERIAL_LED);
    gpio_set_direction(CONFIG_PSX_SERIAL_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_PSX_SERIAL_LED, !CONFIG_PSX_SERIAL_LED_INVERT);
#endif
}

static void setup_logging(void)
{
#ifdef CONFIG_PSX_ESP_DEBUG_LOGS
    esp_log_set_vprintf(tcplogger);
    esp_log_level_set("*", TCP_LOG_LEVEL);
#endif
}

void app_main(void)
{
    setup_nvs();
    setup_led();
    setup_wifi();

    setup_logging();
    setup_serial();

    mainloop();
}

// FIXME:
// add baud rate auto detection
// add ability for the esp32 to host the wifi if no wifi found or if configured in ap mode explicitly
// add web page for on the fly configuration changes
// add web page for drag and drop/ upload of exe
// add support for logs via uart, assuming is not one of the uart in use
// test that if wifi disconnects and reconnects the tcp listen still works
