#include <stdio.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"

static const char *TAG = "i2c-Temperature";
static const char *TAG2 = "Partition";
static const char *TAG3 = "Timer High Precission";
SemaphoreHandle_t print_mux = NULL;
// definimos la variable
esp_timer_handle_t periodic_timer; 
float new_raw_temperature = 0;
int timer = 1000000;
int stop = 0;


// Handle of the wear levelling library instance
static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

// Mount path for the partition
const char *base_path = "/spiflash";


// Adress SI7021
#define SI7021_ADDR					0x40
#define WRITE_BIT 					I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT 					I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 				0x1                        /*!< I2C master will check ack from slave*/
#define ACK_VAL 					0x0                             /*!< I2C ack value */
#define NACK_VAL 					0x1                            /*!< I2C nack value */

//
#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM		        CONFIG_I2C_MASTER_PORT_NUM /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 	0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 	0                           /*!< I2C master doesn't need buffer */

//return values
#define SI7021_ERR_NOTFOUND				0x03
#define TRIGGER_TEMP_MEASURE_NOHOLD  	0xF3

// Config master
static esp_err_t __attribute__((unused)) i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SI7021_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Config slave

static esp_err_t __attribute__((unused)) i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SI7021_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

//configuration
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,   /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

//Comunication master slave

static esp_err_t i2c_master_sensor_test(i2c_port_t i2c_num, uint8_t *data_h, uint8_t *data_l)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SI7021_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, TRIGGER_TEMP_MEASURE_NOHOLD, ACK_CHECK_EN); //
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(30 / portTICK_RATE_MS);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SI7021_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_h, ACK_VAL);
    i2c_master_read_byte(cmd, data_l, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static void i2c_test_task(void *arg)
{
    int ret;
	uint16_t raw_temperature = 0;
    uint8_t sensor_data_h, sensor_data_l;

    while (1) {
        //ESP_LOGI(TAG, "TASK[%d] test cnt: %d",);
        ret = i2c_master_sensor_test(I2C_MASTER_NUM, &sensor_data_h, &sensor_data_l);
        xSemaphoreTake(print_mux, portMAX_DELAY);
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C Timeout");
        } else if (ret == ESP_OK) {

            //ESP_LOGW(TAG," MASTER READ SENSOR( SI7021 )\n");
            //printf("data_h: %02x\n", sensor_data_h);
            //printf("data_l: %02x\n", sensor_data_l);
			raw_temperature = (sensor_data_h << 8 | sensor_data_l);

			if(raw_temperature == 0){
				printf("- 999");
			}
            new_raw_temperature = ((raw_temperature * 175.72 / 65536.0) - 46.85);
            //printf("Temperature: %.02f [^C]\n", (raw_temperature * 175.72 / 65536.0) - 46.85);
        } else {
            ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
        }
    }

    vTaskDelete(NULL);
}


static void periodic_timer_callback(void* arg)
{
    stop = stop + timer * 10;
    printf("TIME ELAPSED: %d seconds\n", stop/1000000);
    ESP_LOGI(TAG2, "Opening file");
    FILE *f = fopen("/spiflash/si7021.txt", "wb");
    if (f == NULL) {
        ESP_LOGE(TAG2, "Failed to open file for writing");
        return;
    }
    fprintf(f, "written using ESP-IDF %.02f\n", new_raw_temperature);
    fclose(f);
    ESP_LOGI(TAG2, "File written");

    

    // Unmount FATFS
    //ESP_LOGI(TAG2, "Unmounting FAT filesystem");
    //ESP_ERROR_CHECK( esp_vfs_fat_spiflash_unmount(base_path, s_wl_handle));

    ESP_LOGI(TAG2, "Done");

    if (stop == 60000000)

    {
        printf("STOP TIMER HIGH PRECISSION \n");
        ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));
        ESP_ERROR_CHECK(esp_timer_delete(periodic_timer));
        ESP_LOGI(TAG3, "Stopped and deleted timers to %i seconds", stop/1000000);

        // Open file for reading
        ESP_LOGI(TAG2, "Reading file");
        f = fopen("/spiflash/si7021.txt", "rb");
        if (f == NULL) {
            ESP_LOGE(TAG2, "Failed to open file for reading");
            return;
        }
        char line[128];
        fgets(line, sizeof(line), f);
        fclose(f);
        // strip newline
        char *pos = strchr(line, '\n');
        if (pos) {
            *pos = '\0',
            *pos = '\1',
            *pos = '\2',
            *pos = '\3',
            *pos = '\4';
        }
        ESP_LOGI(TAG2, "Read from file '%s': '%s'", pos, line);
        ESP_LOGI(TAG2, "Read from file '%s': '%s'", pos, line);
        ESP_LOGI(TAG2, "Read from file '%s': '%s'", pos, line);
        ESP_LOGI(TAG2, "Read from file '%s': '%s'", pos, line);
        ESP_LOGI(TAG2, "Read from file '%s': '%s'", pos, line);

        // Unmount FATFS
        ESP_LOGI(TAG2, "Unmounting FAT filesystem");
        ESP_ERROR_CHECK( esp_vfs_fat_spiflash_unmount(base_path, s_wl_handle));
    } 

}

static void timer_High_Precission(){
    //int time = 1000000;
    //int stop = 0;
    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &periodic_timer_callback,
            /* el nombre es opcional, pero puede ayudar a identificar el temporizador al depurar */
            .name = "periodic"
    };
    //esp_timer_handle_t periodic_timer; // definimos la variable
    
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, timer * 10));        
    
    
}


void app_main(void)
{
    ESP_LOGI(TAG, "Mounting FAT filesystem");
    // To mount device we need name of device partition, define base_path
    // and allow format partition in case if it is new one and was not formated before
    const esp_vfs_fat_mount_config_t mount_config = {
            .max_files = 4,
            .format_if_mount_failed = true,
            .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount(base_path, "storage", &mount_config, &s_wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return;
    }

    timer_High_Precission();
    print_mux = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreate(i2c_test_task, "i2c_test_task_1", 1024 * 2, (void *)1, 10, NULL);
    vprintf_like_t esp_log_set_vprintf(vprintf_like_t i2c_test_task);

}