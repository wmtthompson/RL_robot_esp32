#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "common.h"
#include "Episode.h"
#include "Agent.h"
#include "Simulator.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_random.h"
#include "bootloader_random.h"
#include <cstring>

using namespace std;

//int bus;
//I2CDevice device;
static uint8_t buffer[8] = {0};

Action ac1;
Scan_State ss1;
Agent agent1(60);
StateActionTable sat1;
Episode ep1(&agent1, &sat1);
Simulator sim1(&agent1, &ep1);
static float rew; //reward
void print_action(Action a2);

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       2000

#define ROBOT_MOTORS_ADDR                 0x08        /*!< Slave address of the Motor Controller */


static const char *TAG = "RL Robot";


/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
	conf.clk_flags = 0; //use this to allow to chose which clock to get the desired frequency.


    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    int i2c_master_port = I2C_MASTER_NUM;
	bootloader_random_enable();
	ac1.steer = R;
	ac1.direction = REV;
	ac1.speed = SLOW;
	int i = 1;
	int episode_num = 1;



	while(1)
	{
		printf("HELLO WORLD! \n");
		// read state from robot i2c

	    ESP_ERROR_CHECK(i2c_master_read_from_device(i2c_master_port, ROBOT_MOTORS_ADDR, buffer, sizeof(Scan_State), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS));

	    memcpy((void *)&ss1,(void *)buffer, sizeof(Scan_State));

	    printf("Scan State, left = %d ", ss1.left);
	    printf("right = %d ", ss1.right);
	    printf("center = %d \n", ss1.center);

	    agent1.policy_lookup(ss1, ac1);
	    rew = ep1.reward_of_total_scan(ss1);
	    ep1.rewards[i] = rew;
	    ep1.state_array[i] = ss1;
	    ep1.action_array[i] = ac1;

	    printf("Now sending action \n");
	    print_action(ac1);
	    memcpy((void *)buffer, (void *)&ac1, sizeof(Action));
	    ESP_ERROR_CHECK(i2c_master_write_to_device(i2c_master_port, ROBOT_MOTORS_ADDR, buffer, sizeof(Action), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS));
		i += 1;
		if ((i % EPISODE_LEN) == 0)
		{
//			ep1.report_state_values();
			ep1.evaluate_episode(episode_num);
			episode_num += 1;
		}
		vTaskDelay(2000/ portTICK_PERIOD_MS);
	}


}

void print_action(Action a2)
{
	printf("Action = ");
	switch (a2.direction)
	{
	case FWD:
		printf(" FORWARD ");
		break;
	case REV:
		printf(" REVERSE ");
		break;
	case STOP:
		printf(" STOP ");
		break;
	default:
		break;
	}

	switch (a2.speed)
	{
	case SLOW:
		printf(" SLOW ");
		break;
	case FAST:
		printf(" FAST ");
		break;
	default:
		break;
	}

	switch (a2.steer)
	{
	case L:
		printf(" LEFT ");
		break;
	case R:
		printf(" RIGHT ");
		break;
	case S:
		printf(" CENTER ");
		break;
	default:
		break;
	}

	printf("\n");
}
