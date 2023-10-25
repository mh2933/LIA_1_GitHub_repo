#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/drivers/pwm.h>

LOG_MODULE_REGISTER(ble_test);

volatile bool ble_ready = false;
static bool led_state = 0;

#define LED DT_ALIAS(led0)

#define BT_UUID_MY_CUSTOM_SERV_VAL BT_UUID_128_ENCODE(0x347F1281,0x56EE,0x488B,0xA55A,0x4AFBF234FA8C)
#define BT_UUID_MY_CUSTOM_SERVICE BT_UUID_DECLARE_128(BT_UUID_MY_CUSTOM_SERV_VAL)

#define BT_UUID_LED_STATE_CHRC_VAL BT_UUID_128_ENCODE(0x347F1281,0x56EE,0x488B,0xA55A,0x4AFBF234FA8D)
#define BT_UUID_LED_STATE_CHRC BT_UUID_DECLARE_128(BT_UUID_LED_STATE_CHRC_VAL)

static const struct pwm_dt_spec pwm_led0 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED, gpios);

static const struct bt_data ad[] = 
{
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_MY_CUSTOM_SERV_VAL)
};

static ssize_t read_led_state(struct bt_conn *conn,
									 const struct bt_gatt_attr * attr, void *buf,
									 uint16_t len, uint16_t offset);

static ssize_t write_led_state(struct bt_conn *conn,
									 const struct bt_gatt_attr * attr, void *buf,
									 uint16_t len, uint16_t offset, uint8_t flags);

BT_GATT_SERVICE_DEFINE(custom_srv,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_MY_CUSTOM_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_LED_STATE_CHRC, (BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE), 
	(BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), read_led_state, write_led_state, &led_state),
	
	);

static ssize_t read_led_state(struct bt_conn *conn,
									 const struct bt_gatt_attr * attr, void *buf,
									 uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &led_state, sizeof(led_state));
}

static ssize_t write_led_state(struct bt_conn *conn,
									 const struct bt_gatt_attr * attr, void *buf,
									 uint16_t len, uint16_t offset, uint8_t flags)
{
	memcpy(&led_state, buf, sizeof(led_state));

	    if (len != sizeof(led_state))
    {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

	if (led_state)
	{
		gpio_pin_set_dt(&led, 1);	
		pwm_set_pulse_dt(&pwm_led0, 4000000);

	}
	else
	{
		gpio_pin_set_dt(&led, 0);
		pwm_set_pulse_dt(&pwm_led0, 1000000);
	}
	return len;
}

void bt_ready(int err)
{
	if (err)
	{
		LOG_ERR("bt enable return %d", err);
	}
	LOG_INF("bt ready!");
	ble_ready = true;
}

int init_ble(void)
{
	LOG_INF("Init BLE");
	int err;

	err = bt_enable(bt_ready);
	if (err)
	{
		LOG_ERR("bt_enable failed (err %d)", err);
		return err;
	}

}

void main(void)
{
	init_ble();

	while(!ble_ready)
	{
		LOG_INF("BLE stack not ready yet");
		k_msleep(100);
	}
	LOG_INF("BLE stack ready");

        int err;
        err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
        if (err)
        {
                printk("Advertising failed to start (err %d)\n", err);
        }

		err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
		if (err < 0)
		{
			LOG_INF("LED not configures");
		}


		while(1)
		{
			//k_msleep(2000);

		}
}
