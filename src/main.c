#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/cs.h>
#include <bluetooth/services/ras.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/drivers/sensor.h>




#define BT_UUID_CUSTOM_SERVICE_VAL   BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x1234, 0x1234, 0x123456789abc)
#define BT_UUID_CUSTOM_CHAR_VAL      BT_UUID_128_ENCODE(0xabcdef01, 0x2345, 0x3456, 0x4567, 0x56789abcdef0)
#define I2C_NODE              DT_ALIAS(i2c2)
#define BME280_NODE DT_INST(0, bosch_bme280)
#define DEVICE_ID      1


static struct bt_uuid_128 custom_service_uuid = BT_UUID_INIT_128(BT_UUID_CUSTOM_SERVICE_VAL);
static struct bt_uuid_128 custom_char_uuid    = BT_UUID_INIT_128(BT_UUID_CUSTOM_CHAR_VAL);


LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);

static struct bt_le_ext_adv *adv_conn;
static struct bt_le_ext_adv *adv_data;
static K_SEM_DEFINE(sem_connected, 0, 1);

static struct bt_conn *connection;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_RANGING_SERVICE_VAL)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static  char hello_msg[32] = "HelloBLE";
static const struct device *bme280_dev;


static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	(void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected to %s (err 0x%02X)", addr, err);

	if (err) {
		bt_conn_unref(conn);
		connection = NULL;
	}

	connection = bt_conn_ref(conn);
	k_sem_give(&sem_connected);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason 0x%02X)", reason);

	bt_conn_unref(conn);
	connection = NULL;

	sys_reboot(SYS_REBOOT_COLD);
}

static void remote_capabilities_cb(struct bt_conn *conn,
				   uint8_t status,
				   struct bt_conn_le_cs_capabilities *params)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);

	if (status == BT_HCI_ERR_SUCCESS) {
		LOG_INF("CS capability exchange completed.");
	} else {
		LOG_WRN("CS capability exchange failed. (HCI status 0x%02x)", status);
	}
}

static void config_create_cb(struct bt_conn *conn,
			      uint8_t status,
			      struct bt_conn_le_cs_config *config)
{
	ARG_UNUSED(conn);

	if (status == BT_HCI_ERR_SUCCESS) {
		LOG_INF("CS config creation complete. ID: %d", config->id);
	} else {
		LOG_WRN("CS config creation failed. (HCI status 0x%02x)", status);
	}
}

static void security_enable_cb(struct bt_conn *conn, uint8_t status)
{
	ARG_UNUSED(conn);

	if (status == BT_HCI_ERR_SUCCESS) {
		LOG_INF("CS security enabled.");
	} else {
		LOG_WRN("CS security enable failed. (HCI status 0x%02x)", status);
	}
}

static void procedure_enable_cb(struct bt_conn *conn,
				uint8_t status,
				struct bt_conn_le_cs_procedure_enable_complete *params)
{
	ARG_UNUSED(conn);

	if (status == BT_HCI_ERR_SUCCESS) {
		if (params->state == 1) {
			LOG_INF("CS procedures enabled.");
		} else {
			LOG_INF("CS procedures disabled.");
		}
	} else {
		LOG_WRN("CS procedures enable failed. (HCI status 0x%02x)", status);
	}
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
	.le_cs_read_remote_capabilities_complete = remote_capabilities_cb,
	.le_cs_config_complete = config_create_cb,
	.le_cs_security_enable_complete = security_enable_cb,
	.le_cs_procedure_enable_complete = procedure_enable_cb,
};
static ssize_t read_hello_msg(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;
	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, strlen(value));
}

BT_GATT_SERVICE_DEFINE(custom_svc,
	BT_GATT_PRIMARY_SERVICE(&custom_service_uuid),
	BT_GATT_CHARACTERISTIC(&custom_char_uuid.uuid,
                       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                       BT_GATT_PERM_READ,
                       read_hello_msg,
                       NULL,
                       hello_msg),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

int main(void)
{
	int err;

	LOG_INF("Starting Channel Sounding Reflector Sample");

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return 0;
	}
	bme280_dev = DEVICE_DT_GET(BME280_NODE);
	if (!device_is_ready(bme280_dev)) {
		LOG_ERR("❌ BME280 sensor not ready");
		return 0;
	}
	LOG_INF("✅ BME280 sensor initialized");


	// err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), NULL, 0);
	// if (err) {
	// 	LOG_ERR("Advertising failed to start (err %d)", err);
	// 	return 0;
	// }
    struct bt_le_adv_param adv_conn_param = BT_LE_ADV_PARAM_INIT(
        BT_LE_ADV_OPT_CONNECTABLE,
        BT_GAP_ADV_FAST_INT_MIN_2,
        BT_GAP_ADV_FAST_INT_MAX_2,
        NULL
    );

    err = bt_le_ext_adv_create(&adv_conn_param, NULL, &adv_conn);
    if (err) {
        LOG_ERR("Failed to create connectable adv set (%d)", err);
        return 0;
    }

    err = bt_le_ext_adv_set_data(adv_conn, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Failed to set connectable adv data (%d)", err);
        return 0;
    }

    err = bt_le_ext_adv_start(adv_conn, BT_LE_EXT_ADV_START_DEFAULT);
    if (err) {
        LOG_ERR("Failed to start connectable adv (%d)", err);
        return 0;
    }

    LOG_INF("✅ Connectable advertising started");

    char message[] = "HelloBLE";

    struct bt_data data_ad[] = {
        BT_DATA(BT_DATA_MANUFACTURER_DATA, message, strlen(message)),
    };
    // // --- Non-connectable Data Advertising Set (Parallel) ---
    // struct bt_data data_ad[] = {
    //     BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA, 0x01, 0x02, 0x03, 0x04), // Custom payload
    // };

    struct bt_le_adv_param adv_data_param = BT_LE_ADV_PARAM_INIT(
        BT_LE_ADV_OPT_USE_NAME,
        0x00A0, 0x00F0,  // Slower interval
        NULL
    );

    err = bt_le_ext_adv_create(&adv_data_param, NULL, &adv_data);
    if (err) {
        LOG_ERR("Failed to create data adv set (%d)", err);
        return 0;
    }

    err = bt_le_ext_adv_set_data(adv_data, data_ad, ARRAY_SIZE(data_ad), NULL, 0);
    if (err) {
        LOG_ERR("Failed to set data adv data (%d)", err);
        return 0;
    }

    err = bt_le_ext_adv_start(adv_data, BT_LE_EXT_ADV_START_DEFAULT);
    if (err) {
        LOG_ERR("Failed to start data adv (%d)", err);
        return 0;
    }

    LOG_INF("✅ Parallel data advertising started");

	while (true) {
		k_sem_take(&sem_connected, K_FOREVER);

		const struct bt_le_cs_set_default_settings_param default_settings = {
			.enable_initiator_role = false,
			.enable_reflector_role = true,
			.cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
			.max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
		};

		err = bt_le_cs_set_default_settings(connection, &default_settings);
		if (err) {
			LOG_ERR("Failed to configure default CS settings (err %d)", err);
		}
		 while (connection) {
			struct sensor_value temp_val, press_val, hum_val;

			if (sensor_sample_fetch(bme280_dev) == 0 &&
				sensor_channel_get(bme280_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp_val) == 0 &&
				sensor_channel_get(bme280_dev, SENSOR_CHAN_PRESS, &press_val) == 0 &&
				sensor_channel_get(bme280_dev, SENSOR_CHAN_HUMIDITY, &hum_val) == 0) {

				float temp = sensor_value_to_double(&temp_val);
				float press = sensor_value_to_double(&press_val);
				float hum = sensor_value_to_double(&hum_val);

				snprintf(hello_msg, sizeof(hello_msg), "D%dTM%dPR%dHM%d",
					DEVICE_ID,
					(int)(temp * 10),
					(int)(press * 10),
					(int)(hum * 10));
			} else {
				snprintf(hello_msg, sizeof(hello_msg), "TMERR");
				LOG_WRN("⚠️ Failed to read BME280 data");
			}

			err = bt_gatt_notify(NULL, &custom_svc.attrs[1], hello_msg, strlen(hello_msg));
			if (err) {
				LOG_WRN("Notify failed (err %d)", err);
			} else {
				LOG_INF("🔔 Sent: %s", hello_msg);
			}

			k_sleep(K_SECONDS(2));
		}
	}

	return 0;
}
