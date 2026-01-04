#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include "bpm_ble.h"

/* HR (0x180D/0x2A37) */
static struct bt_uuid_16 hrs_uuid = BT_UUID_INIT_16(0x180D);
static struct bt_uuid_16 hrm_uuid = BT_UUID_INIT_16(0x2A37);

static bool hrm_notify_enabled;
static uint8_t hrm_payload[2];

static void hrm_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    hrm_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

BT_GATT_SERVICE_DEFINE(hrs_svc,
    BT_GATT_PRIMARY_SERVICE(&hrs_uuid.uuid),
    BT_GATT_CHARACTERISTIC(&hrm_uuid.uuid,
        BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_NONE,
        NULL, NULL, hrm_payload),
    BT_GATT_CCC(hrm_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

static int hrs_notify(struct bt_conn *conn, uint16_t bpm, bool worn)
{
    if (!conn || !hrm_notify_enabled) {
        return 0;
    }

    hrm_payload[0] = 0x00;
    hrm_payload[0] |= (1U << 1);
    if (worn) {
        hrm_payload[0] |= (1U << 2);
    }
    hrm_payload[1] = (uint8_t)MIN(bpm, 255);

    return bt_gatt_notify(conn, &hrs_svc.attrs[2], hrm_payload, sizeof(hrm_payload));
}

/* PLX (0x1822/0x2A5F + 0x2A60) */
static struct bt_uuid_16 plx_svc_uuid  = BT_UUID_INIT_16(0x1822);
static struct bt_uuid_16 plx_cont_uuid = BT_UUID_INIT_16(0x2A5F);
static struct bt_uuid_16 plx_feat_uuid = BT_UUID_INIT_16(0x2A60);

static bool plx_notify_enabled;
static uint8_t plx_payload[5];
static uint16_t plx_features_le = 0x0000;

static void plx_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    plx_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static ssize_t read_plx_features(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 void *buf, uint16_t len, uint16_t offset)
{
    ARG_UNUSED(attr);
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &plx_features_le, sizeof(plx_features_le));
}

static uint16_t sfloat_encode_int(int16_t mantissa, int8_t exponent)
{
    uint16_t m = (uint16_t)mantissa & 0x0FFF;
    uint16_t e = ((uint16_t)exponent & 0x000F) << 12;
    return (uint16_t)(e | m);
}

BT_GATT_SERVICE_DEFINE(plx_svc,
    BT_GATT_PRIMARY_SERVICE(&plx_svc_uuid.uuid),

    BT_GATT_CHARACTERISTIC(&plx_cont_uuid.uuid,
        BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_NONE,
        NULL, NULL, plx_payload),
    BT_GATT_CCC(plx_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&plx_feat_uuid.uuid,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        read_plx_features, NULL, &plx_features_le)
);

static int plx_notify(struct bt_conn *conn, uint8_t spo2_percent, uint16_t pulse_rate_bpm)
{
    if (!conn || !plx_notify_enabled) {
        return 0;
    }

    plx_payload[0] = 0x00;

    uint16_t spo2_sf = sfloat_encode_int((int16_t)spo2_percent, 0);
    uint16_t pr_sf   = sfloat_encode_int((int16_t)MIN(pulse_rate_bpm, 300), 0);

    sys_put_le16(spo2_sf, &plx_payload[1]);
    sys_put_le16(pr_sf,   &plx_payload[3]);

    return bt_gatt_notify(conn, &plx_svc.attrs[2], plx_payload, sizeof(plx_payload));
}

/* Custom */
#define BT_UUID_BPM_SPO2_SVC_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

#define BT_UUID_SPO2_CHAR_VAL \
    BT_UUID_128_ENCODE(0x1234567a, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

#define BT_UUID_ALARM_CHAR_VAL \
    BT_UUID_128_ENCODE(0x1234567b, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

static struct bt_uuid_128 custom_svc_uuid = BT_UUID_INIT_128(BT_UUID_BPM_SPO2_SVC_VAL);
static struct bt_uuid_128 spo2_uuid       = BT_UUID_INIT_128(BT_UUID_SPO2_CHAR_VAL);
static struct bt_uuid_128 alarm_uuid      = BT_UUID_INIT_128(BT_UUID_ALARM_CHAR_VAL);

static uint8_t g_spo2;
static uint8_t g_alarm;

static bool spo2_notify_enabled;
static bool alarm_notify_enabled;

static void spo2_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    spo2_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static void alarm_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    alarm_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

BT_GATT_SERVICE_DEFINE(custom_svc,
    BT_GATT_PRIMARY_SERVICE(&custom_svc_uuid),

    BT_GATT_CHARACTERISTIC(&spo2_uuid.uuid,
        BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_NONE,
        NULL, NULL, &g_spo2),
    BT_GATT_CCC(spo2_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&alarm_uuid.uuid,
        BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_NONE,
        NULL, NULL, &g_alarm),
    BT_GATT_CCC(alarm_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

/* Adv + conn */
static struct bt_conn *current_conn;

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x0D, 0x18, 0x22, 0x18),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_BPM_SPO2_SVC_VAL),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static const struct bt_le_adv_param adv_param = {
    .id = BT_ID_DEFAULT,
    .options = BT_LE_ADV_OPT_CONN,
    .interval_min = 0x0640,
    .interval_max = 0x06A4,
    .peer = NULL,
};

static int start_advertising(void)
{
    int err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err == -EALREADY) {
        return 0;
    }
    return err;
}

static void adv_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(adv_work, adv_work_handler);

static void adv_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    (void)start_advertising();
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        return;
    }

    if (current_conn) {
        bt_conn_unref(current_conn);
    }
    current_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(reason);

    hrm_notify_enabled = false;
    plx_notify_enabled = false;
    spo2_notify_enabled = false;
    alarm_notify_enabled = false;

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    (void)k_work_schedule(&adv_work, K_MSEC(200));
}

BT_CONN_CB_DEFINE(conn_cb) = {
    .connected = connected,
    .disconnected = disconnected,
};

void ble_init(void)
{
    int err = bt_enable(NULL);
    if (err) {
        return;
    }

    g_spo2 = 0;
    g_alarm = 0;

    plx_features_le = sys_cpu_to_le16(plx_features_le);

    (void)start_advertising();
}

int ble_update(uint16_t bpm, uint8_t spo2, bool worn)
{
    int err;

    g_spo2 = spo2;

    if (!current_conn) {
        return 0;
    }

    err = hrs_notify(current_conn, bpm, worn);
    if (err == -ENOMEM || err == -EAGAIN) {
        err = 0;
    }
    if (err) {
        return err;
    }

    err = plx_notify(current_conn, spo2, bpm);
    if (err == -ENOMEM || err == -EAGAIN) {
        err = 0;
    }
    if (err) {
        return err;
    }

    if (spo2_notify_enabled) {
        err = bt_gatt_notify_uuid(current_conn, &spo2_uuid.uuid,
                                  custom_svc.attrs, &g_spo2, sizeof(g_spo2));
        if (err == -ENOMEM || err == -EAGAIN) {
            err = 0;
        }
        if (err) {
            return err;
        }
    }

    return 0;
}

int ble_alarm_update(uint8_t alarm)
{
    int err;

    alarm = alarm ? 1U : 0U;
    if (g_alarm == alarm) {
        return 0;
    }

    g_alarm = alarm;

    if (!current_conn || !alarm_notify_enabled) {
        return 0;
    }

    err = bt_gatt_notify_uuid(current_conn, &alarm_uuid.uuid,
                              custom_svc.attrs, &g_alarm, sizeof(g_alarm));
    if (err == -ENOMEM || err == -EAGAIN) {
        return 0;
    }

    return err;
}
