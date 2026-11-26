/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * zmk,input-processor-sticky-temp-layer
 *
 * ZMK v0.3 の app/src/pointing/input_processor_temp_layer.c をもとに、
 * 「excluded-positions に挙げたキーを押しても無効化タイマーを延長する」
 * 挙動だけを追加したもの。
 *
 * 元の実装では無効化タイマーを再セットしている箇所が
 * temp_layer_handle_event() の中の 1 行だけで、これは入力リスナー経由で
 * ポインタ入力イベントが来たときしか呼ばれない。キー押下は別経路
 * (handle_position_state_changed というイベントリスナー) で届くが、
 * そこには解除の処理しか無く延長が無かった。
 * そのためボールを止めてレイヤー上のキーを操作し続けても、最後の
 * ポインタ入力からタイムアウト時間でレイヤーが落ちていた。
 *
 * 追加点は state に timeout_ms を保持することと、
 * handle_position_state_changed() の excluded 判定で k_work_reschedule
 * を呼ぶこと。それ以外は元のままなので、require-prior-idle-ms と
 * excluded-positions の意味は変わらない。
 */

#define DT_DRV_COMPAT zmk_input_processor_sticky_temp_layer

#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <drivers/input_processor.h>
#include <zephyr/logging/log.h>
#include <zmk/keymap.h>
#include <zmk/behavior.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/events/layer_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* Constants and Types */
#define MAX_LAYERS ZMK_KEYMAP_LAYERS_LEN

struct temp_layer_config {
    int16_t require_prior_idle_ms;
    /* 発動に必要な累積移動量 (センサーのカウント数)。0 なら無効 = 1 カウントでも発動。 */
    int32_t activation_threshold;
    /* この時間だけ動きが無ければ累積をリセットする。0 なら減衰しない。 */
    int32_t accumulation_timeout_ms;
    const uint16_t *excluded_positions;
    size_t num_positions;
};

struct temp_layer_state {
    uint8_t toggle_layer;
    bool is_active;
    int64_t last_tapped_timestamp;
    /* handle_event で受けた param2 (ミリ秒)。キー押下からの延長に使う。 */
    uint32_t timeout_ms;
    /* activation_threshold 用。発動したら 0 に戻す。 */
    int32_t accumulated_distance;
    int64_t last_movement_timestamp;
};

struct temp_layer_data {
    const struct device *dev;
    struct k_mutex lock;
    struct temp_layer_state state;
};

/* Static Work Queue Items */
static struct k_work_delayable layer_disable_works[MAX_LAYERS];

/* Position Search */
static bool position_is_excluded(const struct temp_layer_config *config, uint32_t position) {
    if (!config->excluded_positions || !config->num_positions) {
        return false;
    }

    const uint16_t *end = config->excluded_positions + config->num_positions;
    for (const uint16_t *pos = config->excluded_positions; pos < end; pos++) {
        if (*pos == position) {
            return true;
        }
    }

    return false;
}

/* Timing Check */
static bool should_quick_tap(const struct temp_layer_config *config, int64_t last_tapped,
                             int64_t current_time) {
    return (last_tapped + config->require_prior_idle_ms) > current_time;
}

/* Layer State Management */
static void update_layer_state(struct temp_layer_state *state, bool activate) {
    if (state->is_active == activate) {
        return;
    }

    state->is_active = activate;
    if (activate) {
        zmk_keymap_layer_activate(state->toggle_layer);
        LOG_DBG("Layer %d activated", state->toggle_layer);
    } else {
        /* 次の発動判定を 0 から始める */
        state->accumulated_distance = 0;
        state->last_movement_timestamp = 0;
        zmk_keymap_layer_deactivate(state->toggle_layer);
        LOG_DBG("Layer %d deactivated", state->toggle_layer);
    }
}

/* 累積移動量がしきい値を超えたかを判定する。
 *
 * X/Y の相対移動イベントごとに |value| を足す。X と Y は別イベントで届くので
 * 合算はマンハッタン距離になる。
 * accumulation_timeout_ms より長く動きが無ければ累積を捨てる。これが無いと、
 * レイヤーが一度も発動しない限り累積が増え続け、わずかなドリフトの積み重ねで
 * いつか発動してしまう。 */
static bool accumulated_movement_reached(const struct temp_layer_config *cfg,
                                         struct temp_layer_state *state,
                                         const struct input_event *event, int64_t now) {
    if (event->type != INPUT_EV_REL ||
        (event->code != INPUT_REL_X && event->code != INPUT_REL_Y)) {
        return false;
    }

    if (cfg->accumulation_timeout_ms > 0 && state->last_movement_timestamp > 0 &&
        (now - state->last_movement_timestamp) > cfg->accumulation_timeout_ms) {
        LOG_DBG("Movement accumulation timed out, resetting");
        state->accumulated_distance = 0;
    }
    state->last_movement_timestamp = now;

    state->accumulated_distance += abs(event->value);
    if (state->accumulated_distance < cfg->activation_threshold) {
        return false;
    }

    state->accumulated_distance = 0;
    return true;
}

struct layer_state_action {
    uint8_t layer;
    bool activate;
};

K_MSGQ_DEFINE(sticky_temp_layer_action_msgq, sizeof(struct layer_state_action),
              CONFIG_ZMK_INPUT_PROCESSOR_STICKY_TEMP_LAYER_MAX_ACTION_EVENTS, 4);

static void layer_action_work_cb(struct k_work *work) {

    const struct device *dev = DEVICE_DT_INST_GET(0);
    struct temp_layer_data *data = (struct temp_layer_data *)dev->data;

    int ret = k_mutex_lock(&data->lock, K_FOREVER);
    if (ret < 0) {
        LOG_ERR("Error locking for updating %d", ret);
        return;
    }

    struct layer_state_action action;

    while (k_msgq_get(&sticky_temp_layer_action_msgq, &action, K_MSEC(10)) >= 0) {
        if (!action.activate) {
            if (zmk_keymap_layer_active(action.layer)) {
                update_layer_state(&data->state, false);
            }
        } else {
            update_layer_state(&data->state, true);
        }
    }

    k_mutex_unlock(&data->lock);
}

static K_WORK_DEFINE(layer_action_work, layer_action_work_cb);

/* Work Queue Callback */
static void layer_disable_callback(struct k_work *work) {
    struct k_work_delayable *d_work = k_work_delayable_from_work(work);
    int layer_index = ARRAY_INDEX(layer_disable_works, d_work);

    struct layer_state_action action = {.layer = layer_index, .activate = false};

    int ret = k_msgq_put(&sticky_temp_layer_action_msgq, &action, K_MSEC(10));
    if (ret < 0) {
        /* 溢れるとこのレイヤーが解除されないまま残る */
        LOG_WRN("Failed to queue layer deactivation for layer %d (%d)", layer_index, ret);
        return;
    }
    k_work_submit(&layer_action_work);
}

/* Event Handlers */
static int handle_layer_state_changed(const struct device *dev, const zmk_event_t *eh) {
    struct temp_layer_data *data = (struct temp_layer_data *)dev->data;
    int ret = k_mutex_lock(&data->lock, K_FOREVER);
    if (ret < 0) {
        return ret;
    }
    if (!zmk_keymap_layer_active(zmk_keymap_layer_index_to_id(data->state.toggle_layer))) {
        LOG_DBG("Deactivating layer that was activated by this processor");
        data->state.is_active = false;
        k_work_cancel_delayable(&layer_disable_works[data->state.toggle_layer]);
    }
    ret = k_mutex_unlock(&data->lock);
    if (ret < 0) {
        return ret;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

static int handle_position_state_changed(const struct device *dev, const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);

    struct temp_layer_data *data = (struct temp_layer_data *)dev->data;
    int ret = k_mutex_lock(&data->lock, K_FOREVER);
    if (ret < 0) {
        return ret;
    }

    const struct temp_layer_config *cfg = dev->config;

    /* 解除は押下のみで判定する (離上で解除すると、レイヤーを抜ける契機に
     * なったキー自身の離上を取りこぼす)。延長は押下・離上の両方で行う。 */
    if (!ev->state && !position_is_excluded(cfg, ev->position)) {
        k_mutex_unlock(&data->lock);
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (data->state.is_active && cfg->excluded_positions && cfg->num_positions > 0) {
        if (!position_is_excluded(cfg, ev->position)) {
            LOG_DBG("Position not excluded, deactivating layer");
            update_layer_state(&data->state, false);
        } else if (data->state.timeout_ms > 0) {
            /* このレイヤーに割り当てられたキーなので解除せず、
             * 無効化タイマーを再セットして寿命を延ばす。 */
            LOG_DBG("Position excluded, extending layer timeout");
            k_work_reschedule(&layer_disable_works[data->state.toggle_layer],
                              K_MSEC(data->state.timeout_ms));
        }
    }

    k_mutex_unlock(&data->lock);

    return ZMK_EV_EVENT_BUBBLE;
}

static int handle_keycode_state_changed(const struct device *dev, const zmk_event_t *eh) {
    const struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);
    if (!ev->state) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    struct temp_layer_data *data = (struct temp_layer_data *)dev->data;

    int ret = k_mutex_lock(&data->lock, K_FOREVER);
    if (ret < 0) {
        return ret;
    }

    LOG_DBG("Setting last_tapped_timestamp to: %lld", ev->timestamp);
    data->state.last_tapped_timestamp = ev->timestamp;

    ret = k_mutex_unlock(&data->lock);
    if (ret < 0) {
        return ret;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

static int handle_state_changed_dispatcher(const struct device *dev, const zmk_event_t *eh) {
    if (as_zmk_layer_state_changed(eh) != NULL) {
        LOG_DBG("Dispatching handle_layer_state_changed");
        return handle_layer_state_changed(dev, eh);
    } else if (as_zmk_position_state_changed(eh) != NULL) {
        LOG_DBG("Dispatching handle_position_state_changed");
        return handle_position_state_changed(dev, eh);
    } else if (as_zmk_keycode_state_changed(eh) != NULL) {
        LOG_DBG("Dispatching handle_keycode_state_changed");
        return handle_keycode_state_changed(dev, eh);
    }

    return ZMK_EV_EVENT_BUBBLE;
}

#define DISPATCH_EVENT(inst)                                                                       \
    {                                                                                              \
        int err = handle_state_changed_dispatcher(DEVICE_DT_INST_GET(inst), eh);                   \
        if (err < 0) {                                                                             \
            return err;                                                                            \
        }                                                                                          \
    }

static int handle_event_dispatcher(const zmk_event_t *eh) {
    DT_INST_FOREACH_STATUS_OKAY(DISPATCH_EVENT)

    return 0;
}

/* Driver Implementation */
static int temp_layer_handle_event(const struct device *dev, struct input_event *event,
                                   uint32_t param1, uint32_t param2,
                                   struct zmk_input_processor_state *state) {
    if (param1 >= MAX_LAYERS) {
        LOG_ERR("Invalid layer index: %d", param1);
        return -EINVAL;
    }

    struct temp_layer_data *data = (struct temp_layer_data *)dev->data;

    int ret = k_mutex_lock(&data->lock, K_FOREVER);
    if (ret < 0) {
        return ret;
    }

    const struct temp_layer_config *cfg = dev->config;

    data->state.toggle_layer = param1;
    data->state.timeout_ms = param2;

    const int64_t now = k_uptime_get();
    bool may_activate =
        !data->state.is_active && !should_quick_tap(cfg, data->state.last_tapped_timestamp, now);

    /* しきい値が設定されている場合は、累積移動量が足りるまで発動させない。
     * 抑制窓 (require-prior-idle-ms) の中では累積もしないので、
     * タイピング中の微振動は蓄積されない。 */
    if (may_activate && cfg->activation_threshold > 0) {
        may_activate = accumulated_movement_reached(cfg, &data->state, event, now);
    }

    if (may_activate) {
        struct layer_state_action action = {.layer = param1, .activate = true};

        int err = k_msgq_put(&sticky_temp_layer_action_msgq, &action, K_MSEC(10));
        if (err < 0) {
            LOG_WRN("Failed to queue layer activation for layer %d (%d)", param1, err);
        } else {
            k_work_submit(&layer_action_work);
        }
    }

    if (param2 > 0) {
        k_work_reschedule(&layer_disable_works[param1], K_MSEC(param2));
    }

    k_mutex_unlock(&data->lock);

    return ZMK_INPUT_PROC_CONTINUE;
}

static int temp_layer_init(const struct device *dev) {
    struct temp_layer_data *data = (struct temp_layer_data *)dev->data;
    k_mutex_init(&data->lock);

    for (int i = 0; i < MAX_LAYERS; i++) {
        k_work_init_delayable(&layer_disable_works[i], layer_disable_callback);
    }

    return 0;
}

/* Driver API */
static const struct zmk_input_processor_driver_api temp_layer_driver_api = {
    .handle_event = temp_layer_handle_event,
};

/* Event Listeners Conditions */
#define NEEDS_POSITION_HANDLERS(n, ...) DT_INST_PROP_HAS_IDX(n, excluded_positions, 0)
#define NEEDS_KEYCODE_HANDLERS(n, ...) (DT_INST_PROP_OR(n, require_prior_idle_ms, 0) > 0)

/* Event Handlers Registration */
ZMK_LISTENER(processor_sticky_temp_layer, handle_event_dispatcher);
ZMK_SUBSCRIPTION(processor_sticky_temp_layer, zmk_layer_state_changed);

/* Individual Subscriptions */
#if DT_INST_FOREACH_STATUS_OKAY_VARGS(NEEDS_POSITION_HANDLERS, ||)
ZMK_SUBSCRIPTION(processor_sticky_temp_layer, zmk_position_state_changed);
#endif

#if DT_INST_FOREACH_STATUS_OKAY_VARGS(NEEDS_KEYCODE_HANDLERS, ||)
ZMK_SUBSCRIPTION(processor_sticky_temp_layer, zmk_keycode_state_changed);
#endif

/* Device Instantiation */
#define TEMP_LAYER_INST(n)                                                                         \
    static struct temp_layer_data processor_sticky_temp_layer_data_##n = {};                              \
    static const uint16_t excluded_positions_##n[] = DT_INST_PROP(n, excluded_positions);          \
    static const struct temp_layer_config processor_sticky_temp_layer_config_##n = {                      \
        .require_prior_idle_ms = DT_INST_PROP_OR(n, require_prior_idle_ms, 0),                     \
        .activation_threshold = DT_INST_PROP_OR(n, activation_threshold, 0),                       \
        .accumulation_timeout_ms = DT_INST_PROP_OR(n, accumulation_timeout_ms, 0),                 \
        .excluded_positions = excluded_positions_##n,                                              \
        .num_positions = DT_INST_PROP_LEN(n, excluded_positions),                                  \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, temp_layer_init, NULL, &processor_sticky_temp_layer_data_##n,                \
                          &processor_sticky_temp_layer_config_##n, POST_KERNEL,                           \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &temp_layer_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TEMP_LAYER_INST)
