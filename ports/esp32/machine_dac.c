/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Nick Moore
 * Copyright (c) 2025 Nicasio Sirvent
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#include <stdio.h>
#include <math.h>

#include "py/runtime.h"
#include "py/mphal.h"
#include "py/objarray.h"
#include "modmachine.h"

#if MICROPY_PY_MACHINE_DAC

#include "driver/gpio.h"
#include "driver/dac_oneshot.h"
#include "driver/dac_continuous.h"
#include "driver/dac_cosine.h"
#include "esp_check.h"

// DAC operating modes
typedef enum {
    DAC_MODE_ONESHOT,     // Basic voltage output
    DAC_MODE_CONTINUOUS,  // Continuous output via DMA
    DAC_MODE_COSINE,      // Cosine wave generation
} dac_mode_t;

typedef struct _mdac_obj_t {
    mp_obj_base_t base;
    gpio_num_t gpio_id;
    dac_channel_t dac_id;
    dac_mode_t mode;
    // One-shot mode handle
    dac_oneshot_handle_t dac_oneshot_handle;
    // Continuous mode handles
    dac_continuous_handle_t dac_continuous_handle;
    // Cosine mode handle
    dac_cosine_handle_t dac_cosine_handle;
    // Resource cleanup tracking
    bool initialized;
    // Track dynamically allocated buffer for continuous mode
    uint8_t *continuous_buffer;
    size_t continuous_buffer_len;
    bool continuous_buffer_needs_free;
} mdac_obj_t;

static mdac_obj_t mdac_obj[] = {
    #if CONFIG_IDF_TARGET_ESP32
    {{&machine_dac_type}, GPIO_NUM_25, DAC_CHAN_0, DAC_MODE_ONESHOT, NULL, NULL, NULL, false, NULL, 0, false},
    {{&machine_dac_type}, GPIO_NUM_26, DAC_CHAN_1, DAC_MODE_ONESHOT, NULL, NULL, NULL, false, NULL, 0, false},
    #else
    {{&machine_dac_type}, GPIO_NUM_17, DAC_CHAN_0, DAC_MODE_ONESHOT, NULL, NULL, NULL, false, NULL, 0, false},
    {{&machine_dac_type}, GPIO_NUM_18, DAC_CHAN_1, DAC_MODE_ONESHOT, NULL, NULL, NULL, false, NULL, 0, false},
    #endif
};

// Internal function to cleanup existing resources for the current mode
static void mdac_cleanup_resources(mdac_obj_t *self) {
    if (!self->initialized) {
        return;
    }

    switch (self->mode) {
        case DAC_MODE_ONESHOT:
            if (self->dac_oneshot_handle != NULL) {
                dac_oneshot_del_channel(self->dac_oneshot_handle);
                self->dac_oneshot_handle = NULL;
            }
            break;
        case DAC_MODE_CONTINUOUS:
            if (self->dac_continuous_handle != NULL) {
                dac_continuous_disable(self->dac_continuous_handle);
                dac_continuous_del_channels(self->dac_continuous_handle);
                self->dac_continuous_handle = NULL;
            }
            // Free any allocated buffer
            if (self->continuous_buffer_needs_free && self->continuous_buffer != NULL) {
                m_del(uint8_t, self->continuous_buffer, self->continuous_buffer_len);
                self->continuous_buffer = NULL;
                self->continuous_buffer_needs_free = false;
                self->continuous_buffer_len = 0;
            }
            break;
        case DAC_MODE_COSINE:
            if (self->dac_cosine_handle != NULL) {
                dac_cosine_stop(self->dac_cosine_handle);
                dac_cosine_del_channel(self->dac_cosine_handle);
                self->dac_cosine_handle = NULL;
            }
            break;
    }

    self->mode = DAC_MODE_ONESHOT;
    self->initialized = false;
}

static mp_obj_t mdac_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw,
    const mp_obj_t *args) {

    mp_arg_check_num(n_args, n_kw, 1, 1, true);
    gpio_num_t pin_id = machine_pin_get_id(args[0]);
    mdac_obj_t *self = NULL;
    for (int i = 0; i < MP_ARRAY_SIZE(mdac_obj); i++) {
        if (pin_id == mdac_obj[i].gpio_id) {
            self = &mdac_obj[i];
            break;
        }
    }
    if (!self) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid Pin for DAC"));
    }

    // Cleanup any existing resources
    mdac_cleanup_resources(self);

    // Initialize in oneshot mode (default)
    dac_oneshot_config_t dac_oneshot_config = {.chan_id = self->dac_id};
    esp_err_t err = dac_oneshot_new_channel(&dac_oneshot_config, &self->dac_oneshot_handle);
    if (err != ESP_OK) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("failed to create oneshot DAC"));
    }
    err = dac_oneshot_output_voltage(self->dac_oneshot_handle, 0);
    if (err != ESP_OK) {
        dac_oneshot_del_channel(self->dac_oneshot_handle);
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("failed to set initial voltage"));
    }

    self->mode = DAC_MODE_ONESHOT;
    self->initialized = true;

    return MP_OBJ_FROM_PTR(self);
}

static void mdac_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    mdac_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "DAC(Pin(%u), mode=%d)", self->gpio_id, self->mode);
}

static mp_obj_t mdac_write(mp_obj_t self_in, mp_obj_t value_in) {
    mdac_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int value = mp_obj_get_int(value_in);
    if (value < 0 || value > 255) {
        mp_raise_ValueError(MP_ERROR_TEXT("value out of range (0-255)"));
    }

    // Ensure we're in oneshot mode. If not, user must call stop() first.
    if (self->mode != DAC_MODE_ONESHOT) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DAC is active. Call stop() before using write()."));
    }

    esp_err_t err = dac_oneshot_output_voltage(self->dac_oneshot_handle, value);
    if (err != ESP_OK) {
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("DAC output failed (error: %d)"), err);
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(mdac_write_obj, mdac_write);

// Method to output continuous data using DMA
static mp_obj_t mdac_write_continuous(size_t n_args, const mp_obj_t *args) {
    mdac_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    uint8_t *data = NULL;
    bool need_free = false;
    size_t len;

    // Check if first argument is a sequence (list/tuple) or buffer
    if (mp_obj_is_type(args[1], &mp_type_list) || mp_obj_is_type(args[1], &mp_type_tuple)) {
        // Handle list/tuple input
        mp_obj_t *items;
        mp_obj_get_array(args[1], &len, &items);
        data = m_new(uint8_t, len);
        for (size_t i = 0; i < len; i++) {
            int val = mp_obj_get_int(items[i]);
            if (val < 0 || val > 255) {
                m_del(uint8_t, data, len);
                mp_raise_ValueError(MP_ERROR_TEXT("data values must be in range 0-255"));
            }
            data[i] = (uint8_t)val;
        }
        need_free = true;
    } else {
        // Handle buffer input (bytes, bytearray)
        mp_buffer_info_t bufinfo;
        mp_get_buffer_raise(args[1], &bufinfo, MP_BUFFER_READ);
        len = bufinfo.len;
        data = (uint8_t *)bufinfo.buf;
        need_free = false;  // Don't free if it's from a buffer object
    }

    if (len == 0) {
        if (need_free) {
            m_del(uint8_t, data, len);
        }
        return mp_const_none;
    }

    // DMA buffer size must be a multiple of 4.
    size_t padded_len = (len + 3) & ~3;
    uint8_t *padded_data = m_new0(uint8_t, padded_len);
    memcpy(padded_data, data, len);

    if (need_free) {
        m_del(uint8_t, data, len);
    }

    // The new padded_data buffer is what we'll use, and it always needs freeing.
    data = padded_data;
    len = padded_len;
    need_free = true;

    // Get frequency (in Hz)
    uint32_t freq = mp_obj_get_int(args[2]);

    // Default to "once" mode if not specified
    bool loop_mode = false;
    if (n_args > 3) {
        mp_obj_t mode_obj = args[3];
        if (mp_obj_is_str(mode_obj)) {
            const char *mode_str = mp_obj_str_get_str(mode_obj);
            if (strcmp(mode_str, "loop") == 0) {
                loop_mode = true;
            } else if (strcmp(mode_str, "once") == 0) {
                loop_mode = false;
            } else if (strcmp(mode_str, "async") == 0) {
                // For now, treat async similar to loop
                loop_mode = true;
            }
        } else {
            // If numeric, interpret as boolean
            loop_mode = mp_obj_is_true(mode_obj);
        }
    }

    // Store the buffer information for potential cleanup
    uint8_t *original_data = data;
    size_t data_len = len;

    // Cleanup any existing resources
    mdac_cleanup_resources(self);

    // Configure for continuous output
    dac_continuous_config_t cont_config = {
        .chan_mask = 1ULL << self->dac_id,
        .desc_num = 4,  // Use multiple descriptors as per ESP-IDF recommendations
        .buf_size = data_len > 2048 ? ((data_len + 3) & ~3) : 2048,  // Use larger buffer as per ESP-IDF examples, ensure multiple of 4
        .freq_hz = freq,
        .offset = 0,
        #if CONFIG_IDF_TARGET_ESP32S2
        .clk_src = DAC_DIGI_CLK_SRC_APB,  // ESP32-S2 uses APB clock by default
        #else
        // Default to APB clock, but select APLL for low frequencies (< ~320kHz) on ESP32
        .clk_src = (freq < 320000) ? DAC_DIGI_CLK_SRC_APLL : DAC_DIGI_CLK_SRC_DEFAULT,
        #endif
    };

    esp_err_t err = dac_continuous_new_channels(&cont_config, &self->dac_continuous_handle);
    if (err != ESP_OK) {
        if (need_free) {
            m_del(uint8_t, original_data, data_len);
        }
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("failed to create continuous DAC (error: %d)"), err);
    }

    self->mode = DAC_MODE_CONTINUOUS;
    self->initialized = true;

    // Enable the channel
    err = dac_continuous_enable(self->dac_continuous_handle);
    if (err != ESP_OK) {
        dac_continuous_del_channels(self->dac_continuous_handle);
        if (need_free) {
            m_del(uint8_t, original_data, data_len);
        }
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("failed to enable continuous DAC (error: %d)"), err);
    }

    // Write the data - different implementation for different modes
    if (loop_mode) {
        // For loop mode, use cyclic write
        // When using cyclic write, the buffer must remain available, so we need to keep it
        size_t bytes_loaded = 0;
        err = dac_continuous_write_cyclically(self->dac_continuous_handle, original_data, data_len, &bytes_loaded);
        // For cyclic mode, we need to keep the buffer alive as long as the DAC is running
        if (need_free) {
            self->continuous_buffer = original_data;
            self->continuous_buffer_len = data_len;
            self->continuous_buffer_needs_free = true;
        }
    } else {
        // For one-time mode, use normal write
        size_t actual_write = 0;
        err = dac_continuous_write(self->dac_continuous_handle, original_data, data_len, &actual_write, 0);
        // For one-time mode, we can free the buffer after writing
        if (need_free) {
            // If it was a one-time write, we should free the memory after the write completes
            // But since we don't have a callback mechanism here, we'll free it after setting the fields
            // The actual freeing should happen when the mode changes or the object is destroyed
            self->continuous_buffer = original_data;
            self->continuous_buffer_len = data_len;
            self->continuous_buffer_needs_free = true;
        }
    }

    if (err != ESP_OK) {
        dac_continuous_disable(self->dac_continuous_handle);
        dac_continuous_del_channels(self->dac_continuous_handle);
        if (need_free && self->continuous_buffer == NULL) {
            // If we didn't set the tracking fields, free it now
            m_del(uint8_t, original_data, data_len);
        }
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("failed to write continuous data (error: %d)"), err);
    }

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mdac_write_continuous_obj, 3, 4, mdac_write_continuous);

// Method to output cosine wave
static mp_obj_t mdac_cosine(size_t n_args, const mp_obj_t *args) {
    mdac_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    // Get frequency (in Hz)
    uint32_t freq = mp_obj_get_int(args[1]);

    // Handle attenuation, phase, and offset parameters
    dac_cosine_atten_t atten = DAC_COSINE_ATTEN_DEFAULT;  // Default to full scale
    dac_cosine_phase_t phase = DAC_COSINE_PHASE_0;    // Default to 0 degree phase
    int8_t offset = 0;    // Default to no DC offset

    if (n_args > 2) {
        atten = (dac_cosine_atten_t)mp_obj_get_int(args[2]);
    }
    if (n_args > 3) {
        phase = (dac_cosine_phase_t)mp_obj_get_int(args[3]);
    }
    if (n_args > 4) {
        offset = (int8_t)mp_obj_get_int(args[4]);
    }

    // Cleanup any existing resources
    mdac_cleanup_resources(self);

    dac_cosine_config_t cosine_config = {
        .chan_id = self->dac_id,
        .freq_hz = freq,
        .clk_src = DAC_COSINE_CLK_SRC_DEFAULT,
        .atten = atten,
        .phase = phase,
        .offset = offset,
        .flags = {
            .force_set_freq = false,
        },
    };

    esp_err_t err = dac_cosine_new_channel(&cosine_config, &self->dac_cosine_handle);
    if (err != ESP_OK) {
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("failed to create cosine DAC (error: %d)"), err);
    }

    self->mode = DAC_MODE_COSINE;
    self->initialized = true;

    // Start the cosine wave
    err = dac_cosine_start(self->dac_cosine_handle);
    if (err != ESP_OK) {
        dac_cosine_del_channel(self->dac_cosine_handle);
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("failed to start cosine DAC (error: %d)"), err);
    }

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mdac_cosine_obj, 2, 5, mdac_cosine);

// Method to stop any active output and re-initialize the DAC
static mp_obj_t mdac_stop(mp_obj_t self_in) {
    mdac_obj_t *self = MP_OBJ_TO_PTR(self_in);

    // Cleanup any existing resources from other modes
    mdac_cleanup_resources(self);

    // Re-initialize into default oneshot mode at 0V
    dac_oneshot_config_t dac_oneshot_config = {.chan_id = self->dac_id};
    esp_err_t err = dac_oneshot_new_channel(&dac_oneshot_config, &self->dac_oneshot_handle);
    if (err != ESP_OK) {
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("failed to re-initialize DAC (error: %d)"), err);
    }
    err = dac_oneshot_output_voltage(self->dac_oneshot_handle, 0);
    if (err != ESP_OK) {
        dac_oneshot_del_channel(self->dac_oneshot_handle);
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("failed to set DAC to 0V (error: %d)"), err);
    }
    self->mode = DAC_MODE_ONESHOT;
    self->initialized = true;

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(mdac_stop_obj, mdac_stop);

static const mp_rom_map_elem_t mdac_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_write), MP_ROM_PTR(&mdac_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_write_continuous), MP_ROM_PTR(&mdac_write_continuous_obj) },
    { MP_ROM_QSTR(MP_QSTR_cosine), MP_ROM_PTR(&mdac_cosine_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop), MP_ROM_PTR(&mdac_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&mdac_stop_obj) }, // deinit is an alias for stop
    // Cosine wave attenuation constants
    { MP_ROM_QSTR(MP_QSTR_ATTN_0DB), MP_ROM_INT(DAC_COSINE_ATTEN_DEFAULT) },  // Default (0dB) attenuation
    { MP_ROM_QSTR(MP_QSTR_ATTN_6DB), MP_ROM_INT(DAC_COSINE_ATTEN_DB_6) },     // -6dB attenuation
    { MP_ROM_QSTR(MP_QSTR_ATTN_12DB), MP_ROM_INT(DAC_COSINE_ATTEN_DB_12) },   // -12dB attenuation
    { MP_ROM_QSTR(MP_QSTR_ATTN_18DB), MP_ROM_INT(DAC_COSINE_ATTEN_DB_18) },   // -18dB attenuation
    // Cosine wave phase constants
    { MP_ROM_QSTR(MP_QSTR_PHASE_0), MP_ROM_INT(DAC_COSINE_PHASE_0) },        // 0 degree phase
    { MP_ROM_QSTR(MP_QSTR_PHASE_180), MP_ROM_INT(DAC_COSINE_PHASE_180) },    // 180 degree phase
};

static MP_DEFINE_CONST_DICT(mdac_locals_dict, mdac_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    machine_dac_type,
    MP_QSTR_DAC,
    MP_TYPE_FLAG_NONE,
    make_new, mdac_make_new,
    print, mdac_print,
    locals_dict, &mdac_locals_dict
    );

#endif // MICROPY_PY_MACHINE_DAC
