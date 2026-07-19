/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

   this file only sends the ID to the servo queue 
*/
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// iface means interface 
#include "esp_wn_iface.h" // wn means wakenet - thewake word detector
// models mean the trained neural-network data
#include "esp_wn_models.h"
#include "esp_afe_sr_iface.h" // afe means audio front end, sr means speech recognition -> cleans the raw microphone sound
#include "esp_afe_sr_models.h"
#include "esp_mn_iface.h" // mn means multinet - the command recognition model (6 commands)
#include "esp_mn_models.h"
#include "esp_board_init.h" // set up board's microphone
#include "speech_commands_action.h"
#include "model_path.h"
#include "esp_process_sdkconfig.h"

#include "servo.h"

#define CMD_GOOD_BOY 0
#define CMD_SIT_DOWN 1
#define CMD_LIE_DOWN 2
#define CMD_STRETCH 3
#define CMD_WALK 4
#define CMD_DANCE 5

int wakeup_flag = 0;
static const esp_afe_sr_iface_t *afe_handle = NULL;
static volatile int task_flag = 0;
srmodel_list_t *models = NULL;

QueueHandle_t servo_cmd_queue;

// grab sound from the mic, hand it to the audio cleaner, repeat forever
void feed_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    // ->: reach through this pointer to get a function inside
    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
    int nch = afe_handle->get_feed_channel_num(afe_data);
    int feed_channel = esp_get_feed_channel();
    // sanity check
    assert(nch == feed_channel);
    // malloc(): memory allocate, asks the system for a block of memory of the given size and returns its address
    int16_t *i2s_buff = malloc(audio_chunksize * sizeof(int16_t) * feed_channel);
    assert(i2s_buff);

    while (task_flag)
    {
        // esp_get_feed_data: reads raw audio samples from the microphone and fills the buffer. The parameters are:
        // - true: indicates that the function should block until enough data is available.
        // - i2s_buff: pointer to the buffer where the audio samples will be stored.
        // - audio_chunksize * sizeof(int16_t) * feed_channel: the size of
        // the buffer in bytes, calculated as the number of samples (audio_chunksize) multiplied by the size of each sample (sizeof(int16_t)) and the number of channels (feed_channel).
        esp_get_feed_data(true, i2s_buff, audio_chunksize * sizeof(int16_t) * feed_channel); 
        
        // afe does noise suppression, echo cancellation, and other audio preprocessing. The feed function takes the raw audio data and processes it according to the AFE configuration.
        afe_handle->feed(afe_data, i2s_buff);
    }
    if (i2s_buff)
    {
        free(i2s_buff); // give memory back to system
        i2s_buff = NULL;
    }
    // delete the current task after all work done
    vTaskDelete(NULL);
}

/*
Chunk 1:
One-time setup of MultiNet (the command recognizer): find the English model, get a handle, 
create its working data (with a 6-second listen window), and — critically — 
load the command words from the settings file via esp_mn_commands_update_from_sdkconfig.

Chunk 2:
The start of the listening loop — fetching cleaned audio and detecting the wake word.

Chunk 3:
What happens once awake — feeding audio into MultiNet, recognizing the command, and dropping its number in the mailbox.

Chunk 4:
The timeout case — when no command comes and the dog goes back to sleep.
*/
void detect_Task(void *arg)
{
    // Chunk 1 starts
    esp_afe_sr_data_t *afe_data = arg;
    // ask how big a batch of CLEANED audio it hands out at a time
    int afe_chunksize = afe_handle->get_fetch_chunksize(afe_data);
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_ENGLISH);
    printf("multinet:%s\n", mn_name);
    esp_mn_iface_t *multinet = esp_mn_handle_from_name(mn_name);
    model_iface_data_t *model_data = multinet->create(mn_name, 6000);
    int mu_chunksize = multinet->get_samp_chunksize(model_data);
    esp_mn_commands_update_from_sdkconfig(multinet, model_data); // Add speech commands from sdkconfig
    multinet->set_det_threshold(model_data, 0.4); // lower = easier to trigger commands (safe: our commands sound distinct)
    assert(mu_chunksize == afe_chunksize);
    // print active speech commands
    multinet->print_active_speech_commands(model_data);

    printf("------------detect start------------\n");
    // Chunk 1 ends
    // Chunk 2 starts
    while (task_flag)
    {
        // pulls processed audio out of AFE 
        afe_fetch_result_t *res = afe_handle->fetch(afe_data);
        if (!res || res->ret_value == ESP_FAIL)
        {
            printf("fetch error!\n");
            break;
        }

        if (res->wakeup_state == WAKENET_DETECTED)
        {
            printf("WAKEWORD DETECTED\n");
            // reset MultiNet's memory, wiping any leftover partial audio so it starts istening for the command fresh
            multinet->clean(model_data);
        }

        // checks if the wake word (WakeNet) was detected, sets wakeup_flag=1
        if (res->raw_data_channels == 1 && res->wakeup_state == WAKENET_DETECTED)
        {
            wakeup_flag = 1;
        }
        else if (res->raw_data_channels > 1 && res->wakeup_state == WAKENET_CHANNEL_VERIFIED)
        {
            // For a multi-channel AFE, it is necessary to wait for the channel to be verified.
            printf("AFE_FETCH_CHANNEL_VERIFIED, channel index: %d\n", res->trigger_channel_id);
            wakeup_flag = 1;
        }
        // Chunk 2 ends
        // Chunk 3 starts
        // once awake, feeds audio into MultiNet (the command-recognition model) via multinet->detect(...)
        if (wakeup_flag == 1)
        {
            esp_mn_state_t mn_state = multinet->detect(model_data, res->data);

            if (mn_state == ESP_MN_STATE_DETECTING)
            {
                continue;
            }

            if (mn_state == ESP_MN_STATE_DETECTED)
            {
                esp_mn_results_t *mn_result = multinet->get_results(model_data);
                for (int i = 0; i < mn_result->num; i++)
                {
                    printf("TOP %d, command_id: %d, phrase_id: %d, string: %s, prob: %f\n",
                           i + 1, mn_result->command_id[i], mn_result->phrase_id[i], mn_result->string, mn_result->prob[i]);
                }

                // Send top command_id to servo task — non-blocking
                int cmd_id = mn_result->command_id[0];
                // pushes the command ID onto a queue without blocking, so the recognition loop never stalls waiting on the servo task
                // 0: timeout: how long to wait if the mailbox is full -> 0 means don't wait at all
                xQueueSend(servo_cmd_queue, &cmd_id, 0);
                printf("-----------listening-----------\n");
            }
            // Chunk 3 ends
            // Chunk 4 starts
            // on timeout (no command matched in time), it re-arms wake word detection and goes back to sleep, waiting to be woken up again
            if (mn_state == ESP_MN_STATE_TIMEOUT)
            {
                esp_mn_results_t *mn_result = multinet->get_results(model_data);
                printf("timeout, string:%s\n", mn_result->string);
                afe_handle->enable_wakenet(afe_data);
                wakeup_flag = 0;
                printf("\n-----------awaits to be waken up-----------\n");
                continue;
            }
        }
    }
    if (model_data)
    {
        multinet->destroy(model_data);
        model_data = NULL;
    }
    printf("detect exit\n");
    vTaskDelete(NULL);
    // Chunk 4 ends
}

// Reads the mailbox and plays the tricks
void servo_Task(void *arg)
{
    int cmd_id;
    while (1)
    {
        // Block here until a command arrives — no CPU waste
        if (xQueueReceive(servo_cmd_queue, &cmd_id, portMAX_DELAY))
        {
            printf("servo_Task received command: %d\n", cmd_id);
            switch (cmd_id)
            {
            case CMD_GOOD_BOY:
                anim_good_boy();
                break;
            case CMD_SIT_DOWN:
                anim_sit_down();
                break;
            case CMD_LIE_DOWN:
                anim_lie_down();
                break;
            case CMD_STRETCH:
                anim_stretch();
                break;
            case CMD_WALK:
                anim_walk();
                break;
            case CMD_DANCE:
                anim_dance();
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

// Creates everything and launches all three tasks.
void app_main()
{
    // configure the PWM timer and 4 channels
    servo_init();        // ← add this early, before tasks start
    servo_all_neutral(); // park all legs at 90°

    servo_cmd_queue = xQueueCreate(5, sizeof(int));

    models = esp_srmodel_init("model"); // partition label defined in partitions.csv
    ESP_ERROR_CHECK(esp_board_init(16000, 2, 16));
    // ESP_ERROR_CHECK(esp_sdcard_init("/sdcard", 10));

#if CONFIG_IDF_TARGET_ESP32
    printf("This demo only support ESP32S3\n");
    return;
#else
    afe_config_t *afe_config = afe_config_init(esp_get_input_format(), models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
    afe_handle = esp_afe_handle_from_config(afe_config);
    esp_afe_sr_data_t *afe_data = afe_handle->create_from_config(afe_config);
    afe_config_free(afe_config);
    afe_handle->set_wakenet_threshold(afe_data, 1, 0.5); // lower = easier to detect "Hi Jolly" (range 0.4–0.9999)
#endif

    task_flag = 1;
    xTaskCreatePinnedToCore(&detect_Task, "detect", 8 * 1024, (void *)afe_data, 5, NULL, 1);
    xTaskCreatePinnedToCore(&feed_Task, "feed", 8 * 1024, (void *)afe_data, 5, NULL, 0);
    xTaskCreatePinnedToCore(servo_Task, "servo", 4 * 1024, NULL, 5, NULL, 0);
}
