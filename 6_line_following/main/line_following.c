#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sra_board.h"
#include "tuning_http_server.h"

#define MODE NORMAL_MODE
#define BLACK_MARGIN 4095
#define WHITE_MARGIN 0
#define bound_LSA_LOW 0
#define bound_LSA_HIGH 1000
#define BLACK_BOUNDARY 820

const int weights[5] = {-5, -3, 1, 3, 5};

int optimum_duty_cycle = 57;
int lower_duty_cycle = 45;
int higher_duty_cycle = 65;
float left_duty_cycle = 0, right_duty_cycle = 0;

// PID variables
float error = 0, prev_error = 0, difference, cumulative_error, correction;

// Sensor readings
line_sensor_array line_sensor_readings;

// Motor handles
motor_handle_t motor_a_0, motor_a_1;

// Junction handling variables
bool junction_detected = false;
int junction_cooldown = 0;

// Path memory for optimization (optional but recommended)
#define MAX_PATH_LENGTH 100
char path[MAX_PATH_LENGTH];
int path_index = 0;

// --- Helper Function Prototypes ---
void calculate_error();
void calculate_correction();
void moveForward();
void moveForwardDistance(int ms);
void takeLeft();
void takeRight();
void takeUTurn();
void stopBot();
bool isLeftAvailable();
bool isRightAvailable();
bool isStraightAvailable();
bool isAllWhite();
bool isJunction();
void handleJunction();
void simplifyPath();

// --- Core Logic ---

void calculate_correction()
{
    error = error * 10;
    difference = error - prev_error;
    cumulative_error += error;
    cumulative_error = bound(cumulative_error, -30, 30);
    correction = read_pid_const().kp * error + read_pid_const().ki * cumulative_error + read_pid_const().kd * difference;
    prev_error = error;
}

void calculate_error()
{
    int all_black_flag = 1;
    float weighted_sum = 0, sum = 0;
    float pos = 0;
    int k = 0;

    for (int i = 0; i < 5; i++)
    {
        if (line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY)
            all_black_flag = 0;

        if (line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY)
            k = 1;
        else
            k = 0;

        weighted_sum += (float)(weights[i]) * k;
        sum = sum + k;
    }

    if (sum != 0)
        pos = (weighted_sum - 1) / sum;

    if (all_black_flag == 1)
    {
        if (prev_error > 0)
            error = 2.5;
        else
            error = -2.5;
    }
    else
        error = pos;
}

// --- Movement Functions ---

void moveForward()
{
    set_motor_speed(motor_a_0, MOTOR_FORWARD, optimum_duty_cycle);
    set_motor_speed(motor_a_1, MOTOR_FORWARD, optimum_duty_cycle);
}

void moveForwardDistance(int ms)
{
    set_motor_speed(motor_a_0, MOTOR_FORWARD, optimum_duty_cycle);
    set_motor_speed(motor_a_1, MOTOR_FORWARD, optimum_duty_cycle);
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

void takeLeft()
{
    // Move forward slightly to center on junction
    moveForwardDistance(150);
    
    // Turn left
    set_motor_speed(motor_a_0, MOTOR_BACKWARD, 55);
    set_motor_speed(motor_a_1, MOTOR_FORWARD, 55);
    vTaskDelay(350 / portTICK_PERIOD_MS);
    
    // Wait until bot finds the line again
    while (line_sensor_readings.adc_reading[2] < BLACK_BOUNDARY)
    {
        line_sensor_readings = read_line_sensor(line_sensor);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    // Store path
    if (path_index < MAX_PATH_LENGTH - 1)
    {
        path[path_index++] = 'L';
        path[path_index] = '\0';
    }
}

void takeRight()
{
    // Move forward slightly to center on junction
    moveForwardDistance(150);
    
    // Turn right
    set_motor_speed(motor_a_0, MOTOR_FORWARD, 55);
    set_motor_speed(motor_a_1, MOTOR_BACKWARD, 55);
    vTaskDelay(350 / portTICK_PERIOD_MS);
    
    // Wait until bot finds the line again
    while (line_sensor_readings.adc_reading[2] < BLACK_BOUNDARY)
    {
        line_sensor_readings = read_line_sensor(line_sensor);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    // Store path
    if (path_index < MAX_PATH_LENGTH - 1)
    {
        path[path_index++] = 'R';
        path[path_index] = '\0';
    }
}

void takeUTurn()
{
    // Move forward slightly
    moveForwardDistance(100);
    
    // U-turn (turn right twice)
    set_motor_speed(motor_a_0, MOTOR_FORWARD, 55);
    set_motor_speed(motor_a_1, MOTOR_BACKWARD, 55);
    vTaskDelay(650 / portTICK_PERIOD_MS);
    
    // Wait until bot finds the line again
    while (line_sensor_readings.adc_reading[2] < BLACK_BOUNDARY)
    {
        line_sensor_readings = read_line_sensor(line_sensor);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    // Store path
    if (path_index < MAX_PATH_LENGTH - 1)
    {
        path[path_index++] = 'U';
        path[path_index] = '\0';
    }
    
    // Simplify path after dead end
    simplifyPath();
}

void stopBot()
{
    set_motor_speed(motor_a_0, MOTOR_STOP, 0);
    set_motor_speed(motor_a_1, MOTOR_STOP, 0);
}

// --- Junction Detection ---

bool isLeftAvailable()
{
    return (line_sensor_readings.adc_reading[0] > BLACK_BOUNDARY);
}

bool isRightAvailable()
{
    return (line_sensor_readings.adc_reading[4] > BLACK_BOUNDARY);
}

bool isStraightAvailable()
{
    return (line_sensor_readings.adc_reading[2] > BLACK_BOUNDARY);
}

bool isAllWhite()
{
    for (int i = 0; i < 5; i++)
    {
        if (line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY)
            return false;
    }
    return true;
}

bool isJunction()
{
    // A junction is detected when side sensors see black
    // AND center sensor also sees black (confirming it's a junction, not just noise)
    return ((isLeftAvailable() || isRightAvailable()) && isStraightAvailable());
}

// --- Maze Decision Logic (Left-Hand Rule) ---
void handleJunction()
{
    // Read sensors again to confirm junction
    line_sensor_readings = read_line_sensor(line_sensor);
    
    bool left = isLeftAvailable();
    bool straight = isStraightAvailable();
    bool right = isRightAvailable();
    bool dead_end = isAllWhite();
    
    // Dead end - turn around
    if (dead_end)
    {
        takeUTurn();
    }
    // Left-hand rule priority: Left > Straight > Right > U-turn
    else if (left)
    {
        takeLeft();
    }
    else if (straight)
    {
        // Go straight at junction (still need to store it)
        moveForwardDistance(150);
        if (path_index < MAX_PATH_LENGTH - 1)
        {
            path[path_index++] = 'S';
            path[path_index] = '\0';
        }
    }
    else if (right)
    {
        takeRight();
    }
    else
    {
        // No path available - shouldn't happen but handle it
        takeUTurn();
    }
    
    // Set cooldown to avoid detecting same junction multiple times
    junction_cooldown = 30; // ~300ms cooldown
}

// --- Path Simplification (Optimization) ---
void simplifyPath()
{
    if (path_index < 3) return;
    
    // Look for patterns and simplify
    // LUL = S, LUS = R, RUL = S, SUR = L, etc.
    for (int i = 0; i < path_index - 2; i++)
    {
        if (path[i + 1] == 'U')
        {
            char before = path[i];
            char after = path[i + 2];
            char replacement = '\0';
            
            // Apply simplification rules
            if (before == 'L' && after == 'R') replacement = 'U';
            else if (before == 'L' && after == 'S') replacement = 'R';
            else if (before == 'L' && after == 'L') replacement = 'S';
            else if (before == 'R' && after == 'L') replacement = 'U';
            else if (before == 'S' && after == 'L') replacement = 'R';
            else if (before == 'S' && after == 'S') replacement = 'U';
            else if (before == 'S' && after == 'R') replacement = 'L';
            else if (before == 'R' && after == 'S') replacement = 'L';
            else if (before == 'R' && after == 'R') replacement = 'S';
            
            if (replacement != '\0')
            {
                // Replace three moves with one
                path[i] = replacement;
                
                // Shift rest of path
                for (int j = i + 1; j < path_index - 2; j++)
                {
                    path[j] = path[j + 2];
                }
                path_index -= 2;
                path[path_index] = '\0';
                
                // Check again from same position
                i--;
            }
        }
    }
}

// --- Main Line Follow Task ---
void line_follow_task(void *arg)
{
    adc_handle_t line_sensor;
    ESP_ERROR_CHECK(enable_motor_driver(&motor_a_0, MOTOR_A_0));
    ESP_ERROR_CHECK(enable_motor_driver(&motor_a_1, MOTOR_A_1));
    ESP_ERROR_CHECK(enable_line_sensor(&line_sensor));
    ESP_ERROR_CHECK(enable_bar_graph());

    #ifdef CONFIG_ENABLE_OLED
    ESP_ERROR_CHECK(init_oled());
    vTaskDelay(100);
    lv_obj_clean(lv_scr_act());
    #endif

    // Initialize path
    path[0] = '\0';

    while (true)
    {
        // Read and normalize sensor values
        line_sensor_readings = read_line_sensor(line_sensor);

        for (int i = 0; i < 5; i++)
        {
            line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN);
            line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
            line_sensor_readings.adc_reading[i] = 1000 - (line_sensor_readings.adc_reading[i]);
        }

        // Decrease cooldown
        if (junction_cooldown > 0)
        {
            junction_cooldown--;
        }

        // Check for junction only if cooldown expired
        if (junction_cooldown == 0 && (isJunction() || isAllWhite()))
        {
            handleJunction();
        }
        else
        {
            // Normal line following with PID
            calculate_error();
            calculate_correction();

            left_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);
            right_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);

            set_motor_speed(motor_a_0, MOTOR_FORWARD, left_duty_cycle);
            set_motor_speed(motor_a_1, MOTOR_FORWARD, right_duty_cycle);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void app_main()
{
    xTaskCreate(&line_follow_task, "line_follow_task", 4096, NULL, 1, NULL);
    start_tuning_http_server();
}