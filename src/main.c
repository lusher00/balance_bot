/**
 * @file main.c
 * @brief Main entry point for balance_bot
 * 
 * Parses command-line arguments and initializes all subsystems.
 */

#include "cat_follower.h"
#include "display.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>

// Input mode selection
typedef enum {
    INPUT_NONE,
    INPUT_CAT,
    INPUT_XBOX,
    INPUT_MANUAL,
    INPUT_SBUS
} input_mode_t;

static input_mode_t input_mode = INPUT_CAT;
static char* pid_config_file = NULL;
static char* xbox_device = NULL;
static char* sbus_device = NULL;
static bool quiet_mode = false;

/* Display block intervals requested via -d flags (0 = off) */
static int disp_sbus    = 0;
static int disp_pid     = 0;
static int disp_enc     = 0;
static int disp_imu     = 0;
static int disp_motors  = 0;
static int disp_system  = 100;   /* on by default */

/**
 * @brief Print usage information
 */
static void print_usage(void) {
    printf("\n");
    printf("balance_bot - Self-balancing robot with iPhone app\n");
    printf("\n");
    printf("Usage: balance_bot [options]\n");
    printf("\n");
    printf("Options:\n");
    printf("  -i <mode>     Input mode:\n");
    printf("                  cat   - Cat following (default)\n");
    printf("                  xbox  - Xbox controller (specify device as last arg)\n");
    printf("                  sbus  - FrSky R-XSR via UART (T16 transmitter)\n");
    printf("                  none  - Balance only, no input\n");
    printf("  -p <file>     PID config file (default: pidconfig.txt)\n");
    printf("  -u <device>   UART device for SBUS (default: /dev/ttyO4)\n");
    printf("  -d <block>    Enable console display block (repeatable):\n");
    printf("                  sbus  - SBUS TX channels and switch states\n");
    printf("                  pid   - PID setpoint/error/P/I/D/output\n");
    printf("                  enc   - Encoder ticks, position, velocity\n");
    printf("                  imu   - Pitch/roll/yaw and rates\n");
    printf("                  mot   - Motor duty cycles\n");
    printf("                  sys   - System status (on by default)\n");
    printf("                  all   - Enable all blocks\n");
    printf("                  none  - Disable all blocks (including sys)\n");
    printf("  -h            Show this help\n");
    printf("\n");
    printf("Examples:\n");
    printf("  balance_bot                          # Cat following mode\n");
    printf("  balance_bot -i none                  # Balance only\n");
    printf("  balance_bot -i xbox /dev/input/js0   # Xbox controller\n");
    printf("  balance_bot -i sbus                  # T16 via R-XSR SBUS\n");
    printf("  balance_bot -i sbus -u /dev/ttyO4    # SBUS on alternate UART\n");
    printf("  balance_bot -p myconfig.txt          # Custom PID config\n");
    printf("\n");
    printf("Controls:\n");
    printf("  MODE button  - Arm/disarm motors\n");
    printf("  PAUSE button - Toggle cat following\n");
    printf("  Ctrl+C       - Exit\n");
    printf("\n");
}

/**
 * @brief Parse command-line arguments
 */
static int parse_args(int argc, char* argv[]) {
    int opt;
    
    while ((opt = getopt(argc, argv, "i:p:u:d:qh")) != -1) {
        switch (opt) {
            case 'i':
                if (strcmp(optarg, "cat") == 0) {
                    input_mode = INPUT_CAT;
                } else if (strcmp(optarg, "xbox") == 0) {
                    input_mode = INPUT_XBOX;
                } else if (strcmp(optarg, "sbus") == 0) {
                    input_mode = INPUT_SBUS;
                } else if (strcmp(optarg, "none") == 0) {
                    input_mode = INPUT_NONE;
                } else {
                    fprintf(stderr, "Error: Invalid input mode '%s'\n", optarg);
                    print_usage();
                    return -1;
                }
                break;

            case 'p':
                pid_config_file = optarg;
                break;

            case 'u':
                sbus_device = optarg;
                break;

            case 'd':
                if      (strcmp(optarg, "sbus") == 0) disp_sbus   = 10;
                else if (strcmp(optarg, "pid")  == 0) disp_pid    = 10;
                else if (strcmp(optarg, "enc")  == 0) disp_enc    = 10;
                else if (strcmp(optarg, "imu")  == 0) disp_imu    = 10;
                else if (strcmp(optarg, "mot")  == 0) disp_motors = 10;
                else if (strcmp(optarg, "sys")  == 0) disp_system = 100;
                else if (strcmp(optarg, "all")  == 0) {
                    disp_sbus = disp_pid = disp_enc =
                    disp_imu = disp_motors = disp_system = 10;
                }
                else if (strcmp(optarg, "none") == 0) {
                    disp_sbus = disp_pid = disp_enc =
                    disp_imu = disp_motors = disp_system = 0;
                }
                else {
                    fprintf(stderr, "Error: Unknown display block '%s'\n", optarg);
                    print_usage();
                    return -1;
                }
                break;

            case 'q':
                quiet_mode = true;
                break;

            case 'h':
                print_usage();
                exit(0);

            default:
                print_usage();
                return -1;
        }
    }
    
    // If xbox mode, last argument should be device path
    if (input_mode == INPUT_XBOX) {
        if (optind < argc) {
            xbox_device = argv[optind];
        } else {
            xbox_device = "/dev/input/js0";  // Default
        }
    }
    
    return 0;
}

/**
 * @brief Main entry point
 */
int main(int argc, char* argv[]) {
    pid_config_file_t pid_config;
    
    // Parse command-line arguments
    if (parse_args(argc, argv) < 0) {
        return -1;
    }
    
    // Set logging level based on quiet mode
    if (quiet_mode) {
        g_debug_config.logging.level = LOG_LEVEL_WARN;
    } else {
        g_debug_config.logging.level = LOG_LEVEL_INFO;
    }
    
    printf("\n");
    printf("╔═══════════════════════════════════════════════╗\n");
    printf("║         balance_bot v1.0                      ║\n");
    printf("║    Self-Balancing Robot + iPhone App         ║\n");
    printf("╚═══════════════════════════════════════════════╝\n");
    printf("\n");
    
    // Print configuration
    printf("Configuration:\n");
    printf("  Input mode: ");
    switch (input_mode) {
        case INPUT_CAT:    printf("Cat following\n"); break;
        case INPUT_XBOX:   printf("Xbox controller (%s)\n", xbox_device); break;
        case INPUT_SBUS:   printf("SBUS / FrSky R-XSR (%s)\n",
                               sbus_device ? sbus_device : "/dev/ttyO4"); break;
        case INPUT_NONE:   printf("Balance only\n"); break;
        case INPUT_MANUAL: printf("Manual\n"); break;
        default:           printf("Unknown\n"); break;
    }
    printf("  PID config: %s\n", pid_config_file ? pid_config_file : "pidconfig.txt");
    printf("  Log level: %s\n", quiet_mode ? "WARN" : "INFO");
    printf("\n");
    
    // Initialize debug configuration with defaults
    g_debug_config = get_default_debug_config();

    // Apply display flags from -d arguments
    g_debug_config.display.sbus_tx  = disp_sbus;
    g_debug_config.display.pid      = disp_pid;
    g_debug_config.display.encoders = disp_enc;
    g_debug_config.display.imu      = disp_imu;
    g_debug_config.display.motors   = disp_motors;
    g_debug_config.display.system   = disp_system;
    
    // Load PID configuration
    if (pid_config_load_or_default(pid_config_file, &pid_config) < 0) {
        fprintf(stderr, "Error: Failed to load PID configuration\n");
        return -1;
    }
    
    if (!quiet_mode) {
        pid_config_print(&pid_config);
    }
    
    // Initialize telemetry system
    LOG_INFO("Initializing telemetry system...");
    if (telemetry_init() < 0) {
        fprintf(stderr, "Error: Failed to initialize telemetry\n");
        return -1;
    }
    
    // Initialize IPC server for iPhone communication
    LOG_INFO("Starting IPC server...");
    if (ipc_server_init() < 0) {
        fprintf(stderr, "Error: Failed to start IPC server\n");
        return -1;
    }
    
    // Initialize robot (IMU, motors, encoders, etc.)
    LOG_INFO("Initializing robot hardware...");
    if (robot_init() < 0) {
        fprintf(stderr, "Error: Robot initialization failed\n");
        ipc_server_cleanup();
        return -1;
    }
    
    // Apply PID configuration
    pid_config_apply(&pid_config);
    
    // Initialize input mode
    switch (input_mode) {
        case INPUT_CAT:
            LOG_INFO("Initializing cat tracker...");
            if (cat_tracker_init() < 0) {
                LOG_WARN("Cat tracker not available (continuing without cat following)");
            }
            break;
            
        case INPUT_XBOX:
            LOG_INFO("Initializing Xbox controller...");
            if (xbox_init(xbox_device) < 0) {
                fprintf(stderr, "Error: Failed to initialize Xbox controller\n");
                robot_cleanup();
                ipc_server_cleanup();
                return -1;
            }
            break;

        case INPUT_SBUS:
            LOG_INFO("Initializing SBUS input (FrSky R-XSR)...");
            if (sbus_init(sbus_device) < 0) {
                fprintf(stderr, "Error: Failed to initialize SBUS on %s\n",
                        sbus_device ? sbus_device : "/dev/ttyO4");
                robot_cleanup();
                ipc_server_cleanup();
                return -1;
            }
            break;
        case INPUT_MANUAL:
            LOG_INFO("Manual mode not yet implemented");
            break;
        case INPUT_NONE:
            LOG_INFO("No input mode selected (balance only)");
            break;
    }
    
    printf("\n");
    printf("═══════════════════════════════════════════════\n");
    printf("  System ready! Press MODE button to arm.\n");
    printf("═══════════════════════════════════════════════\n");
    printf("\n");
    
    if (!quiet_mode) {
        printf("Controls:\n");
        printf("  MODE button  - Arm/disarm motors\n");
        printf("  PAUSE button - Toggle mode\n");
        if (input_mode == INPUT_XBOX) {
            printf("  Xbox A       - Arm/disarm\n");
            printf("  Xbox B       - Toggle mode\n");
            printf("  Xbox Start   - Exit\n");
        }
        printf("  Ctrl+C       - Exit\n");
        printf("\n");
        
        char telemetry_desc[256];
        telemetry_get_config_description(telemetry_desc, sizeof(telemetry_desc));
        printf("%s\n\n", telemetry_desc);
    }
    
    // Start ncurses display thread
    if (display_init() < 0) {
        LOG_WARN("Display thread failed to start — continuing without live display");
    }

    // Main loop
    robot_run();
    
    // Cleanup
    LOG_INFO("Shutting down...");
    display_cleanup();
    
    switch (input_mode) {
        case INPUT_CAT:
            cat_tracker_cleanup();
            break;
        case INPUT_XBOX:
            xbox_cleanup();
            break;
        case INPUT_SBUS:
            sbus_cleanup();
            break;
        default:
            break;
    }
    
    robot_cleanup();
    ipc_server_cleanup();
    
    printf("\nbalance_bot stopped.\n\n");
    
    return 0;
}
