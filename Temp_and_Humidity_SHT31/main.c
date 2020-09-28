#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <getopt.h>
#include <errno.h>
#include <stdio.h>

#include <applibs/log.h>
#include <applibs/gpio.h>
#include <applibs/eventloop.h>
#include <applibs/networking.h>

#include "../MT3620_Grove_Shield_Library/Grove.h"
#include "../MT3620_Grove_Shield_Library/Sensors/GroveTempHumiSHT31.h"
#include "../MT3620_Grove_Shield_Library/Sensors/GroveLightSensor.h"
#include "../MT3620_Grove_Shield_Library/Sensors/GroveAD7992.h"
#include "../MT3620_Grove_Shield_Library/Sensors/GroveOledDisplay96x96.h"
#include "../MT3620_Grove_Shield_Library/Sensors/GroveRelay.h"

// Azure IoT SDK
#include <azureiot/iothub_client_core_common.h>
#include <azureiot/iothub_device_client_ll.h>
#include <azureiot/iothub_client_options.h>
#include <azureiot/iothubtransportmqtt.h>
#include <azureiot/iothub.h>
#include <azureiot/azure_sphere_provisioning.h>
#include <azure_prov_client/iothub_security_factory.h>

/// <summary>
/// Exit codes for this application. These are used for the
/// application exit code. They must all be between zero and 255,
/// where zero is reserved for successful termination.
/// </summary>
typedef enum
{
    ExitCode_Success = 0,

    ExitCode_TermHandler_SigTerm = 1,

    ExitCode_Main_EventLoopFail = 2,

    ExitCode_ButtonTimer_Consume = 3,

    ExitCode_AzureTimer_Consume = 4,

    ExitCode_Init_EventLoop = 5,
    ExitCode_Init_MessageButton = 6,
    ExitCode_Init_OrientationButton = 7,
    ExitCode_Init_TwinStatusLed = 8,
    ExitCode_Init_ButtonPollTimer = 9,
    ExitCode_Init_AzureTimer = 10,

    ExitCode_IsButtonPressed_GetValue = 11,

    ExitCode_Validate_ConnectionType = 12,
    ExitCode_Validate_ScopeId = 13,
    ExitCode_Validate_IotHubHostname = 14,
    ExitCode_Validate_DeviceId = 15,

    ExitCode_InterfaceConnectionStatus_Failed = 16,
} ExitCode;

/// <summary>
/// Connection types to use when connecting to the Azure IoT Hub.
/// </summary>
typedef enum
{
    ConnectionType_NotDefined = 0,
    ConnectionType_DPS = 1,
    ConnectionType_Direct = 2
} ConnectionType;

/// <summary>
/// Authentication state of the client with respect to the Azure IoT Hub.
/// </summary>
typedef enum
{
    /// <summary>Client is not authenticated by the Azure IoT Hub.</summary>
    IoTHubClientAuthenticationState_NotAuthenticated = 0,
    /// <summary>Client has initiated authentication to the Azure IoT Hub.</summary>
    IoTHubClientAuthenticationState_AuthenticationInitiated = 1,
    /// <summary>Client is authenticated by the Azure IoT Hub.</summary>
    IoTHubClientAuthenticationState_Authenticated = 2
} IoTHubClientAuthenticationState;

// Azure IoT definitions.
static char *scopeId = NULL;                                      // ScopeId for DPS.
static char *hubHostName = NULL;                                  // Azure IoT Hub Hostname.
static char *deviceId = NULL;                                     // Device ID must be in lowercase.
static ConnectionType connectionType = ConnectionType_NotDefined; // Type of connection to use.
static IoTHubClientAuthenticationState iotHubClientAuthenticationState =
    IoTHubClientAuthenticationState_NotAuthenticated; // Authentication state with respect to the
                                                      // IoT Hub.

static IOTHUB_DEVICE_CLIENT_LL_HANDLE iothubClientHandle = NULL;
static const int deviceIdForDaaCertUsage = 1; // A constant used to direct the IoT SDK to use
                                              // the DAA cert under the hood.
static const char NetworkInterface[] = "wlan0";

static const void *relay  = NULL;

static void ParseCommandLineArguments(int argc, char *argv[]);
static void SendTelemetry(const char *jsonMessage);
static bool SetUpAzureIoTHubClientWithDps(void);
static void SendSimulatedTelemetry(void);
static const char *GetAzureSphereProvisioningResultString(AZURE_SPHERE_PROV_RETURN_VALUE provisioningResult);

static void SendEventCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void *context);
static bool IsConnectionReadyToSendTelemetry(void);
static const char *GetReasonString(IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason);

static int DeviceMethodCallback(const char *methodName, const unsigned char *payload,
                                size_t payloadSize, unsigned char **response, size_t *responseSize,
                                void *userContextCallback);

#define TELEMETRY_BUFFER_SIZE 100

// This C application for the MT3620 Reference Development Board (Azure Sphere)
// outputs a string every second to Visual Studio's Device Output window
//
// It uses the API for the following Azure Sphere application libraries:
// - log (messages shown in Visual Studio's Device Output window during debugging)

static volatile sig_atomic_t exitCode = ExitCode_Success;

/// <summary>
///     Signal handler for termination requests. This handler must be async-signal-safe.
/// </summary>
static void TerminationHandler(int signalNumber)
{
    // Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
    exitCode = ExitCode_TermHandler_SigTerm;
}

/// <summary>
///     Parse the command line arguments given in the application manifest.
/// </summary>
static void ParseCommandLineArguments(int argc, char *argv[])
{
    int option = 0;
    static const struct option cmdLineOptions[] = {{"ConnectionType", required_argument, NULL, 'c'},
                                                   {"ScopeID", required_argument, NULL, 's'},
                                                   {"Hostname", required_argument, NULL, 'h'},
                                                   {"DeviceID", required_argument, NULL, 'd'},
                                                   {NULL, 0, NULL, 0}};

    // Loop over all of the options.
    while ((option = getopt_long(argc, argv, "c:s:h:d:", cmdLineOptions, NULL)) != -1)
    {
        // Check if arguments are missing. Every option requires an argument.
        if (optarg != NULL && optarg[0] == '-')
        {
            Log_Debug("WARNING: Option %c requires an argument\n", option);
            continue;
        }
        switch (option)
        {
        case 'c':
            Log_Debug("ConnectionType: %s\n", optarg);
            if (strcmp(optarg, "DPS") == 0)
            {
                connectionType = ConnectionType_DPS;
            }
            else if (strcmp(optarg, "Direct") == 0)
            {
                connectionType = ConnectionType_Direct;
            }
            break;
        case 's':
            Log_Debug("ScopeID: %s\n", optarg);
            scopeId = optarg;
            break;
        case 'h':
            Log_Debug("Hostname: %s\n", optarg);
            hubHostName = optarg;
            break;
        case 'd':
            Log_Debug("DeviceID: %s\n", optarg);
            deviceId = optarg;
            break;
        default:
            // Unknown options are ignored.
            break;
        }
    }
}

/// <summary>
///     Main entry point for this sample.
/// </summary>
int main(int argc, char *argv[])
{
    Log_Debug("Application starting\n");

    ParseCommandLineArguments(argc, argv);

    // Register a SIGTERM handler for termination requests
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = TerminationHandler;
    sigaction(SIGTERM, &action, NULL);

    int i2cFd;
    GroveShield_Initialize(&i2cFd, 115200);

    void *sht31 = GroveTempHumiSHT31_Open(i2cFd);
    void *lightSensor = GroveLightSensor_Init(i2cFd, 0);
    GroveOledDisplay_Init(i2cFd, SH1107G);

    relay = GroveRelay_Open(4);

    setNormalDisplay();
    setVerticalMode();

    char telemetryBuffer[100];
    int len = -1;

    time_t t;
    struct tm *lt;

    // Check whether the device is connected to the internet.
    Networking_InterfaceConnectionStatus status;

    // // Main loop
    while (exitCode == ExitCode_Success)
    {
        if (Networking_GetInterfaceConnectionStatus(NetworkInterface, &status) == 0)
        {
            if ((status & Networking_InterfaceConnectionStatus_ConnectedToInternet) &&
                (iotHubClientAuthenticationState == IoTHubClientAuthenticationState_NotAuthenticated))
            {
                SetUpAzureIoTHubClientWithDps();
            }
        }
        else
        {
            if (errno != EAGAIN)
            {
                Log_Debug("ERROR: Networking_GetInterfaceConnectionStatus: %d (%s)\n", errno,
                          strerror(errno));
                exitCode = ExitCode_InterfaceConnectionStatus_Failed;
                continue;
            }
        }

        GroveTempHumiSHT31_Read(sht31);
        float temp = GroveTempHumiSHT31_GetTemperature(sht31);
        float humi = GroveTempHumiSHT31_GetHumidity(sht31);
        float light = GroveLightSensor_Read(lightSensor);
        light = GroveAD7992_ConvertToMillisVolt(light);

        clearDisplay();

        for (uint8_t i = 0; i < 16; i++)
        {
            setGrayLevel(i); //Set Grayscale level. Any number between 0 - 15.

            if (i == 3) //234
            {
                setTextXY(i, 8);
                putString("Temp:");
                setTextXY(i, 64);
                putNumber((uint16_t)temp);
            }

            if (i == 8) //789
            {
                setTextXY(i, 8);
                putString("Humi:");
                setTextXY(i, 64);
                putNumber((uint16_t)humi);
            }

            if (i == 13) //12,13,14
            {
                setTextXY(i, 8);
                putString("Light:");
                setTextXY(i, 64);
                putNumber((uint16_t)light);
            }
        }

        time(&t);
        lt = localtime(&t);

        len = snprintf(telemetryBuffer, 100, "{\"Time\":\"%d%d%d %d:%d:%d\",\"Temperature\":%.1f,\"Humidity\":%.1f,\"Light\":%.1f}",
                       lt->tm_year + 1900, lt->tm_mon + 1, lt->tm_mday, lt->tm_hour + 8, lt->tm_min, lt->tm_sec, temp, humi, light);
        if (len < 0 || len >= 100)
        {
            Log_Debug("ERROR: Cannot write telemetry to buffer.\n");
            continue;
        }

        //SendSimulatedTelemetry();

        SendTelemetry(telemetryBuffer);

        usleep(5000000);
    }

    Log_Debug("Application exiting\n");
    return 0;
}

/// <summary>
///     Callback when the Azure IoT connection state changes.
///     This can indicate that a new connection attempt has succeeded or failed.
///     It can also indicate than an existing connection has expired due to SAS token expiry.
/// </summary>
static void ConnectionStatusCallback(IOTHUB_CLIENT_CONNECTION_STATUS result,
                                     IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason,
                                     void *userContextCallback)
{
    Log_Debug("Azure IoT connection status: %s\n", GetReasonString(reason));

    if (result != IOTHUB_CLIENT_CONNECTION_AUTHENTICATED)
    {
        iotHubClientAuthenticationState = IoTHubClientAuthenticationState_NotAuthenticated;
        return;
    }

    iotHubClientAuthenticationState = IoTHubClientAuthenticationState_Authenticated;
}

/// <summary>
///     Callback invoked when a Direct Method is received from Azure IoT Hub.
/// </summary>
static int DeviceMethodCallback(const char *methodName, const unsigned char *payload,
                                size_t payloadSize, unsigned char **response, size_t *responseSize,
                                void *userContextCallback)
{
    int result;
    char *responseString;

    Log_Debug("Received Device Method callback: Method name %s.\n", methodName);

    if (strcmp("TriggerAlarm", methodName) == 0) {
        // Output alarm using Log_Debug
        Log_Debug("  ----- ALARM TRIGGERED! -----\n");

        GroveRelay_On(relay);
        usleep(1000000);
        GroveRelay_Off(relay);

        responseString = "\"Alarm Triggered\""; // must be a JSON string (in quotes)
        result = 200;
    } else {
        // All other method names are ignored
        responseString = "{}";
        result = -1;
    }

    // if 'response' is non-NULL, the Azure IoT library frees it after use, so copy it to heap
    *responseSize = strlen(responseString);
    *response = malloc(*responseSize);
    memcpy(*response, responseString, *responseSize);
    return result;
}

/// <summary>
///     Sets up the Azure IoT Hub connection (creates the iothubClientHandle)
///     with DPS
/// </summary>
static bool SetUpAzureIoTHubClientWithDps(void)
{

    if (iothubClientHandle != NULL)
    {
        IoTHubDeviceClient_LL_Destroy(iothubClientHandle);
    }

    AZURE_SPHERE_PROV_RETURN_VALUE provResult =
        IoTHubDeviceClient_LL_CreateWithAzureSphereDeviceAuthProvisioning(scopeId, 10000, &iothubClientHandle);
    Log_Debug("IoTHubDeviceClient_LL_CreateWithAzureSphereDeviceAuthProvisioning returned '%s'.\n",
              GetAzureSphereProvisioningResultString(provResult));

    if (provResult.result != AZURE_SPHERE_PROV_RESULT_OK)
    {
        return false;
    }

    iotHubClientAuthenticationState = IoTHubClientAuthenticationState_Authenticated;

    IoTHubDeviceClient_LL_SetConnectionStatusCallback(iothubClientHandle, ConnectionStatusCallback, NULL);
    IoTHubDeviceClient_LL_SetDeviceMethodCallback(iothubClientHandle, DeviceMethodCallback, NULL);


    return true;
}

/// <summary>
///     Generate simulated telemetry and send to Azure IoT Hub.
/// </summary>
void SendSimulatedTelemetry(void)
{
    // Generate a simulated temperature.
    static float temperature = 50.0f;                    // starting temperature
    float delta = ((float)(rand() % 41)) / 20.0f - 1.0f; // between -1.0 and +1.0
    temperature += delta;

    char telemetryBuffer[TELEMETRY_BUFFER_SIZE];
    int len =
        snprintf(telemetryBuffer, TELEMETRY_BUFFER_SIZE, "{\"Temperature\":%3.2f}", temperature);
    if (len < 0 || len >= TELEMETRY_BUFFER_SIZE)
    {
        Log_Debug("ERROR: Cannot write telemetry to buffer.\n");
        return;
    }
    SendTelemetry(telemetryBuffer);
}

/// <summary>
///     Sends telemetry to Azure IoT Hub
/// </summary>
static void SendTelemetry(const char *jsonMessage)
{
    if (iotHubClientAuthenticationState != IoTHubClientAuthenticationState_Authenticated)
    {
        // AzureIoT client is not authenticated. Log a warning and return.
        Log_Debug("WARNING: Azure IoT Hub is not authenticated. Not sending telemetry.\n");
        return;
    }

    Log_Debug("Sending Azure IoT Hub telemetry: %s.\n", jsonMessage);

    // Check whether the device is connected to the internet.
    if (IsConnectionReadyToSendTelemetry() == false)
    {
        return;
    }

    IOTHUB_MESSAGE_HANDLE messageHandle = IoTHubMessage_CreateFromString(jsonMessage);
    if (messageHandle == 0)
    {
        Log_Debug("ERROR: unable to create a new IoTHubMessage.\n");
        return;
    }

    if (IoTHubDeviceClient_LL_SendEventAsync(iothubClientHandle, messageHandle, SendEventCallback,
                                             /*&callback_param*/ NULL) != IOTHUB_CLIENT_OK)
    {
        Log_Debug("ERROR: failure requesting IoTHubClient to send telemetry event.\n");
    }
    else
    {
        Log_Debug("INFO: IoTHubClient accepted the telemetry event for delivery.\n");
    }

    IoTHubMessage_Destroy(messageHandle);

    if (iothubClientHandle != NULL)
    {
        IoTHubDeviceClient_LL_DoWork(iothubClientHandle);
    }
}

/// <summary>
///     Converts AZURE_SPHERE_PROV_RETURN_VALUE to a string.
/// </summary>
static const char *GetAzureSphereProvisioningResultString(
    AZURE_SPHERE_PROV_RETURN_VALUE provisioningResult)
{
    switch (provisioningResult.result)
    {
    case AZURE_SPHERE_PROV_RESULT_OK:
        return "AZURE_SPHERE_PROV_RESULT_OK";
    case AZURE_SPHERE_PROV_RESULT_INVALID_PARAM:
        return "AZURE_SPHERE_PROV_RESULT_INVALID_PARAM";
    case AZURE_SPHERE_PROV_RESULT_NETWORK_NOT_READY:
        return "AZURE_SPHERE_PROV_RESULT_NETWORK_NOT_READY";
    case AZURE_SPHERE_PROV_RESULT_DEVICEAUTH_NOT_READY:
        return "AZURE_SPHERE_PROV_RESULT_DEVICEAUTH_NOT_READY";
    case AZURE_SPHERE_PROV_RESULT_PROV_DEVICE_ERROR:
        return "AZURE_SPHERE_PROV_RESULT_PROV_DEVICE_ERROR";
    case AZURE_SPHERE_PROV_RESULT_GENERIC_ERROR:
        return "AZURE_SPHERE_PROV_RESULT_GENERIC_ERROR";
    default:
        return "UNKNOWN_RETURN_VALUE";
    }
}

/// <summary>
///     Callback invoked when the Azure IoT Hub send event request is processed.
/// </summary>
static void SendEventCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void *context)
{
    Log_Debug("INFO: Azure IoT Hub send telemetry event callback: status code %d.\n", result);
}

/// <summary>
///     Check the network status.
/// </summary>
static bool IsConnectionReadyToSendTelemetry(void)
{
    Networking_InterfaceConnectionStatus status;
    if (Networking_GetInterfaceConnectionStatus(NetworkInterface, &status) != 0)
    {
        if (errno != EAGAIN)
        {
            Log_Debug("ERROR: Networking_GetInterfaceConnectionStatus: %d (%s)\n", errno,
                      strerror(errno));
            exitCode = ExitCode_InterfaceConnectionStatus_Failed;
            return false;
        }
        Log_Debug(
            "WARNING: Cannot send Azure IoT Hub telemetry because the networking stack isn't ready "
            "yet.\n");
        return false;
    }

    if ((status & Networking_InterfaceConnectionStatus_ConnectedToInternet) == 0)
    {
        Log_Debug(
            "WARNING: Cannot send Azure IoT Hub telemetry because the device is not connected to "
            "the internet.\n");
        return false;
    }

    return true;
}

/// <summary>
///     Converts the Azure IoT Hub connection status reason to a string.
/// </summary>
static const char *GetReasonString(IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason)
{
    static char *reasonString = "unknown reason";
    switch (reason)
    {
    case IOTHUB_CLIENT_CONNECTION_EXPIRED_SAS_TOKEN:
        reasonString = "IOTHUB_CLIENT_CONNECTION_EXPIRED_SAS_TOKEN";
        break;
    case IOTHUB_CLIENT_CONNECTION_DEVICE_DISABLED:
        reasonString = "IOTHUB_CLIENT_CONNECTION_DEVICE_DISABLED";
        break;
    case IOTHUB_CLIENT_CONNECTION_BAD_CREDENTIAL:
        reasonString = "IOTHUB_CLIENT_CONNECTION_BAD_CREDENTIAL";
        break;
    case IOTHUB_CLIENT_CONNECTION_RETRY_EXPIRED:
        reasonString = "IOTHUB_CLIENT_CONNECTION_RETRY_EXPIRED";
        break;
    case IOTHUB_CLIENT_CONNECTION_NO_NETWORK:
        reasonString = "IOTHUB_CLIENT_CONNECTION_NO_NETWORK";
        break;
    case IOTHUB_CLIENT_CONNECTION_COMMUNICATION_ERROR:
        reasonString = "IOTHUB_CLIENT_CONNECTION_COMMUNICATION_ERROR";
        break;
    case IOTHUB_CLIENT_CONNECTION_OK:
        reasonString = "IOTHUB_CLIENT_CONNECTION_OK";
        break;
    case IOTHUB_CLIENT_CONNECTION_NO_PING_RESPONSE:
        reasonString = "IOTHUB_CLIENT_CONNECTION_NO_PING_RESPONSE";
        break;
    }
    return reasonString;
}