/*
 * @file HFP_profile.cpp
 * @brief HFP profile implementation
 *
 * This file implements HFP profile as per HFP 1.6 specification
 * Support for profile connection/disconnection/
 * slc/audio connection/SCO connection support for in-band ring is supported
 *
 * @author : vijay
 * @ingroup bt service
 *
                ______________________
                |   Application       |  
                |_____________________|
                |  HF Control         |   
                |_____________________|
                |  RFCOMM|SDP         |
                |_____________________|
                |   LMP | L2CAP       |
                |_____________________|
                |   Baseband          |
                |_____________________|

                ____________               __________
                |  HF       |              |  AG    |   
                |__________ |              |________|
                     |     RFComm              |
                     |<----------------------->| 
                     |------AT+BRSF----------->|        SLC
                     |<-------BRSF------------ |
                     |<-------OK-------------- |
                     |--------AT+BAC---------->|
                     |<--------OK------------- |
                     |----------AT+CIND=?------|
                     |<--------+CIND-----------|
                     |<---------OK-------------|
                     |--------AT+CIND?-------->|
                     |<--------+CIND-----------|
                     |<--------OK--------------|
                     |--------AT+CMER=-------->|
                     |<-------OK---------------|
                     |--------AT+CHLD=?------->|
                     |<-------+CHLD------------|
                     |<-------OK---------------|

                     |------->AT+BCC---------->|
                     |<-------OK---------------|
                     |<------+BCS------------->|    Audio connection
                     |--------AT+BCS=--------->|
                     |<-------OK---------------|

                     |<-------sco setup--------|    SCO 

*/
#define MAX_SIZE        10

/* HF and AG features Definitions. These values are concurrent with the ones listed in HFP spec */
/* used by AT+BRSF and SDP */
#define HF_NREC	                0x0001
#define HF_3WAY	                0x0002
#define HF_CLI	                0x0004
#define HF_VOICE_RECOGNITION	0x0008
#define HF_REMOTE_VOL	        0x0010
#define HF_ENHANCED_STATUS	    0x0020
#define HF_ENHANCED_CONTROL	    0x0040
#define HF_CODEC_NEGOTIATION	0x0080

/* bit 0 to 4 are the same as for AT+BRSF */
#define SDP_HF_AG_WIDEBAND_SPEECH	0x0020


#define AG_3WAY                 0x0001
#define AG_NREC                 0x0002
#define AG_VOICE_RECOGNITION    0x0004
#define AG_INBAND_RING          0x0008
#define AG_VOICE_TAG            0x0010
#define AG_REJECT_CALL          0x0020
#define AG_ENHANCED_STATUS      0x0040
#define AG_ENHANCED_CONTROL     0x0080
#define AG_EXTENDED_RESULT      0x0100
#define AG_CODEC_NEGOTIATION    0x0200



#define BASE_FEATURES (HF_NREC | HF_3WAY | HF_CLI | HF_VOICE_RECOGNITION | HF_REMOTE_VOL)
#define HF_FEATURES ( BASE_FEATURES | HF_ENHANCED_STATUS | HF_ENHANCED_CONTROL | HF_CODEC_NEGOTIATION)


#define 	WIDEBANDSPEECH_ENABLE	FALSE

/* The available codecs can be mSBC and CVSD(WBS).
   CVSD -  1
   mSBC -  2
   CVSD is mandatory
   */
#if (WIDEBANDSPEECH_ENABLE)
#define SDP_FEATURES (BASE_FEATURES | SDP_HF_AG_WIDEBAND_SPEECH)
#define AVAILABLE_CODECS "1,2"
#else
#define SDP_FEATURES (BASE_FEATURES)
#define AVAILABLE_CODECS "1"
/* What is this??? */
//#warning "mSBC disabled"
#endif

#define VOICE_RECOGNITION_ACTIVATED "1"
#define VOICE_RECOGNITION_DEACTIVATED "0"

/* Paramters to be sent with AT+CMER command. To activate the
   indicator updates ,i.e, +CIEV unsolicited responses, from AG*/
#define ACTIVATE_INDICATORS "3,0,0,1"
#define INVALID_EVT -1
#define MAX_AT_CMD_SIZE 128
#define PROFILE_HFP_PROFILE_PATH	"/vijay/vijayhfpprofile"

#define PROFILE_HFP_UUID            "0000111E-0000-1000-8000-00805f9b34fb"

#define CALL_IND "\"call\"" 
#define SERVICE_IND "\"service\""
#define CALL_SETUP_IND "\"callsetup\""
#define CALL_HELD_IND  "\"callheld\""
#define SIGNAL "\"signal\""
#define BATT_CHG "\"battchg\""


#define DEFAULT_IND_VALUE -1
#define DEFAULT_IND_POS 0

#define BD_ADDR_SIZE 18 

static gboolean            hfpCievUpdated = FALSE;
static gboolean            hfpFirstCallActive = FALSE;
static gboolean            processCallStateChange;
static gboolean            hfpCallHeld = FALSE;
static gboolean            secondIncomingCall = FALSE;
static gboolean            processCallSwap = FALSE;
static gboolean            process_call_held = FALSE;
static GIOChannel 	       *hfpChannel = NULL;
GDBusConnection            *hfpDbusConnection;
gchar                      hfpDevPath[256] = {0};
GInputStream *             hfpInputStream = NULL;
gboolean                   doSecRead = FALSE;

slc_struct hfp_slc;

// Conn Device list info for HFP
gint                        hfpConnIndex = -1;              //index to HFP device
gchar                       hfpBdAddr[BD_ADDR_SIZE] = {0};    //HFP Ref device BD_ADDr
gchar                       *hfpBdString = NULL;

static void INIT_IND (indicator_struct x) {x.value = DEFAULT_IND_VALUE; x.position = DEFAULT_IND_POS;}

#define HFP_SLC_BUF_SIZE_BYTES 256
#define HFP_SCO_BUF_SIZE_BYTES 672

static void hfpHandleIncomingRing(char *features);
static void hfpHandleAgFeatures(char *features);
static void hfpHandleCallHoldFeatures(char *features);
static void hfpIndStatusChange(char *features);
static int  hfpAssIndValues(int value, int pos);
static void hfpHandleCodecNeg(char *features);
static void hfpHandleRespHold(char *cmd);
static void hfpHandleVoiceRecognition(char *cmd);
static void hfpHandleInbandRingChange(char *cmd);
static int  hfpProcessCievUpdate();
static int  hfpSend(int rfcomm_fd, const char *buf);
static int  hfpHandleAgEvents(char *cmd, int fd, char *cmd_buf, char *params, int event);
static int  hfpSendAtCmf(char *cmd, int fd, char *cmd_buf, char *params, int event);
static int  hfpOkRespHandler(char *cmd, int fd, char *cmd_buf, char *params, int event);
static int  hfpScoOpen();
static void hfpStoreAgInd(char * features);
static void hfpSetAgIndValues(char *features);
static void hfpInitAgInd();
static void hfpInitSlc();

/*
 * Function to initialize all HF variables to default values. 
 * @param  : NONE
 * @retval : VOID
 */
static void hfpInitSlc()
{
    hfp_slc.hfp_fd = -1;
    hfp_slc.sco_fd = -1;

    hfp_slc.hfp_ag_supported_features = 0;

    hfp_slc.slc_established = false;
    hfp_slc.sco_established = false;

    hfp_slc.audio_connection_complete = false;
    hfp_slc.inband_ring = false;

    memset(hfp_slc.bd_string,0,BD_ADDR_SIZE);
    process_call_state_change = FALSE;

    hfp_init_ag_indicators();

    hfp_bd_string = NULL;
    hfp_curr = hfp_map;
    hfp_nAgEvt = hfp_ag_event_map;

    process_call_swap =FALSE;
    hfp_call_held = FALSE;
    //HFP disconnected, reset the state in Conn Dev list.
    updateConnStatus(HFP_PROFILE,FALSE,hfp_conn_index);
    //Index to Conn Dev list. Has to be reset when Connection is no longer present 
    //to that device.
    hfpConnIndexReset();
}

/*!
 * Register the HFP profile with the BlueZ daemon
 * @param  : GDBusProxy *dBusProxy                               - Proxy object for org.bluez.Profile
 *          GDBusObjectManagerServer *dBusManager               - DbusManager object
 *          GDBusConnection *conn                               - object to Dbus Connection ,initiated in calling function
 *                                                                Value to be stored for future use.
 * @retval : gboolean 
 *          TRUE                                                -Returned in the case of Success.
 *          FALSE                                               -Returned in the case of Failure.
 */
gboolean hfpProfileRegister (GDBusProxy *dBusProxy, GDBusObjectManagerServer *dBusManager,GDBusConnection *conn)
{
    /*< Register HFP handling Object */
    GVariantBuilder                         *HFPcall;	
    GVariantBuilder                         *HFPparams;		
    GVariant                                *HFPvParams, *HFPvCall;	
    GVariant                                *dBusReply = NULL;	
    GError                                  *dBusError = NULL; 	
    static BluezProfileObjectSkeleton       *bluezHFPProfileObject;
    static BluezProfileProfile1             *bluezHFPProfile;

    hfp_dbus_connection = conn;
    BT_HFP_LOGS("  %s\n",__func__);
    hfpBuffer = buffer_init(HFP_SLC_BUF_SIZE_BYTES*2, "hfpBuffer");
    bluezHFPProfileObject = bluez_profile_object_skeleton_new (PROFILE_HFP_PROFILE_PATH);

    if (bluezHFPProfileObject == NULL)
    {
        g_printerr ("Could not create bluezProfile for HFP\n");
        return FALSE;
    }

    bluezHFPProfile = bluez_profile_profile1_skeleton_new ();
    bluez_profile_object_skeleton_set_profile1 (bluezHFPProfileObject, bluezHFPProfile);
    g_signal_connect (bluezHFPProfile, "handle-release", G_CALLBACK(bluez_hfpprofile_handle_release), NULL);
    g_signal_connect (bluezHFPProfile, "handle-new-connection", G_CALLBACK(bluez_hfpprofile_handle_new_connection), NULL);
    g_signal_connect (bluezHFPProfile, "handle-request-disconnection", G_CALLBACK(bluez_hfpprofile_handle_request_disconnection), NULL);
    g_dbus_object_manager_server_export (dBusManager, G_DBUS_OBJECT_SKELETON (bluezHFPProfileObject));

    /*<
      ******************************************************
      * Register Profile for HFP on DBus
      *****************************************************
    */

    HFPcall = g_variant_builder_new (G_VARIANT_TYPE_TUPLE);
    g_variant_builder_add_value (HFPcall, g_variant_new_object_path (PROFILE_HFP_PROFILE_PATH));
    g_variant_builder_add_value (HFPcall, g_variant_new_string (PROFILE_HFP_UUID));
    HFPparams = g_variant_builder_new (G_VARIANT_TYPE_VARDICT);
    g_variant_builder_add_value (HFPparams, g_variant_new_dict_entry (g_variant_new_string("Name"), \
                                g_variant_new_variant(g_variant_new_string("Hands-Free unit"))));
    g_variant_builder_add_value (HFPparams, g_variant_new_dict_entry (g_variant_new_string("Features"), \
                                g_variant_new_variant(g_variant_new_uint16(SDP_FEATURES))));
    g_variant_builder_add_value (HFPparams, g_variant_new_dict_entry (g_variant_new_string("RequireAuthorization"), \
                                g_variant_new_variant(g_variant_new_boolean(TRUE))));
    g_variant_builder_add_value (HFPparams, g_variant_new_dict_entry (g_variant_new_string("RequireAuthentication"), \
                                g_variant_new_variant(g_variant_new_boolean(TRUE))));
    /*< Support Version 1.6 of HFP */
    g_variant_builder_add_value (HFPparams, g_variant_new_dict_entry (g_variant_new_string("Version"), g_variant_new_variant(g_variant_new_uint16(0x106)))); 
    /*< This is the fixed RFCOMM channel number that Bluez uses for HFP-HF. Numbers given in bluez "assigned-numbers.txt" doc */
    
    HFPvParams = g_variant_builder_end(HFPparams);
    g_variant_builder_add_value (HFPcall, HFPvParams);
    HFPvCall = g_variant_builder_end(HFPcall);

    dBusReply = g_dbus_proxy_call_sync (dBusProxy, "RegisterProfile", \
            HFPvCall, G_DBUS_CALL_FLAGS_NONE, -1, NULL, &dBusError);
    g_variant_builder_unref(HFPcall);
    g_variant_builder_unref(HFPparams);

    if (dBusReply == NULL)
    {
        g_printerr ("Failed to register Profile for HFP on Bluez: %s\n", dBusError->message);
        g_error_free (dBusError); 
            
        return FALSE;
    }
    g_variant_unref(dBusReply);
    hFpInitSlc();
    return TRUE;
}
/*! events from bluez and events for statemachine map */
typedef struct {
    char *cmd;    
    int event;
    void (*agRespHandler) (char *params);
}hfpEventBufMap;

/*!
 * HFP States
 */
typedef enum 
{
    HFP_INIT =0,
    ESTABLISH_SLC,
    ESTABLISH_AUDIO,
    SCO_CONNECTED
}hfpStates;

/*!
    * stae machine struct 
*/
typedef struct
{
    int event;
    int nextState;
    char *cmd;
    char *params;
    int (*fn)(char *cmd, int fd, char *cmd_buf, char * params, int event);
}hfpStateTable;

typedef struct
{
    hfpStateTable *state;
}hfpSm;

typedef struct 
{
    int currentState;
}hfStateInfo;


typedef enum
{
    HF_INIT_EVENT = 0,
    AT_BRSF = 0,
    BRSF_   = 0,
    BRSF_OK,
    AT_BAC,
    AT_BAC_OK,
    AT_CIND,
    CIND,
    AT_CIND_OK,
    AT_CIND_VAL_CMD,
    AT_CIND_VAL,
    AT_CIND_VAL_OK,
    AT_CMER,
    AT_CMER_OK,
    AT_CHLD,
    CHLD,
    CHLD_OK,
    AT_BCC,
    AT_BCC_OK,
    BCS_,
    AT_BCS,
    AT_BCS_OK,
    BTRH,
    BSIR,
    BVRA,
    CIEV,
    RING,
    MAX_EVENTS
}hfpEvents;

    /* cmd          event               handler*/
hfpEventBufMap hfp_map [] =
{
    {"+BRSF"       ,BRSF_,              NULL},
    {"OK"          ,BRSF_OK,            NULL},
    {"AT+BAC="     ,AT_BAC,             NULL},
    {"OK"          ,AT_BAC_OK,          NULL},
    {"AT+CIND=?\r" ,AT_CIND,            NULL},
    {"+CIND"       ,CIND,               NULL},
    {"OK"          ,AT_CIND_OK,         NULL},
    {"AT+CIND?\r"  ,AT_CIND_VAL_CMD,    NULL},
    {"+CIND"       ,AT_CIND_VAL,        NULL},
    {"OK"          ,AT_CIND_VAL_OK,     NULL},
    {"AT+CMER="    ,AT_CMER,            NULL},
    {"OK"          ,AT_CMER_OK,         NULL},
    {"AT+CHLD=?\r" ,AT_CHLD,            NULL},
    {"+CHLD"       ,CHLD,               NULL},
    {"OK"          ,CHLD_OK,            NULL},
    {"AT+BCC\r"    ,AT_BCC,             NULL},
    {"OK"          ,AT_BCC_OK,          NULL},
    {"+BCS"        ,BCS_,               NULL},
    {"AT+BCS="     ,AT_BCS,             NULL},
    {"OK"          ,AT_BCS_OK,          NULL},
    {"+BTRH"       ,BTRH,               NULL},
    {"+BSIR"       ,BSIR,               NULL},
    {"+BVRA"       ,BVRA,               NULL},
    {"+CIEV"       ,CIEV,               NULL},
    {"RING"        ,RING,               NULL},
};

