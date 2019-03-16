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
