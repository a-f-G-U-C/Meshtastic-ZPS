/*
 * ZPS - Zero-GPS Positioning System for standalone Meshtastic devices
 * - experimental tools for estimating own position without a GPS -
 *
 * Copyright 2021 all rights reserved by https://github.com/a-f-G-U-C 
 * Released under GPL v3 (see LICENSE file for details)
 */

#include "configuration.h"
#include "ZPSPlugin.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "RTC.h"
#include "Router.h"
#include <WiFi.h>

#include "esp_nimble_hci.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"

#define BLE_MAX_REC 15
#define BLE_NO_RESULTS -1  // Indicates a BLE scan is in progress
#define ZPS_SEND_EVERY 30

#define ZPS_EXTRAVERBOSE

uint8_t bleCounter = 0;  // used internally by the ble scanner
uint64_t bleResult[BLE_MAX_REC+1];
int bleResSize = BLE_NO_RESULTS;

uint64_t scanStart = 0;

// Mini BLE scanner, NIMBLE based and modelled loosely after the Wifi scanner
static int ble_scan(uint32_t duration, bool passive=true, bool dedup=true);


ZPSPlugin::ZPSPlugin()
    : ProtobufPlugin("ZPS", ZPS_PORTNUM, Position_fields), concurrency::OSThread("ZPSPlugin")
{
    setIntervalFromNow(ZPS_STARTUP_DELAY); // Delay startup by 10 seconds, no need to race :)

    wantBSS = true;
    wantBLE = true;

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.scanNetworks(true, true);  // nonblock, showhidden
    scanState = SCAN_BSS_RUN;

    // FIXME some (de)initialization of BLE may be required
}


bool ZPSPlugin::handleReceivedProtobuf(const MeshPacket &mp, Position *pptr)
{
    auto p = *pptr;

    DEBUG_MSG("ZPSPlugin::handleReceivedProtobuf\n");

    // Log packet size and list of fields, same as position plugin
    DEBUG_MSG("ZPS solution from node=%08x l=%d %s%s%s%s%s%s%s%s%s%s%s%s%s%s\n", 
                getFrom(&mp),
            	mp.decoded.payload.size,
                p.latitude_i ? "LAT ":"",
                p.longitude_i ? "LON ":"",
                p.altitude ? "MSL ":"",
                p.altitude_hae ? "HAE ":"",
                p.alt_geoid_sep ? "GEO ":"",
                p.PDOP ? "PDOP ":"",
                p.HDOP ? "HDOP ":"",
                p.VDOP ? "VDOP ":"",
                p.sats_in_view ? "SIV ":"",
                p.fix_quality ? "FXQ ":"",
                p.fix_type ? "FXT ":"",
                p.pos_timestamp ? "PTS ":"",
                p.time ? "TIME ":"",
                p.battery_level ? "BAT ":"");

    if (p.time) {
        struct timeval tv;
        uint32_t secs = p.time;

        tv.tv_sec = secs;
        tv.tv_usec = 0;

        perhapsSetRTC(RTCQualityFromNet, &tv);
    }

    NodeInfo *node = nodeDB.getNode(nodeDB.getNodeNum());
    if (p.location_source) {
        
        // FIXME should be conditional, to ensure we don't overwrite a good GPS fix!

        DEBUG_MSG("Set lon/lat/dop/pts %d/%d/%d/%d\n", 
                    p.latitude_i, p.longitude_i, p.PDOP, p.pos_timestamp);
        nodeDB.updatePosition(node->num, p);
    }

    return false; // Let others look at this message also if they want
}


MeshPacket *ZPSPlugin::allocReply()
{
    MeshPacket *p = allocDataPacket();
    p->decoded.payload.size = 8 * (netRecs + 2);  // actually can be only +1 if no GPS data

    DEBUG_MSG("Allocating dataPacket for %d items, %d bytes\n", netRecs, p->decoded.payload.size);
    memcpy(p->decoded.payload.bytes, &netData, p->decoded.payload.size);

    return (p);
}


void ZPSPlugin::sendDataPacket(NodeNum dest, bool wantReplies)
{
    // cancel any not yet sent (now stale) position packets
    if (prevPacketId)
        service.cancelSending(prevPacketId);

    MeshPacket *p = allocReply();
    p->to = dest;
    p->decoded.portnum = PortNum_PRIVATE_APP;
    p->decoded.want_response = wantReplies;
    p->priority = MeshPacket_Priority_BACKGROUND;
    prevPacketId = p->id;

    service.sendToMesh(p, RX_SRC_LOCAL);
}


int32_t ZPSPlugin::runOnce()
{
    NodeInfo *node = nodeDB.getNode(nodeDB.getNodeNum());
    assert(node);

    // DEBUG_MSG("ZPSPlugin::runOnce() START, scanState: %d\n", (int) scanState);

    // Check if we have a GPS position
    // FIXME this part is completely undeveloped
    if (node->position.fix_type)
        DEBUG_MSG("GPS fix type: %d, %ds ago\n", node->position.fix_type, node->position.pos_timestamp);

    int numWifi = 0;

    if (scanState == SCAN_BSS_RUN) {
        // check completion status of any running Wifi scan
        numWifi = WiFi.scanComplete();

        // FIXME will this work correctly if ZERO networks are found? 

        if (numWifi >= 0) {
            // scan is complete
            DEBUG_MSG("%d BSS found\n", numWifi);
            DEBUG_MSG("BSS scan done in %d millis\n", millis() - scanStart);

            if (wantBSS && haveBSS) {
                // old data exists, overwrite it
                netRecs = 0;
                haveBSS = haveBLE = false;
            }

            for (int i = 0; i < numWifi; i++) {
                // pack each Wifi network record into a 64-bit int
                uint64_t netBytes = encodeBSS(WiFi.BSSID(i), WiFi.channel(i), abs(WiFi.RSSI(i)));

                if (wantBSS) {
                    // load into outbound array if needed
                    outBufAdd(netBytes);
                    haveBSS = true;
                }
#ifdef ZPS_EXTRAVERBOSE
                DEBUG_MSG("BSS[%02d]: %08x" "%08x\n", i, (uint32_t)(netBytes>>32), (uint32_t)netBytes);
#endif
            }

            WiFi.scanDelete();
            scanState = SCAN_BSS_DONE;

#ifdef ZPS_EXTRAVERBOSE
        } else if (numWifi == -1) {
            // DEBUG_MSG("BSS scan in-progress\n");
        } else {
            DEBUG_MSG("BSS scan state=%d\n", numWifi);
#endif
        }
    } 

    if ((scanState == SCAN_BLE_RUN) && (bleResSize>=0)) {
        // completion status checked above (bleResSize >= 0)
        DEBUG_MSG("BLE scan done in %d millis\n", millis() - scanStart);
        scanState = SCAN_BLE_DONE;

        // FIXME will this work correctly if ZERO networks are found? 

        if (wantBLE && haveBLE) {
            // old data exists, overwrite it
            netRecs = 0;
            haveBSS = haveBLE = false;
        }

        for (int i=0; i < bleResSize; i++) {
            // load data into output array if needed
            if (wantBLE) {
                outBufAdd(bleResult[i]);
                haveBLE = true;
            }
#ifdef ZPS_EXTRAVERBOSE
            DEBUG_MSG("BLE[%d]: %08x" "%08x\n", i, (uint32_t)(bleResult[i]>>32), (uint32_t)bleResult[i]);
#endif
        }

        // Reset the counter once we're done with the dataset
        bleResSize = BLE_NO_RESULTS;
    }

    // Are we finished assembling that packet? Then send it out
    if ((wantBSS == haveBSS) && (wantBLE == haveBLE) &&
            (lastSend == 0 || millis() - lastSend >= ZPS_SEND_EVERY * 1000)) {

        haveBSS = haveBLE = false;
        sendDataPacket(NODENUM_BROADCAST, false);  // no replies
        lastSend = millis();
        netRecs = 0;  // reset packet
    }

    /*
     * State machine transitions
     *
     * FIXME could be managed better, for example: check if we require
     *   each type of scan (wantBSS/wantBLE), and if not, don't start it!
     */
    if (scanState == SCAN_BLE_DONE) {
        // BLE done, transition to BSS scanning
        scanStart = millis();
        DEBUG_MSG("BSS scan start t=%d\n", scanStart);
        if (WiFi.scanNetworks(true, true) == WIFI_SCAN_RUNNING)  // nonblock, showhidden
            scanState = SCAN_BSS_RUN;

    } else if (scanState == SCAN_BSS_DONE) {
        // BSS done, transition to BLE scanning
        scanStart = millis();
        DEBUG_MSG("BLE scan start t=%d\n", scanStart);
        if (ble_scan(ZPS_BLE_SCANTIME) == 0)
            scanState = SCAN_BLE_RUN;
    }

    // DEBUG_MSG("ZPSPlugin::runOnce() DONE, scanState=%d\n", scanState);
    if ((scanState == SCAN_BSS_RUN) || (scanState == SCAN_BLE_RUN))  {
        return 1000;  // scan in progress, re-check soon
    }

    // FIXME - NEVER REACHED! (because yeah there's ALWAYS a scan in progress)
    return 5000;
}


uint64_t encodeBSS(uint8_t *bssid, uint8_t chan, uint8_t absRSSI)
{
    uint64_t netBytes = absRSSI & 0xff;
    netBytes <<= 8;
    netBytes |= (chan & 0xff);

    for (uint8_t b = 0; b < 6; b++) {
        netBytes <<= 8;
        netBytes |= bssid[b];
    }

    return netBytes;
}


uint64_t encodeBLE(uint8_t *addr, uint8_t absRSSI)
{
    uint64_t netBytes = absRSSI & 0xff;
    netBytes <<= 8;
    netBytes |= 0xff;  // "channel" byte reserved in BLE records

    for (uint8_t b = 0; b < 6; b++) {
        netBytes <<= 8;
        netBytes |= addr[5-b] & 0xff;
    }

    return netBytes;
}


/**
 * Event handler
 */
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_hs_adv_fields fields;
    int rc;
    int i = 0;

    uint64_t netBytes = 0;

    switch (event->type) {
        case BLE_GAP_EVENT_DISC:
            // called once for every BLE advert received
            rc = ble_hs_adv_parse_fields(&fields, event->disc.data,
                                     event->disc.length_data);
            if (rc != 0)
                return 0;

            if (bleResSize != BLE_NO_RESULTS)
                // as far as we know, we're not in the middle of a BLE scan!
                DEBUG_MSG("Unexpected BLE_GAP_EVENT_DISC!\n");

            //
            // STORE THE RESULTS IN A SORTED LIST
            //

            // !!!FIXME!!! SOME DUPLICATES SURVIVE through filter_duplicates = 1
            // It should be reasonably inexpensive to filter them at this point

            // first, pack each BLE item reading into a 64-bit int
            netBytes = encodeBLE(event->disc.addr.val, abs(event->disc.rssi));
#ifdef ZPS_EXTRAVERBOSE
            // redundant extraverbosity, but I need it for duplicate hunting
            DEBUG_MSG("BL_[%02d]: %08x" "%08x\n", bleCounter, 
                        (uint32_t)(netBytes>>32), (uint32_t)netBytes);
#endif
            // then insert item into a list (up to BLE_MAX_REC records), sorted by RSSI
            for (i = 0; i < bleCounter; i++) {
                // find first element greater than ours, that will be our insertion point
                if (bleResult[i] > netBytes)
                    break;
            }
            // any other records move down one position to vacate res[i]
            for (int j = bleCounter; j > i; j--)
                bleResult[j] = bleResult[j - 1];
            // write new element at insertion point
            bleResult[i] = netBytes;

            // advance tail of list, but not beyond limit
            if (bleCounter < BLE_MAX_REC)
                bleCounter++;

            return 0;  // SUCCESS

        case BLE_GAP_EVENT_DISC_COMPLETE:
            DEBUG_MSG("EVENT_DISC_COMPLETE in %d millis\n", (millis()-scanStart));
            DEBUG_MSG("%d BLE found\n", bleCounter);
            bleResSize = bleCounter;

            bleCounter = 0;  // reset counter
            return 0;  // SUCCESS

        default:
            return 0;  // SUCCESS
    }
}


/**
 * Initiates the GAP general discovery procedure (non-blocking)
 */
static int ble_scan(uint32_t duration, bool passive, bool dedup)
{
    uint8_t own_addr_type;
    struct ble_gap_disc_params disc_params;
    int rc;

    // Figure out address type to use
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        DEBUG_MSG("error determining address type; rc=%d\n", rc);
        return rc;
    }

    // Scanning parameters, these are mostly default
    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    // These two params are the more interesting ones
    disc_params.filter_duplicates = dedup;  // self-explanatory
    disc_params.passive = passive;	// passive uses less power

    // Start scanning process (non-blocking) and return
    rc = ble_gap_disc(own_addr_type, duration, &disc_params,
                      ble_gap_event, NULL);
    if (rc != 0) {
        DEBUG_MSG("error initiating GAP discovery; rc=%d\n", rc);
    }

    return rc;
}
