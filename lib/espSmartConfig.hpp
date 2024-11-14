#pragma once

#include <iostream>
#include <cstring>
#include <utility>
#include <array>
#include <vector>
#include <cstdint>
#include <span>
#include <memory>

#include <esp_event_cxx.hpp>
#include <esp_exception.hpp>
#include <nvs_flash.h>
#include <esp_err.h>
#include <esp_netif.h>
#include <esp_netif_types.h>
#include <esp_smartconfig.h>
#include <esp_wifi.h>
#include <esp_smartconfig.h>

namespace Connect {

class WifiSmartConfig final {
public:
    enum class ConnectType : std::int8_t {
        eEsptouch         = SC_TYPE_ESPTOUCH, /**< protocol: ESPTouch */
        eAirkiss          = SC_TYPE_AIRKISS, /**< protocol: AirKiss */
        eEsptouch_airkiss = SC_TYPE_ESPTOUCH_AIRKISS, /**< protocol: ESPTouch and AirKiss */
        eEsptouch_v2      = SC_TYPE_ESPTOUCH_V2, /**< protocol: ESPTouch v2*/
    };
    using enum ConnectType;

    enum class EventType : std::int8_t {
        eScan_done     = SC_EVENT_SCAN_DONE, /*!< Station smartconfig has finished to scan for APs */
        eFound_channel = SC_EVENT_FOUND_CHANNEL, /*!< Station smartconfig has found the channel of the target AP */
        eGot_ssid_pswd = SC_EVENT_GOT_SSID_PSWD, /*!< Station smartconfig got the SSID and password */
        eSend_ack_done = SC_EVENT_SEND_ACK_DONE, /*!< Station smartconfig has sent ACK to cellphone */
    };
    using enum EventType;

    enum class Status : std::uint8_t { eStoped, eStarted, eConnected };
    using enum Status;

    WifiSmartConfig() {
        auto err = nvs_flash_init();
        if ( err != ESP_OK )
            throw idf::ESPException( err );

        err = esp_netif_init();
        if ( err != ESP_OK )
            throw idf::ESPException( err );

        using namespace idf::event;

        ESPEventLoop loop;

        esp_netif_t * sta_netif = esp_netif_create_default_wifi_sta();
        assert( sta_netif );

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        err                    = esp_wifi_init( &cfg );
        if ( err != ESP_OK )
            throw idf::ESPException( err );

        mEvents.push_back( loop.register_event( ESPEvent( WIFI_EVENT, ESPEventID( WIFI_EVENT_STA_START ) ),
                                                [ this ]( const ESPEvent & event, void * params ) { begin(); } ) );

        mEvents.push_back( loop.register_event( ESPEvent( WIFI_EVENT, ESPEventID( WIFI_EVENT_STA_DISCONNECTED ) ),
                                                []( const ESPEvent & event, void * params ) { esp_wifi_connect(); } ) );

        mEvents.push_back(
        loop.register_event( ESPEvent( IP_EVENT, ESPEventID( IP_EVENT_STA_GOT_IP ) ),
                             []( const ESPEvent & event, void * params ) { std::cout << "WiFi Connected to ap\n"; } ) );

        mEvents.push_back( loop.register_event(
        ESPEvent( SC_EVENT, ESPEventID( SC_EVENT_GOT_SSID_PSWD ) ), [ this ]( const ESPEvent & event, void * params ) {
            std::cout << "Got SSID and password\n";

            auto evt = reinterpret_cast< const smartconfig_event_got_ssid_pswd_t * >( params );

            wifi_config_t wifi_config;
            bzero( &wifi_config, sizeof( wifi_config_t ) );
            memcpy( wifi_config.sta.ssid, evt->ssid, sizeof( wifi_config.sta.ssid ) );
            memcpy( wifi_config.sta.password, evt->password, sizeof( wifi_config.sta.password ) );

#ifdef CONFIG_SET_MAC_ADDRESS_OF_TARGET_AP
            wifi_config.sta.bssid_set = evt->bssid_set;
            if ( wifi_config.sta.bssid_set == true ) {
                std::cout << "Set MAC address of target AP: " << MAC2STR( evt->bssid ) << std::endl;
                memcpy( wifi_config.sta.bssid, evt->bssid, sizeof( wifi_config.sta.bssid ) );
            }
#endif

            ESP_ERROR_CHECK( esp_wifi_disconnect() );
            ESP_ERROR_CHECK( esp_wifi_set_config( WIFI_IF_STA, &wifi_config ) );
            ESP_ERROR_CHECK( esp_wifi_connect() );

            mStatus = eConnected;
            std::copy( evt->cellphone_ip, evt->cellphone_ip + 4, mApIp.begin() );
            end();
        } ) );

        ESP_ERROR_CHECK( esp_wifi_set_mode( WIFI_MODE_STA ) );
        ESP_ERROR_CHECK( esp_wifi_start() );
    }
    ~WifiSmartConfig() {}

    Status getStatus() const noexcept { return mStatus; }

    void begin( std::array< char, 16 > cryptKey ) {}
    void begin( ConnectType type = eEsptouch ) {
        auto err = esp_smartconfig_set_type( static_cast< smartconfig_type_t >( type ) );
        if ( err != ESP_OK )
            throw idf::ESPException( err );

        smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
        err                            = esp_smartconfig_start( &cfg );
        if ( err != ESP_OK )
            throw idf::ESPException( err );

        mStatus = eStarted;
    }

    void end() {
        auto err = esp_smartconfig_stop();
        if ( err != ESP_OK )
            throw idf::ESPException( err );
        mStatus = eStoped;
    }

    void start( std::array< char, 16 > cryptKey ) { begin( cryptKey ); }
    void start( ConnectType type = eEsptouch ) { begin( type ); }
    void stop() { end(); }

    std::array< std::uint8_t, 4 > getApIp() const noexcept;

private:
    Status mStatus { eStoped };
    alignas( std::uint32_t ) std::array< std::uint8_t, 4 > mApIp {};

    std::vector< std::unique_ptr< idf::event::ESPEventReg > > mEvents;
};
