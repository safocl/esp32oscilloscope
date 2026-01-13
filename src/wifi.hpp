#pragma once

#include <concepts>
#include <cstdint>
#include <memory>
#include <type_traits>

#include <esp_exception.hpp>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <esp_wifi_default.h>
#include <esp_event_cxx.hpp>
#include <esp_wifi_types.h>
#include <utility>

#include "esp_netif_types.h"
#include "nvsFlash.hpp"
#include "netif.hpp"

namespace Connect {

class Wifi final {
    struct NetIfDefaultWifiDeleter final {
        void operator()( esp_netif_t * ptr ) const noexcept { esp_netif_destroy_default_wifi( ptr ); }
    };

public:
    enum class WifiMode : std::underlying_type_t< wifi_mode_t > {
        eNull  = WIFI_MODE_NULL, /**< null mode */
        eSta   = WIFI_MODE_STA, /**< WiFi station mode */
        eAp    = WIFI_MODE_AP, /**< WiFi soft-AP mode */
        eApsta = WIFI_MODE_APSTA, /**< WiFi station + soft-AP mode */
        eNan   = WIFI_MODE_NAN, /**< WiFi NAN mode */
        eMax   = WIFI_MODE_MAX
    };

    enum class Storage : std::underlying_type_t< wifi_storage_t > {
        eFlash = WIFI_STORAGE_FLASH,
        eRam   = WIFI_STORAGE_RAM
    };

    enum class Interface : std::underlying_type_t< wifi_interface_t > {
        eSta = WIFI_IF_STA,
        eAp  = WIFI_IF_AP,
#if defined( CONFIG_IDF_TARGET_ESP32 ) || defined( CONFIG_IDF_TARGET_ESP32S2 )
        eNan = WIFI_IF_NAN,
#endif
        eMax = WIFI_IF_MAX
    };

    enum class TxPower : std::int8_t {};

    static void init() {
        if ( isInited )
            return;

        core::NvsFlash::init();
        core::NetIf::init();

        const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        CHECK_THROW( esp_wifi_init( &cfg ) );

        isInited = true;
    }

    static void init( const wifi_init_config_t & cfg ) {
        if ( isInited )
            return;

        core::NvsFlash::init();
        core::NetIf::init();

        CHECK_THROW( esp_wifi_init( &cfg ) );

        isInited = true;
    }

    static void deinit() noexcept {
        ESP_ERROR_CHECK( esp_wifi_deinit() );
        isInited = false;
    }

    static void start() { CHECK_THROW( esp_wifi_start() ); }
    static void stop() { CHECK_THROW( esp_wifi_stop() ); }

    template < class DefaultProvider >
        requires requires {
            { DefaultProvider::init() } -> std::same_as< esp_netif_t * >;
            requires std::same_as< const WifiMode, decltype( DefaultProvider::wifimode ) >;
            requires std::same_as< const Interface, decltype( DefaultProvider::interface ) >;
        }
    static void createDefault( wifi_config_t & cfg, Storage storage ) {
        if ( isInited )
            deinit();

        core::NvsFlash::init();
        core::NetIf::init();

        DefaultProvider::init();

        init();
        setMode( DefaultProvider::wifimode );
        setConfig( DefaultProvider::interface, cfg );
        setStorage( storage );
    }

    template < class DefaultProvider >
        requires requires {
            { DefaultProvider::init() } -> std::same_as< esp_netif_t * >;
            requires std::same_as< const WifiMode, decltype( DefaultProvider::wifimode ) >;
            requires std::same_as< const Interface, decltype( DefaultProvider::interface ) >;
        }
    [[nodiscard]] static core::NetIfHandler createDefaultWithHandler( wifi_config_t & cfg, Storage storage ) {
        if ( isInited )
            deinit();

        core::NvsFlash::init();
        core::NetIf::init();

        core::NetIfHandler res = core::NetIf::createHandler( DefaultProvider::init(), NetIfDefaultWifiDeleter() );

        init();
        setMode( DefaultProvider::wifimode );
        setConfig( DefaultProvider::interface, cfg );
        setStorage( storage );

        return res;
    }

    static void connect() {}
    static void disconnect() {}

    static void setConfig( Interface interface, wifi_config_t & cfg ) {
        CHECK_THROW( esp_wifi_set_config( static_cast< wifi_interface_t >( interface ), &cfg ) );
    }

    static void setMode( WifiMode mode ) { CHECK_THROW( esp_wifi_set_mode( static_cast< wifi_mode_t >( mode ) ) ); }

    static WifiMode getMode() {
        wifi_mode_t mode;
        CHECK_THROW( esp_wifi_get_mode( &mode ) );
        return static_cast< WifiMode >( mode );
    }

    static void setStorage( Storage storage ) {
        CHECK_THROW( esp_wifi_set_storage( static_cast< wifi_storage_t >( storage ) ) );
    }

    static void setMaxTxPower( TxPower power ) {
        CHECK_THROW( esp_wifi_set_max_tx_power( std::to_underlying( power ) ) );
    }
    static TxPower getMaxTxPower() {
        std::int8_t v;
        CHECK_THROW( esp_wifi_get_max_tx_power( &v ) );
        return TxPower( v );
    }

private:
    inline static bool isInited = false;
};

struct ApProvider final {
    static esp_netif_t *             init() { return esp_netif_create_default_wifi_ap(); }
    static constexpr Wifi::WifiMode  wifimode { Wifi::WifiMode::eAp };
    static constexpr Wifi::Interface interface { Wifi::Interface::eAp };
};

struct StaProvider final {
    static esp_netif_t *             init() { return esp_netif_create_default_wifi_sta(); }
    static constexpr Wifi::WifiMode  wifimode { Wifi::WifiMode::eSta };
    static constexpr Wifi::Interface interface { Wifi::Interface::eSta };
};

struct NanProvider final {
    static esp_netif_t *             init() { return esp_netif_create_default_wifi_nan(); }
    static constexpr Wifi::WifiMode  wifimode { Wifi::WifiMode::eNan };
    static constexpr Wifi::Interface interface { Wifi::Interface::eNan };
};
}   // namespace Connect
