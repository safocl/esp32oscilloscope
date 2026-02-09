#pragma once

#include "esp_netif_types.h"
#include "nvsFlash.hpp"
#include "netif.hpp"

#include <esp_exception.hpp>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <esp_wifi_default.h>
#include <esp_event_cxx.hpp>
#include <esp_wifi_types.h>

#include <utility>
#include <algorithm>
#include <ranges>
#include <concepts>
#include <cstdint>
#include <memory>
#include <type_traits>

namespace Connect {

class Wifi final {
    struct NetIfDefaultWifiDeleter final {
        void operator()( esp_netif_t * ptr ) const noexcept { esp_netif_destroy_default_wifi( ptr ); }
    };

public:
    struct StaConfig : wifi_sta_config_t {
        struct CreateInfo final {
            wifi_scan_method_t    scan_method {};
            bool                  bssid_set {};
            uint8_t               channel {};
            uint16_t              listen_interval {};
            wifi_sort_method_t    sort_method {};
            wifi_scan_threshold_t threshold {};
            wifi_pmf_config_t     pmf_cfg {};
            uint32_t              rm_enabled {};
            uint32_t              btm_enabled {};
            uint32_t              mbo_enabled {};
            uint32_t              ft_enabled {};
            uint32_t              owe_enabled {};
            uint32_t              transition_disable {};
            uint32_t              disable_wpa3_compatible_mode {};
            uint32_t              reserved1 {};
            wifi_sae_pwe_method_t sae_pwe_h2e {};
            wifi_sae_pk_mode_t    sae_pk_mode {};
            uint8_t               failure_retry_cnt {};
            uint32_t              he_dcm_set {};
            uint32_t              he_dcm_max_constellation_tx {};
            uint32_t              he_dcm_max_constellation_rx {};
            uint32_t              he_mcs9_enabled {};
            uint32_t              he_su_beamformee_disabled {};
            uint32_t              he_trig_su_bmforming_feedback_disabled {};
            uint32_t              he_trig_mu_bmforming_partial_feedback_disabled {};
            uint32_t              he_trig_cqi_feedback_disabled {};
            uint32_t              vht_su_beamformee_disabled {};
            uint32_t              vht_mu_beamformee_disabled {};
            uint32_t              vht_mcs8_enabled {};
        };

        constexpr StaConfig( std::span< const std::byte > ssid,
                             std::span< const std::byte > password,
                             std::span< const std::byte > bssid,
                             std::span< const std::byte > sae_h2e_identifier,
                             CreateInfo &&                ci ) :
        wifi_sta_config_t( {},
                           {},
                           ci.scan_method,
                           ci.bssid_set,
                           {},
                           ci.channel,
                           ci.listen_interval,
                           ci.sort_method,
                           ci.threshold,
                           ci.pmf_cfg,
                           ci.rm_enabled,
                           ci.btm_enabled,
                           ci.mbo_enabled,
                           ci.ft_enabled,
                           ci.owe_enabled,
                           ci.transition_disable,
                           ci.disable_wpa3_compatible_mode,
                           ci.reserved1,
                           ci.sae_pwe_h2e,
                           ci.sae_pk_mode,
                           ci.failure_retry_cnt,
                           ci.he_dcm_set,
                           ci.he_dcm_max_constellation_tx,
                           ci.he_dcm_max_constellation_rx,
                           ci.he_mcs9_enabled,
                           ci.he_su_beamformee_disabled,
                           ci.he_trig_su_bmforming_feedback_disabled,
                           ci.he_trig_mu_bmforming_partial_feedback_disabled,
                           ci.he_trig_cqi_feedback_disabled,
                           ci.vht_su_beamformee_disabled,
                           ci.vht_mu_beamformee_disabled,
                           ci.vht_mcs8_enabled,
                           {},
                           {} ) {
            const std::array fromRanges {
                reinterpret_cast< const std::uint8_t * >( ssid.data() ),
                reinterpret_cast< const std::uint8_t * >( password.data() ),
                reinterpret_cast< const std::uint8_t * >( bssid.data() ),
                reinterpret_cast< const std::uint8_t * >( sae_h2e_identifier.data() ),
            };

            const std::array toRanges { this->ssid, this->password, this->bssid, this->sae_h2e_identifier };

            const std::array sizes { std::min( ssid.size(), std::ranges::size( this->ssid ) ),
                                     std::min( password.size(), std::ranges::size( this->password ) ),
                                     std::min( bssid.size(), std::ranges::size( this->bssid ) ),
                                     std::min( sae_h2e_identifier.size(),
                                               std::ranges::size( this->sae_h2e_identifier ) ) };

            for ( auto [ in, out, size ] : std::views::zip( fromRanges, toRanges, sizes ) )
                std::ranges::copy_n( in, size, out );
        }
    };

    struct ApConfig : wifi_ap_config_t {
        struct CreateInfo final {
            uint8_t                    ssid_len             = {};
            uint8_t                    channel              = {};
            wifi_auth_mode_t           authmode             = {};
            uint8_t                    ssid_hidden          = {};
            uint8_t                    max_connection       = {};
            uint16_t                   beacon_interval      = {};
            uint8_t                    csa_count            = {};
            uint8_t                    dtim_period          = {};
            wifi_cipher_type_t         pairwise_cipher      = {};
            bool                       ftm_responder        = {};
            wifi_pmf_config_t          pmf_cfg              = {};
            wifi_sae_pwe_method_t      sae_pwe_h2e          = {};
            uint8_t                    transition_disable   = {};
            uint8_t                    sae_ext              = {};
            uint8_t                    wpa3_compatible_mode = {};
            wifi_bss_max_idle_config_t bss_max_idle_cfg     = {};
            uint16_t                   gtk_rekey_interval   = {};
        };

        constexpr ApConfig( std::span< const std::byte > ssid,
                            std::span< const std::byte > password,
                            CreateInfo &&                ci ) :
        wifi_ap_config_t( {},
                          {},
                          ci.ssid_len,
                          ci.channel,
                          ci.authmode,
                          ci.ssid_hidden,
                          ci.max_connection,
                          ci.beacon_interval,
                          ci.csa_count,
                          ci.dtim_period,
                          ci.pairwise_cipher,
                          ci.ftm_responder,
                          ci.pmf_cfg,
                          ci.sae_pwe_h2e,
                          ci.transition_disable,
                          ci.sae_ext,
                          ci.wpa3_compatible_mode,
                          {},
                          ci.bss_max_idle_cfg,
                          ci.gtk_rekey_interval ) {
            assert( ssid.size() <= std::ranges::size( this->ssid ) );
            assert( password.size() <= std::ranges::size( this->password ) );

            std::ranges::copy(
            ssid | std::views::transform( []( auto el ) { return static_cast< std::uint8_t >( el ); } ), this->ssid );

            std::ranges::copy( password |
                               std::views::transform( []( auto el ) { return static_cast< std::uint8_t >( el ); } ),
                               this->password );
        }
    };

    struct NanConfig : wifi_nan_sync_config_t {
        struct CreateInfo final {
            uint8_t  op_channel  = {}; /**< NAN Discovery operating channel */
            uint8_t  master_pref = {}; /**< Device's preference value to serve as NAN Master */
            uint8_t  scan_time   = {}; /**< Scan time in seconds while searching for a NAN cluster */
            uint16_t warm_up_sec = {}; /**< Warm up time before assuming NAN Anchor Master role */
        };

        constexpr NanConfig( CreateInfo && ci ) :
        wifi_nan_sync_config_t( ci.op_channel, ci.master_pref, ci.scan_time, ci.warm_up_sec ) {}
    };

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
