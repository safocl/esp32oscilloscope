#include <array>
#include <cstdint>
#include <driver/uart.h>
#include <esp_event_cxx.hpp>

#include "connect.hpp"
#include "esp_exception.hpp"
#include "esp_netif_types.h"
#include "esp_wifi_types_generic.h"
#include "esrMeter.hpp"
#include "hal/uart_types.h"
#include "netif.hpp"
#include "soc/clk_tree_defs.h"
#include "wifi.hpp"

#include <asio/ip/address_v4.hpp>

#include <algorithm>
#include <memory>
#include <print>
#include <ranges>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <vector>

using Connect::IpV4;
using Connect::Port;
using Connect::TransferProtocol;
using Connect::Wifi;

extern "C" int app_main() {
    try {
        static idf::event::ESPEventLoop loop {};

        static std::vector< std::unique_ptr< idf::event::ESPEventReg > > eventregs;

        CHECK_THROW( uart_set_baudrate( uart_port_t::UART_NUM_0, 115200 ) );

        constexpr std::string_view ssid     = "esp32osc";
        constexpr std::string_view password = "esp32osc";

        wifi_config_t cfg = { .ap = Wifi::ApConfig(
                              std::as_bytes( std::span( ssid ) ),
                              std::as_bytes( std::span( password ) ),
                              Wifi::ApConfig::CreateInfo { .channel        = 5,
                                                           .authmode       = wifi_auth_mode_t::WIFI_AUTH_WPA2_PSK,
                                                           .max_connection = 5,
                                                           .pmf_cfg        = { .capable = {}, .required = true }

                              } ) };

        static core::NetIfHandler wifiIfHandler =
        Wifi::createDefaultWithHandler< Connect::ApProvider >( cfg, Wifi::Storage::eRam );

        if ( !bool( wifiIfHandler.getFlags() & ESP_NETIF_DHCP_SERVER ) )
            throw std::runtime_error( "DHCP flag not selected.!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" );

        Wifi::start();
        Wifi::setMaxTxPower( Wifi::TxPower( 8 ) );

        esp_netif_ip_info_t ipInfo { .ip      = IpV4( "192.168.88.1" ),
                                     .netmask = IpV4( "255.255.255.0" ),
                                     .gw      = IpV4( "192.168.88.1" ) };

        std::println( "Setted IP: {}", to_string( IpV4( ipInfo.ip ) ) );

        wifiIfHandler.dhcpsStop();
        wifiIfHandler.setIpInfo( ipInfo );
        wifiIfHandler.dhcpsStart();

        const auto currentIpInfo = wifiIfHandler.getIpInfo();
        std::println( "Current IP: {}", to_string( IpV4( currentIpInfo.ip ) ) );

        const auto oldIpInfo = wifiIfHandler.getOldIpInfo();
        std::println( "Old IP: {}", to_string( IpV4( oldIpInfo.ip ) ) );

        while ( wifiIfHandler.dhcpsGetStatus() != ESP_NETIF_DHCP_STARTED ) {
            std::print( "WAIT FOR DHCPS STARTED !!!\n" );
            using namespace std::chrono_literals;
            std::this_thread::sleep_for( 1000ms );
        }

        static Esr::EsrMeter osc1;

        eventregs.push_back( loop.register_event(
        idf::event::ESPEvent( IP_EVENT, idf::event::ESPEventID( IP_EVENT_ASSIGNED_IP_TO_CLIENT ) ),
        []( const idf::event::ESPEvent &, void * params ) {
            try {
                auto evt = static_cast< ip_event_assigned_ip_to_client_t * >( params );

                const auto     remoteIp   = IpV4( evt->ip );
                constexpr auto remotePort = Port( 8881 );

                constexpr auto localIp   = IpV4( "127.0.0.1" );
                constexpr auto localPort = Port( 8882 );

                osc1.stop( true );
                osc1.setTransferProtocol( TransferProtocol::create( remoteIp, remotePort, localIp, localPort ) );
                osc1.start();
                std::println( "-------Endpoint: {}:{} added.", to_string( remoteIp ), to_string( remotePort ) );

            } catch ( const std::exception & e ) { std::println( "-------Endpoint event handler: {}", e.what() ); }
        } ) );

    } catch ( const std::exception & e ) { std::println( "MAIN EXCEPTION: {}", e.what() ); }

    if constexpr ( false ) {
        using namespace std::chrono_literals;
        while ( true ) {
            std::println( "------endless while" );
            std::this_thread::sleep_for( 1000ms );
        }
    }

    return 0;
}
