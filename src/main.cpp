#include <esp_netif.h>
#include <esp_netif_ip_addr.h>
#include <esp_netif_types.h>
#include <esp_event_cxx.hpp>
#include <driver/uart.h>

#include "connect.hpp"
#include "esp_exception.hpp"
#include "hal/uart_types.h"
#include "netif.hpp"
#include "oscilloscopeLib.hpp"
#include "wifi.hpp"

#include <asio/ip/address_v4.hpp>

#include <algorithm>
#include <memory>
#include <print>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

using Connect::Wifi;
using Connect::TransferProtocol;
using Connect::IpV4;
using Connect::Port;

extern "C" int app_main() {
    try {
        static idf::event::ESPEventLoop loop {};

        static std::vector< std::unique_ptr< idf::event::ESPEventReg > > eventregs;

        CHECK_THROW( uart_set_baudrate( uart_port_t::UART_NUM_0, 115200 ) );

        wifi_pmf_config_t pfm {};
        pfm.required = true;

        static constexpr std::string_view ssid( "esp32osc" );
        static constexpr std::string_view passwd( "esp32osc" );

        wifi_ap_config_t apConf {
            .channel = 5, .authmode = wifi_auth_mode_t::WIFI_AUTH_WPA2_PSK, .max_connection = 5, .pmf_cfg = pfm
        };
        std::ranges::copy( ssid, apConf.ssid );
        std::ranges::copy( passwd, apConf.password );

        wifi_config_t cfg = { .ap = apConf };

        static core::NetIfHandler wifiIfHandler =
        Wifi::createDefaultWithHandler< Connect::ApProvider >( cfg, Wifi::Storage::eRam );

        if ( !bool( wifiIfHandler.getFlags() & ESP_NETIF_DHCP_SERVER ) )
            throw std::runtime_error( "DHCP flag not selected.!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" );

        Wifi::start();
        Wifi::setMaxTxPower( 8 );

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

        static Osc::Oscilloscope osc1;

        eventregs.push_back( loop.register_event(
        idf::event::ESPEvent( IP_EVENT, idf::event::ESPEventID( IP_EVENT_AP_STAIPASSIGNED ) ),
        []( const idf::event::ESPEvent &, void * params ) {
            try {
                auto evt = static_cast< ip_event_ap_staipassigned_t * >( params );

                const auto     remoteIp   = IpV4( evt->ip );
                constexpr auto remotePort = Port( 8881 );

                constexpr auto localIp   = IpV4( "127.0.0.1" );
                constexpr auto localPort = Port( 8882 );

                osc1.stop( true );
                osc1.setTransferProtocol( TransferProtocol::create( remoteIp, remotePort, localIp, localPort ) );
                osc1.start();
                std::print( "-------Endpoint: {}:{} added.\n", to_string( remoteIp ), to_string( remotePort ) );

            } catch ( const std::exception & e ) { std::print( "-------Endpoint event handler: {}\n", e.what() ); }
        } ) );

    } catch ( const std::exception & e ) { std::print( "MAIN EXCEPTION: {}\n", e.what() ); }

#if 0
    using namespace std::chrono_literals;
    while ( true ) {
        std::print( "------endless while \n" );
        std::this_thread::sleep_for( 1000ms );
    }
#endif

    return 0;
}
