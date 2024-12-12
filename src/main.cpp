#include <esp_netif.h>
#include <esp_netif_ip_addr.h>
#include <esp_netif_types.h>
#include <esp_event_cxx.hpp>

#include "connect.hpp"
#include "netif.hpp"
#include "oscilloscopeLib.hpp"
#include "wifi.hpp"

#include <asio/ip/address_v4.hpp>

#include <algorithm>
#include <memory>
#include <print>
#include <stdexcept>
#include <string_view>
#include <vector>

using Connect::Wifi;
using Connect::TransferProtocol;

extern "C" int app_main() {
    try {
        static idf::event::ESPEventLoop loop {};

        static std::vector< std::unique_ptr< idf::event::ESPEventReg > > eventregs;

        wifi_pmf_config_t pfm {};
        pfm.required = true;

        constexpr std::string_view ssid( "esp32osc" );
        constexpr std::string_view passwd( "esp32osc" );

        wifi_ap_config_t apConf {};
        std::ranges::copy( ssid, &apConf.ssid[ 0 ] );
        std::ranges::copy( passwd, &apConf.password[ 0 ] );
        apConf.channel        = 5;
        apConf.authmode       = wifi_auth_mode_t::WIFI_AUTH_WPA2_PSK;
        apConf.max_connection = 5;
        apConf.pmf_cfg        = pfm;

        wifi_config_t             cfg = { .ap = apConf };
        static core::NetIfHandler wifiIfHandler =
        Wifi::createDefaultWithHandler< Connect::ApProvider >( cfg, Wifi::Storage::eRam );

        if ( !bool( wifiIfHandler.getFlags() & ESP_NETIF_DHCP_SERVER ) )
            throw std::runtime_error( "DHCP flag not selected.!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" );

        eventregs.push_back(
        loop.register_event( idf::event::ESPEvent( IP_EVENT, idf::event::ESPEventID( ESP_NETIF_IP_EVENT_GOT_IP ) ),
                             []( const idf::event::ESPEvent &, void * params ) {
                                 auto evt = reinterpret_cast< ip_event_got_ip_t * >( params );
                                 std::print( "-------Got IP: {}.{}.{}.{}\n",
                                             esp_ip4_addr1_16( &evt->ip_info.ip ),
                                             esp_ip4_addr2_16( &evt->ip_info.ip ),
                                             esp_ip4_addr3_16( &evt->ip_info.ip ),
                                             esp_ip4_addr4_16( &evt->ip_info.ip ) );
                             } ) );

        Wifi::start();
        Wifi::setMaxTxPower( 8 );

        esp_netif_ip_info_t ipInfo {};
        IP4_ADDR( &ipInfo.ip, 192, 168, 88, 1 );
        IP4_ADDR( &ipInfo.netmask, 255, 255, 255, 0 );
        IP4_ADDR( &ipInfo.gw, 192, 168, 88, 1 );

        std::print( "Setted IP: {}.{}.{}.{}\n",
                    esp_ip4_addr1_16( &ipInfo.ip ),
                    esp_ip4_addr2_16( &ipInfo.ip ),
                    esp_ip4_addr3_16( &ipInfo.ip ),
                    esp_ip4_addr4_16( &ipInfo.ip ) );

        wifiIfHandler.dhcpsStop();
        wifiIfHandler.setIpInfo( ipInfo );
        wifiIfHandler.dhcpsStart();

        const auto currentIpInfo = wifiIfHandler.getIpInfo();
        std::print( "Current IP: {}.{}.{}.{}\n",
                    esp_ip4_addr1_16( &currentIpInfo.ip ),
                    esp_ip4_addr2_16( &currentIpInfo.ip ),
                    esp_ip4_addr3_16( &currentIpInfo.ip ),
                    esp_ip4_addr4_16( &currentIpInfo.ip ) );

        const auto oldIpInfo = wifiIfHandler.getOldIpInfo();
        std::print( "Old IP: {}.{}.{}.{}\n",
                    esp_ip4_addr1_16( &oldIpInfo.ip ),
                    esp_ip4_addr2_16( &oldIpInfo.ip ),
                    esp_ip4_addr3_16( &oldIpInfo.ip ),
                    esp_ip4_addr4_16( &oldIpInfo.ip ) );

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
                auto       evt = reinterpret_cast< ip_event_ap_staipassigned_t * >( params );
                const auto ip  = evt->ip;

                asio::ip::tcp::endpoint endpoint {
                    asio::ip::make_address_v4( asio::ip::address_v4::bytes_type {
                    esp_ip4_addr1( &ip ), esp_ip4_addr2( &ip ), esp_ip4_addr3( &ip ), esp_ip4_addr4( &ip ) } ),
                    8881
                };

                osc1.setTransferProtocol( TransferProtocol::create( endpoint ) );

                // osc1.stop();
                osc1.start();
                std::print( "-------Endpoint: {}:{} added.\n", endpoint.address().to_string(), endpoint.port() );

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
