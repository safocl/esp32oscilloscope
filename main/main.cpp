#include <array>
#include <cstdint>
#include <driver/uart.h>
#include <esp_event_cxx.hpp>

#include <driver/dac_cosine.h>
#include <esp_adc/adc_continuous.h>
#include <soc/soc_caps.h>
#include "connect.hpp"
#include "esp_exception.hpp"
#include "esp_netif_types.h"
#include "esp_wifi_types_generic.h"
#include "hal/uart_types.h"
#include "netif.hpp"
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

std::jthread                        mTransmitterThread {};
std::shared_ptr< TransferProtocol > mNetProto;

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

        eventregs.push_back( loop.register_event(
        idf::event::ESPEvent( IP_EVENT, idf::event::ESPEventID( IP_EVENT_ASSIGNED_IP_TO_CLIENT ) ),
        []( const idf::event::ESPEvent &, void * params ) {
            try {
                auto evt = static_cast< ip_event_assigned_ip_to_client_t * >( params );

                const auto     remoteIp   = IpV4( evt->ip );
                constexpr auto remotePort = Port( 8881 );

                constexpr auto localIp   = IpV4( "127.0.0.1" );
                constexpr auto localPort = Port( 8882 );

                mNetProto = TransferProtocol::create( remoteIp, remotePort, localIp, localPort );
                std::println( "-------Endpoint: {}:{} added.", to_string( remoteIp ), to_string( remotePort ) );

                mTransmitterThread = std::jthread( []( std::stop_token token ) noexcept {
                    auto transmitter = mNetProto;

                    while ( !token.stop_requested() ) {
                        try {
                            using namespace std::chrono_literals;
                            while ( !token.stop_requested() && !transmitter->hasConnection() )
                                std::this_thread::sleep_for( 1s );

                            constexpr std::uint32_t samplingRate = 30000;
                            constexpr std::uint32_t dacFreq      = 300;

                            std::println( "Main measure start." );
                            dac_cosine_handle_t h;
                            dac_cosine_config_t conf = { DAC_CHAN_0,
                                                         dacFreq,
                                                         DAC_COSINE_CLK_SRC_DEFAULT,
                                                         DAC_COSINE_ATTEN_DB_6,
                                                         DAC_COSINE_PHASE_0,
                                                         0,
                                                         { 0 } };
                            dac_cosine_new_channel( &conf, &h );
                            dac_cosine_start( h );

                            adc_unit_t    unit1;
                            adc_channel_t channel1;
                            ESP_ERROR_CHECK( adc_continuous_io_to_channel( 39, &unit1, &channel1 ) );

                            adc_unit_t    unit2;
                            adc_channel_t channel2;
                            ESP_ERROR_CHECK( adc_continuous_io_to_channel( 36, &unit2, &channel2 ) );

                            adc_digi_pattern_config_t patterns[] {
                                { adc_atten_t::ADC_ATTEN_DB_12,
                                  static_cast< std::uint8_t >( channel1 ),
                                  static_cast< std::uint8_t >( unit1 ),
                                  adc_bitwidth_t::ADC_BITWIDTH_12 },
#if 1
                                { adc_atten_t::ADC_ATTEN_DB_12,
                                  static_cast< std::uint8_t >( channel2 ),
                                  static_cast< std::uint8_t >( unit2 ),
                                  adc_bitwidth_t::ADC_BITWIDTH_12 },
#endif
                            };

                            constexpr int samplesCount = 128;

                            adc_continuous_handle_t     handle {};
                            adc_continuous_handle_cfg_t adc_config = {
                                .max_store_buf_size =
                                samplesCount * SOC_ADC_DIGI_RESULT_BYTES * std::ranges::size( patterns ),
                                .conv_frame_size =
                                samplesCount * SOC_ADC_DIGI_RESULT_BYTES * std::ranges::size( patterns ),
                                .flags = {},
                            };
                            ESP_ERROR_CHECK( adc_continuous_new_handle( &adc_config, &handle ) );

                            adc_continuous_config_t config { .pattern_num    = std::ranges::size( patterns ),
                                                             .adc_pattern    = patterns,
                                                             .sample_freq_hz = samplingRate,
                                                             .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
                                                             .format         = {} };

                            ESP_ERROR_CHECK( adc_continuous_config( handle, &config ) );
                            ESP_ERROR_CHECK( adc_continuous_start( handle ) );

                            std::this_thread::sleep_for( 3s );

                            std::array< adc_continuous_data_t, samplesCount * std::ranges::size( patterns ) > mData {};

                            std::array< std::int16_t, samplesCount > mValues {};

                            std::uint32_t samplesRead;
                            ESP_ERROR_CHECK(
                            adc_continuous_read_parse( handle, mData.data(), mData.size(), &samplesRead, 10000 ) );
                            assert( samplesRead == mData.size() );

                            ESP_ERROR_CHECK( adc_continuous_stop( handle ) );
                            ESP_ERROR_CHECK( dac_cosine_stop( h ) );
                            ESP_ERROR_CHECK( dac_cosine_del_channel( h ) );
                            ESP_ERROR_CHECK( adc_continuous_deinit( handle ) );

                            std::println( "Main measure stop." );

                            auto toIt = mValues.begin();
                            for ( int i = 0; i < mData.size(); ++i )
                                if ( mData.at( i ).valid && mData.at( i ).channel == channel1 ) {
                                    *toIt = static_cast< std::int16_t >( mData.at( i ).raw_data );
                                    toIt++;
                                }

                            std::span values( mValues );
                            std::println( "------mValues size: {}", mValues.size() );

                            static constexpr std::array< char, 64 > magic { "EsrMeter 42 42 i tak soydet!!!" };

                            transmitter->write( std::as_bytes( std::span( magic ) ) );
                            transmitter->write( std::as_bytes( values ) );
                        } catch ( const std::bad_alloc & e ) {
                            std::println( "------Sending error: {}", e.what() );
                            transmitter->waitForDone();
                        } catch ( const std::exception & e ) { std::print( "------Sending error: {}\n", e.what() ); }
                        // using namespace std::chrono_literals;
                        // std::this_thread::sleep_for( 1s );
                    }
                } );

            } catch ( const std::exception & e ) { std::println( "-------Endpoint event handler: {}", e.what() ); }
        } ) );

    } catch ( const std::exception & e ) { std::println( "MAIN EXCEPTION: {}", e.what() ); }

    return 0;
}
