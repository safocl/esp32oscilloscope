#pragma once

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "gpio_cxx.hpp"
#include "hal/adc_types.h"
#include <chrono>
#include <concepts>
#include <cstddef>
#include <esp_exception.hpp>
#include <cstdint>

#include <esp_adc/adc_continuous.h>
#include <esp_adc/adc_oneshot.h>
#include <functional>
#include <memory>
#include <span>
#include <type_traits>
#include <utility>

namespace core::Periph {

class Adc final {
public:
    struct Caps final {
        /*-------------------------- ADC CAPS ----------------------------------------*/
        /*!< SAR ADC Module*/
        static constexpr bool rtcCtrlSupported = SOC_ADC_RTC_CTRL_SUPPORTED;
        static constexpr bool digCtrlSupported = SOC_ADC_DIG_CTRL_SUPPORTED;
        static constexpr bool dmaSupported     = SOC_ADC_DMA_SUPPORTED;
        static constexpr auto periphNum        = SOC_ADC_PERIPH_NUM;
        static constexpr auto maxChannelNum    = SOC_ADC_MAX_CHANNEL_NUM;
        static constexpr auto attenNum         = SOC_ADC_ATTEN_NUM;

        /*!< Digital */
        static constexpr auto digiControllerNum    = SOC_ADC_DIGI_CONTROLLER_NUM;
        static constexpr auto pattLenMax           = SOC_ADC_PATT_LEN_MAX;
        static constexpr auto digiMinBitwidth      = SOC_ADC_DIGI_MIN_BITWIDTH;
        static constexpr auto digiMaxBitwidth      = SOC_ADC_DIGI_MAX_BITWIDTH;
        static constexpr auto digiResultBytes      = SOC_ADC_DIGI_RESULT_BYTES;
        static constexpr auto digiDataBytesPerConv = SOC_ADC_DIGI_DATA_BYTES_PER_CONV;
        static constexpr auto digiMonitorNum       = SOC_ADC_DIGI_MONITOR_NUM;
        static constexpr auto sampleFreqThresHigh  = SOC_ADC_SAMPLE_FREQ_THRES_HIGH;
        static constexpr auto sampleFreqThresLow   = SOC_ADC_SAMPLE_FREQ_THRES_LOW;

        /*!< RTC */
        static constexpr auto rtcMinBitwidth = SOC_ADC_RTC_MIN_BITWIDTH;
        static constexpr auto rtcMaxBitwidth = SOC_ADC_RTC_MAX_BITWIDTH;

        /*!< ADC power control is shared by PWDET */
        static constexpr bool sharedPower = SOC_ADC_SHARED_POWER;

        static constexpr bool digSupportedUnit( auto unit ) { return SOC_ADC_DIG_SUPPORTED_UNIT( unit ); }
        static constexpr auto channelNum( auto periphNum ) { return SOC_ADC_CHANNEL_NUM( periphNum ); }
    };

#if SOC_ADC_DIGI_RESULT_BYTES == 2
    using ValueType = std::byte;
#else
    #error The size of RESULT is not 2 bytes.
#endif

    struct ChannelInfo final {
        adc_unit_t    unit;
        adc_channel_t channel;
    };

    struct OneShot final {
        friend class Adc;

        using InitConfig    = adc_oneshot_unit_init_cfg_t;
        using VariantConfig = adc_oneshot_chan_cfg_t;
        using Handle        = adc_oneshot_unit_handle_t;

    private:
        struct Construct final {
            Handle operator()( InitConfig && c ) const {
                Handle h {};
                CHECK_THROW( adc_oneshot_new_unit( &c, &h ) );
                return h;
            }
        };

        struct Deleter final {
            void operator()( Handle h ) { adc_oneshot_del_unit( h ); }
        };

        OneShot( InitConfig && c ) : mHandle( Construct()( std::move( c ) ) ) {}

    public:
        ~OneShot() { Deleter()( mHandle ); }

        ValueType getOneShotValue();

        static ChannelInfo ioToChannel( idf::GPIONum pin ) {
            ChannelInfo ret;
            CHECK_THROW( adc_oneshot_io_to_channel( pin.get_value(), &ret.unit, &ret.channel ) );

            return ret;
        }

        static idf::GPIONum channelToIo( const ChannelInfo & info ) {
            int ret;
            CHECK_THROW( adc_oneshot_channel_to_io( info.unit, info.channel, &ret ) );

            return idf::GPIONum( ret );
        }

    private:
        Handle mHandle;
    };

    struct Continuous final {
        friend class Adc;

        using InitConfig    = adc_continuous_handle_cfg_t;
        using VariantConfig = adc_continuous_config_t;
        using Handle        = adc_continuous_handle_t;

        struct ReadInfo final {
            std::uint32_t bytes;
            std::uint32_t resultNum;
        };

    private:
        struct Construct final {
            Handle operator()( InitConfig && c ) const {
                Handle h {};
                CHECK_THROW( adc_continuous_new_handle( &c, &h ) );
                return h;
            }
        };

        struct Deleter final {
            void operator()( Handle h ) const noexcept { adc_continuous_deinit( h ); }
        };

        Continuous( InitConfig && c ) : mHandle( Construct()( std::move( c ) ) ) {}

    public:
        ~Continuous() { Deleter()( mHandle ); }

        void configure( VariantConfig && config ) { CHECK_THROW( adc_continuous_config( mHandle, &config ) ); }

        void configure( std::span< adc_digi_pattern_config_t > adcPatterns,
                        uint32_t                               samplingRateHZ,
                        adc_digi_convert_mode_t                convMode,
                        adc_digi_output_format_t               format

        ) {
            configure( { .pattern_num    = adcPatterns.size(),
                         .adc_pattern    = adcPatterns.data(),
                         .sample_freq_hz = samplingRateHZ,
                         .conv_mode      = convMode,
                         .format         = format } );
        }

        template < class OnConvDone, class OnPoolOverflow >
            requires std::convertible_to< OnConvDone, adc_continuous_callback_t > &&
                     std::convertible_to< OnPoolOverflow, adc_continuous_callback_t >
        void registerEventCallbacks( OnConvDone && onConvDone, OnPoolOverflow && onPoolOverflow, void * userData ) {
            const adc_continuous_evt_cbs_t cbs { .on_conv_done = onConvDone, .on_pool_ovf = onPoolOverflow };
            CHECK_THROW( adc_continuous_register_event_callbacks( mHandle, &cbs, userData ) );
        }

        template < class OnConvDone >
            requires std::convertible_to< OnConvDone, adc_continuous_callback_t >
        void registerEventCallbacks( OnConvDone && onConvDone, void * userData ) {
            const adc_continuous_evt_cbs_t cbs { .on_conv_done = onConvDone };
            CHECK_THROW( adc_continuous_register_event_callbacks( mHandle, &cbs, userData ) );
        }

        void start() const { CHECK_THROW( adc_continuous_start( mHandle ) ); }
        void stop() const { CHECK_THROW( adc_continuous_stop( mHandle ) ); }

        ReadInfo read( std::span< ValueType > buf, std::chrono::milliseconds timeOut ) const {
            std::uint32_t realReadSize;
            CHECK_THROW( adc_continuous_read( mHandle,
                                              reinterpret_cast< std::uint8_t * >( buf.data() ),
                                              buf.size_bytes(),
                                              &realReadSize,
                                              timeOut.count() ) );

            return { realReadSize, realReadSize / Adc::Caps::digiResultBytes };
        }

        void flushPool() { adc_continuous_flush_pool( mHandle ); }

        static ChannelInfo ioToChannel( idf::GPIONum pin ) {
            ChannelInfo ret;
            CHECK_THROW( adc_continuous_io_to_channel( pin.get_value(), &ret.unit, &ret.channel ) );

            return ret;
        }

        static idf::GPIONum channelToIo( const ChannelInfo & info ) {
            int ret;
            CHECK_THROW( adc_continuous_channel_to_io( info.unit, info.channel, &ret ) );

            return idf::GPIONum( ret );
        }

    private:
        Handle mHandle;
    };

    static Continuous createContinuous( Continuous::InitConfig && cfg ) { return Continuous( std::move( cfg ) ); }
    static OneShot    createOneShot( OneShot::InitConfig && cfg ) { return OneShot( std::move( cfg ) ); }
};

class AdcCali final {
public:
    using SchemeVerFlags = adc_cali_scheme_ver_t;

    static SchemeVerFlags checkScheme() {
        SchemeVerFlags f;
        CHECK_THROW( adc_cali_check_scheme( &f ) );
        return f;
    }

    template < class Deleter > class BaseHandle final {
    public:
        friend class AdcCali;

        int rawToVoltage( int raw ) {
            int v;
            CHECK_THROW( adc_cali_raw_to_voltage( h, raw, &v ) );

            return v;
        }

        ~BaseHandle() { Deleter::destroy( h ); }

    private:
        constexpr BaseHandle( adc_cali_handle_t h ) : h( h ) {}
        adc_cali_handle_t h;
    };

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    struct LineDeleter final {
    public:
        inline static void destroy( adc_cali_handle_t h ) { CHECK_THROW( adc_cali_delete_scheme_line_fitting( h ) ); }
    };

    static BaseHandle< LineDeleter > create( adc_cali_line_fitting_config_t && conf ) {
        adc_cali_handle_t h;
        CHECK_THROW( adc_cali_create_scheme_line_fitting( &conf, &h ) );
        return { h };
    }

    static adc_cali_line_fitting_efuse_val_t schemeLineFittingCheckEfuse() {
        adc_cali_line_fitting_efuse_val_t v;
        CHECK_THROW( adc_cali_scheme_line_fitting_check_efuse( &v ) );
        return v;
    }

#endif

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    struct CurveDeleter final {
    public:
        inline static void destroy( adc_cali_handle_t h ) { CHECK_THROW( adc_cali_delete_scheme_curve_fitting( h ) ); }
    };

    static BaseHandle< CurveDeleter > create( adc_cali_curve_fitting_config_t && conf ) {
        adc_cali_handle_t h;
        CHECK_THROW( adc_cali_create_scheme_curve_fitting( &conf, &h ) );
        return { h };
    }

#endif
};
}   // namespace core::Periph
