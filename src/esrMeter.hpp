#pragma once

#include <cmath>
#include <algorithm>
#include <bit>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <exception>
#include <memory>
#include <print>
#include <ranges>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <system_error>
#include <thread>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "adc.hpp"
#include "connect.hpp"
#include "dac.hpp"
#include "utils.hpp"

#include <asio/system_error.hpp>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_continuous.h>
#include <gpio_cxx.hpp>
#include <hal/adc_types.h>
#include <hal/dac_types.h>
#include <system_cxx.hpp>

/***
 * Capacitor calculator: https://www.chipdip.ru/calc/capacitor-reactance
 * C = 1/(2*pi*f*R)
 */

/***
 * Scheme:
 * DAC_voltage 0------*----[==]----*----||----*----0 GND
 *                    |            |          |
 *                 ADC_Hi       ADC_Low      GND
 */

namespace Esr {

enum class RequestType : std::uint8_t {
    eData             = 1,
    eStart            = 2,
    eStop             = 3,
    ePause            = 4,
    eResume           = 5,
    eSamplingRate     = 6,
    eAuto             = 7,
    eAtten            = 8,
    eSamplesPerPocket = 9,
};

struct SamplingRateRequest final {
    std::array< std::byte, 1 > type;
    std::array< std::byte, 4 > rateHz;
};
static_assert( std::is_standard_layout_v< SamplingRateRequest > );
static_assert( std::alignment_of_v< SamplingRateRequest > == 1 );
static_assert( sizeof( SamplingRateRequest ) == 5 );

struct SamplesPerPocketRequest final {
    std::array< std::byte, 1 > type;
    std::array< std::byte, 4 > samplesNum;
};
static_assert( std::is_standard_layout_v< SamplesPerPocketRequest > );
static_assert( std::alignment_of_v< SamplesPerPocketRequest > == 1 );
static_assert( sizeof( SamplesPerPocketRequest ) == 5 );

inline constexpr auto MAX_SIZE_RX = maxSizeOfTypes< SamplingRateRequest, SamplesPerPocketRequest >();

enum class BitsPerElement : std::uint8_t { e8bit = 8, e16bit = 16, e32bit = 32 };

struct EsrDataResponseHeader final {
    std::array< std::byte, 1 > cmd;
    std::array< std::byte, 1 > bitsPerElement;
    std::array< std::byte, 1 > dataPerElement;
    std::array< std::byte, 4 > dataSize;
};
static_assert( std::is_standard_layout_v< EsrDataResponseHeader > );
static_assert( std::alignment_of_v< EsrDataResponseHeader > == 1 );
static_assert( sizeof( EsrDataResponseHeader ) == 7 );

[[nodiscard]] constexpr EsrDataResponseHeader
esrDataResponseHeaderToRaw( BitsPerElement bpe, std::uint8_t dataPerElement, std::uint32_t dataSize ) noexcept {
    return { { std::byte( RequestType::eData ) },
             { std::byte( bpe ) },
             { std::byte( dataPerElement ) },
             toBigEndianBytes< std::byte >( dataSize ) };
}

inline constexpr std::uint32_t nearestBytes( std::uint32_t leastSamples ) noexcept {
    return asLeastNearestMultiple( leastSamples, core::Periph::Adc::Caps::digiDataBytesPerConv );
}

class EsrMeter final {
public:
    using FreqType       = idf::Frequency;
    using OutputDataType = std::int16_t;
    using Adc            = core::Periph::Adc;
    using AdcHandler     = Adc::Continuous;
    using AdcCali        = core::Periph::AdcCali;

    enum class VoltageAtten : std::underlying_type_t< dac_cosine_atten_t > {
        dB_0  = DAC_COSINE_ATTEN_DB_0, /*!< Original amplitude of the DAC cosine
                                       wave, equals to DAC_COSINE_ATTEN_DEFAULT */
        dB_6  = DAC_COSINE_ATTEN_DB_6, /*!< 1/2 amplitude of the DAC cosine wave */
        dB_12 = DAC_COSINE_ATTEN_DB_12, /*!< 1/4 amplitude of the DAC cosine wave */
        dB_18 = DAC_COSINE_ATTEN_DB_18, /*!< 1/8 amplitude of the DAC cosine wave */
    };

    struct CreateInfo final {
        idf::GPIONum signalHi { 39 };
        idf::GPIONum signalLow { 36 };

        FreqType adcLowSamplingRateHz { FreqType::KHz( 20 ) };
        FreqType adcHiSamplingRateHz { FreqType::MHz( 2 ) };

        FreqType dacLowSamplingRateHz { FreqType::Hz( 200 ) };
        FreqType dacHiSamplingRateHz { FreqType::KHz( 100 ) };
    };

    EsrMeter() = default;
    EsrMeter( const CreateInfo & c, std::shared_ptr< Connect::TransferProtocol > transferProtocol ) :
    mAdcLowSamplingRate( c.adcLowSamplingRateHz ), mAdcHiSamplingRate( c.adcHiSamplingRateHz ),
    mDacLowSamplingRate( c.dacLowSamplingRateHz ), mDacHiSamplingRate( c.dacHiSamplingRateHz ),
    mNetProto( transferProtocol ) {}

    void start();
    void stop( bool stopReciever = false ) {
        if ( mTransmitterThread.joinable() ) {
            mTransmitterThread.request_stop();
            mTransmitterThread.join();
        }
    }

    void setTransferProtocol( std::shared_ptr< Connect::TransferProtocol > other ) {
        // stop();
        mNetProto = other;
        // start();
    }

    struct EsrMeterInfo final {
        std::shared_ptr< Connect::TransferProtocol > mNetProto;
        // BitsPerElement                               mBpe;
        // std::uint8_t                                 mDpe;
    };

    EsrMeterInfo getInfo() const { return { mNetProto, /*mBpe, mDpe*/ }; }

private:
    // static inline constexpr std::uint32_t nearestBytes( std::uint32_t leastSamples ) noexcept {
    //     return asLeastNearestMultiple( leastSamples, Adc::Caps::digiDataBytesPerConv );
    // }

    std::jthread mTransmitterThread;

    Adc::Continuous::SamplingRate mAdcLowSamplingRate { FreqType::KHz( 30 ) };
    Adc::Continuous::SamplingRate mAdcHiSamplingRate { Adc::Caps::sampleFreqThresHigh };

    FreqType mDacLowSamplingRate { FreqType::Hz( 150 ) };
    FreqType mDacHiSamplingRate { FreqType::KHz( 10 ) };

    // static inline constexpr std::uint32_t mBytesPerChannel { nearestBytes( 1024 ) };
    static inline constexpr std::uint32_t mSamplesPerChannel { 512 };

    idf::GPIONum mSignalHi { 39 };
    idf::GPIONum mSignalLow { 36 };

    VoltageAtten mDacCosineAtten { VoltageAtten::dB_18 };

    // observed minimum value by ADC is 75mv. DAC offset should be incremented to +4
    // 128 / (3000mv / 75mv) = 3.2 (4 with a rounding)
    std::int8_t mDacCosineOffset { -( 128 * 7 / 8 - 8 ) };

    std::vector< AdcHandler::AdcDigiPatternConfig > mDigiPatterns {
        { adc_atten_t::ADC_ATTEN_DB_0, mSignalHi, adc_bitwidth_t::ADC_BITWIDTH_12 },
        { adc_atten_t::ADC_ATTEN_DB_0, mSignalLow, adc_bitwidth_t::ADC_BITWIDTH_12 },
    };

    std::shared_ptr< Connect::TransferProtocol > mNetProto { nullptr };

    std::vector< adc_continuous_data_t > mData =
    std::vector< adc_continuous_data_t >( mSamplesPerChannel * mDigiPatterns.size() );

    std::vector< OutputDataType > mValues =
    std::vector< OutputDataType >( mSamplesPerChannel * mDigiPatterns.size() * 3 );
};

inline void EsrMeter::start() {
    if ( !mNetProto )
        throw std::runtime_error( "-----Network Protocol not configured!!!\n" );

    if ( mDigiPatterns.empty() )
        throw std::runtime_error( "-----Osc is not configured!!!\n" );

    if ( mTransmitterThread.joinable() )
        return;

    mTransmitterThread = std::jthread( [ this ]( std::stop_token token ) noexcept {
        // mData.reserve( mBytesPerPocket );

        auto transmitter = mNetProto;

        switch ( AdcCali::schemeLineFittingCheckEfuse() ) {
        case ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF:
            std::println( "----ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF----" );
            assert( false && "TODO" );
            break;
        case ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_TP:
            std::println( "----ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_TP----" );
            break;
        case ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_VREF:
            std::println( "----ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_VREF----" );
            break;
        };

        auto caliHandler =
        AdcCali::create( adc_cali_line_fitting_config_t { adc_unit_t( mDigiPatterns.at( 0 ).get_value().unit ),
                                                          adc_atten_t( mDigiPatterns.at( 0 ).get_value().atten ),
                                                          adc_bitwidth_t( mDigiPatterns.at( 0 ).get_value().bit_width ),
                                                          0 } );

        auto calcValueFn = [ &caliHandler ]( const adc_continuous_data_t & el ) -> OutputDataType {
            return caliHandler.rawToVoltage( el.raw_data ).get_value();
        };

        std::span dataSpan( mData );

        const auto sigHi  = mDigiPatterns[ 0 ].get_value().channel;
        const auto sigLow = mDigiPatterns[ 1 ].get_value().channel;

        constexpr auto dacChannel { DAC_CHAN_0 };
        // idf::GPIO_Output( idf::GPIONum( 25 ) ).set_drive_strength( idf::GPIODriveStrength::STRONGEST() );

        auto createDacConfig = [ this ]( FreqType freq ) {
            return dac_cosine_config_t { dacChannel,
                                         freq.get_value(),
                                         DAC_COSINE_CLK_SRC_DEFAULT,
                                         dac_cosine_atten_t( std::to_underlying( mDacCosineAtten ) ),
                                         DAC_COSINE_PHASE_0,
                                         mDacCosineOffset,
                                         {} };
        };
        const std::array dacConfigs {
            createDacConfig( mDacLowSamplingRate ),
            createDacConfig( mDacHiSamplingRate ),
        };

        const auto       sizeOneValueRange = mValues.size() / 3;
        const auto       lowOffset         = sizeOneValueRange / mDigiPatterns.size();
        const std::array measureRangesIters {
            std::make_pair( mValues.begin() + sizeOneValueRange * 0,
                            mValues.begin() + sizeOneValueRange * 0 + lowOffset ),
            std::make_pair( mValues.begin() + sizeOneValueRange * 1,
                            mValues.begin() + sizeOneValueRange * 1 + lowOffset ),
            std::make_pair( mValues.begin() + sizeOneValueRange * 2,
                            mValues.begin() + sizeOneValueRange * 2 + lowOffset ),
        };

        const std::array measureFreqs {
            mAdcLowSamplingRate,
            mAdcHiSamplingRate,
        };
        // constexpr Adc::Continuous::SamplingRate sr = idf::Frequency::KHz( 1 );

        while ( !token.stop_requested() ) {
            try {
                using namespace std::chrono_literals;
                while ( !token.stop_requested() && !transmitter->hasConnection() )
                    std::this_thread::sleep_for( 1s );

                /// TODO: implement first measure with leakage current.
                /// this will also allow to achieve the desired voltage on the
                /// capacitor.

                const std::uint8_t targetDigiValue = 128 + mDacCosineOffset;

                std::println( "Set Low to lower that 100mv" );
                auto dacOneshotHandler =
                core::Periph::DacOneShot::create( dac_oneshot_config_t { .chan_id = dacChannel } );
                dacOneshotHandler.outputVoltage( 0 );
                // idf::GPIO_Output( idf::GPIONum( 25 ) ).set_drive_strength( idf::GPIODriveStrength::STRONGEST() );
                std::this_thread::sleep_for( 100ms );

                {
                    const auto [ unitNum, channelNum ] = Adc::OneShot::ioToChannel( mSignalLow );

                    auto oneShotAdc = Adc::createOneShot(
                    Adc::OneShot::InitConfig { .unit_id  = unitNum.get_value(),
                                               .clk_src  = adc_oneshot_clk_src_t::ADC_RTC_CLK_SRC_DEFAULT,
                                               .ulp_mode = adc_ulp_mode_t::ADC_ULP_MODE_DISABLE } );

                    std::println(
                    "Unit is: {}, channel is: {}", int( unitNum.get_value() ), int( channelNum.get_value() ) );

                    oneShotAdc.congigChannel(
                    channelNum,
                    adc_oneshot_chan_cfg_t { adc_atten_t( mDigiPatterns.at( 0 ).get_value().atten ),
                                             adc_bitwidth_t( mDigiPatterns.at( 0 ).get_value().bit_width ) } );

                    auto v = caliHandler.rawToVoltage( oneShotAdc.getOneShotValue( channelNum ) );
                    while ( ( v = caliHandler.rawToVoltage( oneShotAdc.getOneShotValue( channelNum ) ) ) >
                            AdcCali::Voltage::mV( 100 ) ) {
                        std::println( "Low voltage is: {}", v.get_value() );
                        std::this_thread::sleep_for( 30ms );
                    }

                    dacOneshotHandler.outputVoltage( 8 );
                    // idf::GPIO_Output( idf::GPIONum( 25 ) ).set_drive_strength( idf::GPIODriveStrength::STRONGEST() );
                    while ( ( v = caliHandler.rawToVoltage( oneShotAdc.getOneShotValue( channelNum ) ) ) <
                            AdcCali::Voltage::mV( 100 ) ) {
                        std::this_thread::sleep_for( 10ms );
                    }
                    std::println( "Initial voltage: {}mv", v.get_value() );

                    const auto targetVoltage = AdcCali::Voltage::mV( 3300 * 78 / 256 );
                    std::println( "Target voltage: {}mv", targetVoltage.get_value() );
                    dacOneshotHandler.outputVoltage( 78 );
                    const auto oldTimestamp = std::chrono::high_resolution_clock::now();

                    std::println( "Voltage: {}mv",
                                  caliHandler.rawToVoltage( oneShotAdc.getOneShotValue( channelNum ) ).get_value() );

                    // idf::GPIO_Output( idf::GPIONum( 25 ) ).set_drive_strength( idf::GPIODriveStrength::STRONGEST() );
                    const int              initVolts = v.get_value();
                    const AdcCali::Voltage voltage63percent( ( targetVoltage.get_value() - initVolts ) * 63 / 100 +
                                                             initVolts );
                    std::println( "63% voltage: {}mv", voltage63percent.get_value() );
                    while ( ( v = caliHandler.rawToVoltage( oneShotAdc.getOneShotValue( channelNum ) ) ) <
                            voltage63percent ) {
                        std::this_thread::sleep_for( 1ms );
                    }
                    const auto newTimestamp = std::chrono::high_resolution_clock::now();

                    const auto chargeDuration =
                    std::chrono::duration_cast< std::chrono::milliseconds >( newTimestamp - oldTimestamp );
                    constexpr int resistor = 110;
                    std::println( "Target voltage in: {}, capacitance = {}",
                                  chargeDuration,
                                  -chargeDuration.count() * 1000 /
                                  ( resistor * std::log( 1 - ( voltage63percent.get_value() - initVolts ) /
                                                             double( targetVoltage.get_value() ) ) ) );
                }
                std::println( "Low is lower that 100mv" );

                // static_assert( mBytesPerChannel == mSamplesPerChannel * Adc::Caps::digiResultBytes );
                std::println( "Data buffer size: {} samples", dataSpan.size() );

                const auto adcBufferSamples = dataSpan.size();
                AdcHandler continuousAdc    = Adc::createContinuous( adcBufferSamples, adcBufferSamples, false );

                continuousAdc.configure(
                mDigiPatterns, mAdcLowSamplingRate, ADC_CONV_SINGLE_UNIT_1, ADC_DIGI_OUTPUT_FORMAT_TYPE1 );
                continuousAdc.flushPool();

                dacOneshotHandler.outputVoltage( targetDigiValue );
                idf::GPIO_Output( idf::GPIONum( 25 ) ).set_drive_strength( idf::GPIODriveStrength::STRONGEST() );

                {
                    continuousAdc.start();
                    std::this_thread::sleep_for( mAdcLowSamplingRate.toDuration( mSamplesPerChannel ) * 2 );

                    const auto resultNum =
                    continuousAdc.readParse( dataSpan, std::chrono::milliseconds( ADC_MAX_DELAY ) );
                    continuousAdc.stop();

                    if ( resultNum != sizeOneValueRange )
                        std::println( "------Read num don't equal the expected value num." );

                    auto [ hiRangeIter, lowRangeIter ] = measureRangesIters[ 0 ];
                    const std::array contexts {
                        std::tuple { sigHi, hiRangeIter },
                        std::tuple { sigLow, lowRangeIter },
                    };
                    for ( auto [ sigChannel, iter ] : contexts ) {
                        auto digiesSig = dataSpan | std::views::as_const |
                                         std::views::filter( [ sigChannel ]( const adc_continuous_data_t & el ) {
                                             return el.channel == sigChannel && el.valid;
                                         } ) |
                                         std::views::transform( calcValueFn );

                        std::ranges::copy( digiesSig, iter );
                    }
                }

                std::this_thread::sleep_for( std::chrono::seconds( 1 ) );
                std::println( "Low to offset voltage" );
                while ( !token.stop_requested() ) {
                    continuousAdc.flushPool();
                    continuousAdc.start();
                    // std::this_thread::sleep_for( mAdcLowSamplingRate.toDuration( mSamplesPerChannel ) * 2 );

                    const auto resultNum =
                    continuousAdc.readParse( dataSpan, std::chrono::milliseconds( ADC_MAX_DELAY ) );
                    continuousAdc.stop();

                    if ( resultNum != sizeOneValueRange )
                        std::println( "------Read num don't equal the expected value num." );

                    auto projChannelFn = []( const adc_continuous_data_t & el ) { return el.channel; };
                    auto digiSigLow    = std::ranges::find_last( dataSpan, sigLow, projChannelFn );

                    if ( digiSigLow.empty() )
                        throw std::runtime_error( "------[Target voltage wait] Channel is not find!!!" );

                    const double valLow = calcValueFn( digiSigLow.front() );
                    if ( valLow > ( double( 3300 ) * targetDigiValue / 256 * 0.9 ) )
                        break;
                    std::this_thread::sleep_for( 1000ms );
                }
                std::println( "Low is offset voltage" );

                dacOneshotHandler.release();

                constexpr unsigned needForSkip = 1;
                static_assert( measureRangesIters.size() - needForSkip == dacConfigs.size() &&
                               dacConfigs.size() == measureFreqs.size() );

                // std::this_thread::sleep_for( std::chrono::seconds( 1 ) );
                std::println( "Main measure start." );
                for ( auto [ r, dacConf, adcFreq ] : std::views::zip(
                      measureRangesIters | std::views::drop( needForSkip ), dacConfigs, measureFreqs ) ) {
                    continuousAdc.configure(
                    mDigiPatterns, adcFreq, ADC_CONV_SINGLE_UNIT_1, ADC_DIGI_OUTPUT_FORMAT_TYPE1 );

                    // TODO: maybe better implement lock|unlock methods for
                    // std::lock_guard?
                    //
                    auto dacHandler = core::Periph::DacCosine::createWithAutoStop( dacConf );
                    // idf::GPIO_Output( idf::GPIONum( 25 ) ).set_drive_strength( idf::GPIODriveStrength::STRONGEST() );
                    dacHandler.start();
                    idf::GPIO_Output( idf::GPIONum( 25 ) ).set_drive_strength( idf::GPIODriveStrength::STRONGEST() );
                    std::this_thread::sleep_for( 2s );

                    continuousAdc.flushPool();
                    continuousAdc.start();
                    // std::this_thread::sleep_for( adcFreq.toDuration( mSamplesPerChannel ) * 2 );

                    const auto resultNum =
                    continuousAdc.readParse( dataSpan, std::chrono::milliseconds( ADC_MAX_DELAY ) );

                    continuousAdc.stop();

                    if ( resultNum != sizeOneValueRange )
                        std::println( "------Read num don't equal the expected value num." );

                    auto [ hiRangeIter, lowRangeIter ] = r;

                    const std::array contexts {
                        std::tuple { sigHi, hiRangeIter },
                        std::tuple { sigLow, lowRangeIter },
                    };
                    for ( auto [ sigChannel, iter ] : contexts ) {
                        auto digiesSig = dataSpan | std::views::as_const |
                                         std::views::filter( [ sigChannel ]( const adc_continuous_data_t & el ) {
                                             return el.channel == sigChannel && el.valid;
                                         } ) |
                                         std::views::transform( calcValueFn );

                        std::ranges::copy( digiesSig, iter );
                    }
                }
                std::println( "Main measure stop." );

                // std::ranges::fill( mValues, 150 );
                std::span values( mValues );
                std::println( "------mValues size: {}", mValues.size() );

                static constexpr std::array< char, 64 > magic { "EsrMeter 42 42 i tak soydet!!!" };

                transmitter->write( std::as_bytes( std::span( magic ) ) );
                transmitter->write( std::as_bytes( values ) );

            } catch ( const asio::system_error & e ) { stop(); } catch ( const std::bad_alloc & e ) {
                std::println( "------Sending error: {}", e.what() );
                transmitter->waitForDone();
            } catch ( const std::exception & e ) { std::print( "------Sending error: {}\n", e.what() ); }
            using namespace std::chrono_literals;
            std::this_thread::sleep_for( 1s );
        }
    } );
}

}   // namespace Esr
