#pragma once

#include <print>
#include <algorithm>
#include <bit>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <exception>
#include <memory>
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

#include "utils.hpp"
#include "adc.hpp"
#include "dac.hpp"
#include "connect.hpp"

#include <hal/dac_types.h>
#include <asio/system_error.hpp>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_continuous.h>
#include <gpio_cxx.hpp>
#include <hal/adc_types.h>
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

struct SamplesPerPocketRequest final {
    std::array< std::byte, 1 > type;
    std::array< std::byte, 4 > samplesNum;
};

inline constexpr auto MAX_SIZE_RX = maxSizeOfTypes< SamplingRateRequest, SamplesPerPocketRequest >();

enum class BitsPerElement : std::uint8_t { e8bit = 8, e16bit = 16, e32bit = 32 };

struct EsrDataResponseHeader final {
    std::array< std::byte, 1 > cmd;
    std::array< std::byte, 1 > bitsPerElement;
    std::array< std::byte, 1 > dataPerElement;
    std::array< std::byte, 4 > dataSize;
};

[[nodiscard]] constexpr EsrDataResponseHeader
esrDataResponseHeaderToRaw( BitsPerElement bpe, std::uint8_t dataPerElement, std::uint32_t dataSize ) noexcept {
    return { { std::byte( RequestType::eData ) },
             { std::byte( bpe ) },
             { std::byte( dataPerElement ) },
             toBigEndianBytes< std::byte >( dataSize ) };
}

class EsrMeter final {
public:
    using FreqType       = idf::Frequency;
    using OutputDataType = std::int16_t;
    using Adc            = core::Periph::Adc;
    using AdcHandler     = Adc::Continuous;
    using AdcCali        = core::Periph::AdcCali;

    enum class VoltageAtten : std::underlying_type_t< dac_cosine_atten_t > {
        dB_0 =
        DAC_COSINE_ATTEN_DB_0, /*!< Original amplitude of the DAC cosine wave, equals to DAC_COSINE_ATTEN_DEFAULT */
        dB_6  = DAC_COSINE_ATTEN_DB_6, /*!< 1/2 amplitude of the DAC cosine wave */
        dB_12 = DAC_COSINE_ATTEN_DB_12, /*!< 1/4 amplitude of the DAC cosine wave */
        dB_18 = DAC_COSINE_ATTEN_DB_18, /*!< 1/8 amplitude of the DAC cosine wave */
    };

    struct CreateInfo final {
        idf::GPIONum signalHi { 36 };
        idf::GPIONum signalLow { 39 };

        FreqType adcLowSamplingRateHz { FreqType::KHz( 20 ) };
        FreqType adcHiSamplingRateHz { FreqType::MHz( 2 ) };

        FreqType dacLowSamplingRateHz { FreqType::Hz( 200 ) };
        FreqType dacHiSamplingRateHz { FreqType::KHz( 100 ) };
    };

    EsrMeter() = default;
    explicit EsrMeter( const CreateInfo & c, std::shared_ptr< Connect::TransferProtocol > transferProtocol ) :
    mAdcLowSamplingRateHz( c.adcLowSamplingRateHz ), mAdcHiSamplingRateHz( c.adcHiSamplingRateHz ),
    mDacLowSamplingRateHz( c.dacLowSamplingRateHz ), mDacHiSamplingRateHz( c.dacHiSamplingRateHz ),
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
    static constexpr std::uint32_t nearestBytes( std::uint32_t leastSamples ) noexcept {
        return asLeastNearestMultiple( leastSamples * Adc::Caps::digiResultBytes, Adc::Caps::digiDataBytesPerConv );
    }

    std::jthread mTransmitterThread;

    FreqType mAdcLowSamplingRateHz { FreqType::KHz( 20 ) };
    FreqType mAdcHiSamplingRateHz { FreqType::MHz( 2 ) };

    FreqType mDacLowSamplingRateHz { FreqType::Hz( 200 ) };
    FreqType mDacHiSamplingRateHz { FreqType::KHz( 100 ) };

    std::uint32_t mSamplesPerPocket { nearestBytes( 4096 ) / Adc::Caps::digiResultBytes };
    std::uint32_t mBytesPerPocket { nearestBytes( mSamplesPerPocket ) };

    idf::GPIONum mSignalHi { 36 };
    idf::GPIONum mSignalLow { 39 };

    VoltageAtten mDacCosineAtten { VoltageAtten::dB_6 };

    std::vector< AdcHandler::AdcDigiPatternConfig > mDigiPatterns {
        { adc_atten_t::ADC_ATTEN_DB_12, mSignalHi, adc_bitwidth_t::ADC_BITWIDTH_12 },
        { adc_atten_t::ADC_ATTEN_DB_12, mSignalLow, adc_bitwidth_t::ADC_BITWIDTH_12 },
    };

    std::shared_ptr< Connect::TransferProtocol > mNetProto { nullptr };

    std::vector< adc_digi_output_data_t > mData =
    std::vector< adc_digi_output_data_t >( mSamplesPerPocket * mDigiPatterns.size() );

    std::vector< OutputDataType > mValues =
    std::vector< OutputDataType >( mSamplesPerPocket * mDigiPatterns.size() * 3 );
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

        auto calcValueFn = [ &caliHandler ]( auto el ) -> OutputDataType {
            return caliHandler.rawToVoltage( el.type1.data ).get_value();
        };

        std::span dataSpan( mData );

        const auto sigHi  = mDigiPatterns[ 0 ].get_value().channel;
        const auto sigLow = mDigiPatterns[ 1 ].get_value().channel;

        constexpr auto dacChannel { DAC_CHAN_0 };

        auto createDacConfig = [ this ]( FreqType freq ) {
            return dac_cosine_config_t { dacChannel,
                                         freq.get_value(),
                                         DAC_COSINE_CLK_SRC_DEFAULT,
                                         dac_cosine_atten_t( std::to_underlying( mDacCosineAtten ) ),
                                         DAC_COSINE_PHASE_0,
                                         0,
                                         {} };
        };
        const std::array dacConfigs {
            createDacConfig( mDacLowSamplingRateHz ),
            createDacConfig( mDacHiSamplingRateHz ),
        };

        const auto       sizeOneValueRange = mValues.size() / 3;
        const auto       sizeHalfRange     = sizeOneValueRange / mDigiPatterns.size();
        const std::array measureRangesIters {
            std::make_pair( mValues.begin() + sizeOneValueRange * 0,
                            mValues.begin() + sizeOneValueRange * 0 + sizeHalfRange ),
            std::make_pair( mValues.begin() + sizeOneValueRange * 1,
                            mValues.begin() + sizeOneValueRange * 1 + sizeHalfRange ),
            std::make_pair( mValues.begin() + sizeOneValueRange * 2,
                            mValues.begin() + sizeOneValueRange * 2 + sizeHalfRange ),
        };

        const std::array measureFreqs {
            std::max( Adc::Caps::sampleFreqThresLow, mAdcLowSamplingRateHz ),
            std::min( Adc::Caps::sampleFreqThresHigh, mAdcHiSamplingRateHz ),
        };

        while ( !token.stop_requested() ) {
            try {
                using namespace std::chrono_literals;
                while ( !token.stop_requested() && !transmitter->hasConnection() )
                    std::this_thread::sleep_for( 1s );

                /// TODO: implement first measure with leakage current.
                /// this will also allow to achieve the desired voltage on the capacitor.

                // 6db atten is 1/2 and need 1/2 from this (1/4 of full range)?
                constexpr std::uint8_t targetDigiValue = 256 / 4;

                auto dacOneshotHandler =
                core::Periph::DacOneShot::create( dac_oneshot_config_t { .chan_id = dacChannel } );
                dacOneshotHandler.outputVoltage( 0 );
                std::this_thread::sleep_for( 100ms );

                {
                    const auto [ unitNum, channelNum ] = Adc::OneShot::ioToChannel( mSignalLow );

                    auto oneShotAdc = Adc::createOneShot(
                    Adc::OneShot::InitConfig { .unit_id  = unitNum.get_value(),
                                               .clk_src  = adc_oneshot_clk_src_t::ADC_RTC_CLK_SRC_DEFAULT,
                                               .ulp_mode = adc_ulp_mode_t::ADC_ULP_MODE_DISABLE } );

                    while ( caliHandler.rawToVoltage( oneShotAdc.getOneShotValue( channelNum ) ) >
                            AdcCali::Voltage::mV( 100 ) )
                        std::this_thread::sleep_for( 10ms );
                }

                AdcHandler continueAdc =
                Adc::createContinuous( { .max_store_buf_size = mBytesPerPocket * 2 * mDigiPatterns.size(),
                                         .conv_frame_size    = mBytesPerPocket * mDigiPatterns.size(),
                                         .flags              = { .flush_pool = true } } );

                continueAdc.configure( mDigiPatterns,
                                       std::max( Adc::Caps::sampleFreqThresLow, mAdcLowSamplingRateHz ),
                                       ADC_CONV_SINGLE_UNIT_1,
                                       ADC_DIGI_OUTPUT_FORMAT_TYPE1 );

                continueAdc.start();

                dacOneshotHandler.outputVoltage( targetDigiValue );

                {
                    const auto resultNum =
                    continueAdc.read( std::as_writable_bytes( dataSpan ), std::chrono::milliseconds( ADC_MAX_DELAY ) )
                    .resultNum;

                    if ( resultNum != sizeOneValueRange )
                        std::println( "------Read num don't equal the expected value num." );

                    auto [ hiRangeIter, lowRangeIter ] = measureRangesIters[ 0 ];
                    const std::array contexts {
                        std::tuple { sigHi, hiRangeIter },
                        std::tuple { sigLow, lowRangeIter },
                    };
                    for ( auto [ sigChannel, iter ] : contexts ) {
                        auto digiesSig =
                        dataSpan | std::views::as_const |
                        std::views::filter( [ sigChannel ]( auto & el ) { return el.type1.channel == sigChannel; } ) |
                        std::views::transform( calcValueFn );

                        std::ranges::copy( digiesSig, iter );
                    }
                }

                while ( !token.stop_requested() ) {
                    const auto resultNum =
                    continueAdc.read( std::as_writable_bytes( dataSpan ), std::chrono::milliseconds( ADC_MAX_DELAY ) )
                    .resultNum;

                    if ( resultNum != sizeOneValueRange )
                        std::println( "------Read num don't equal the expected value num." );

                    auto projChannel = []( auto & el ) { return el.type1.channel; };
                    auto digiSigHi   = std::ranges::find( dataSpan, sigHi, projChannel );
                    auto digiSigLow  = std::ranges::find( dataSpan, sigLow, projChannel );

                    if ( digiSigHi == dataSpan.end() || digiSigLow == dataSpan.end() )
                        throw std::runtime_error( "------[Target voltage wait] Channel is not find!!!" );

                    const double valHi  = calcValueFn( *digiSigHi );
                    const double valLow = calcValueFn( *digiSigLow );
                    if ( ( valHi / valLow ) > 0.98 )
                        break;
                    std::this_thread::sleep_for( 100ms );
                }

                continueAdc.stop();
                dacOneshotHandler.release();

                static_assert( measureRangesIters.size() - 1 == dacConfigs.size() &&
                               dacConfigs.size() == measureFreqs.size() );

                for ( auto [ r, dacConf, adcFreq ] :
                      std::views::zip( measureRangesIters | std::views::drop( 1 ), dacConfigs, measureFreqs ) ) {
                    continueAdc.configure(
                    mDigiPatterns, adcFreq, ADC_CONV_SINGLE_UNIT_1, ADC_DIGI_OUTPUT_FORMAT_TYPE1 );

                    // TODO: maybe better implement lock|unlock methods for std::lock_guard?
                    auto dacHandler = core::Periph::DacCosine::createWithAutoStop( dacConf );
                    dacHandler.start();

                    continueAdc.start();

                    const auto resultNum =
                    continueAdc.read( std::as_writable_bytes( dataSpan ), std::chrono::milliseconds( ADC_MAX_DELAY ) )
                    .resultNum;

                    continueAdc.stop();

                    if ( resultNum != sizeOneValueRange )
                        std::println( "------Read num don't equal the expected value num." );

                    auto [ hiRangeIter, lowRangeIter ] = r;

                    const std::array contexts {
                        std::tuple { sigHi, hiRangeIter },
                        std::tuple { sigLow, lowRangeIter },
                    };
                    for ( auto [ sigChannel, iter ] : contexts ) {
                        auto digiesSig =
                        dataSpan | std::views::as_const |
                        std::views::filter( [ sigChannel ]( auto & el ) { return el.type1.channel == sigChannel; } ) |
                        std::views::transform( calcValueFn );

                        std::ranges::copy( digiesSig, iter );
                    }
                }

                std::span values( mValues );

                transmitter->write( std::as_bytes( values ) );

                using namespace std::chrono_literals;
                std::this_thread::sleep_for( 5s );
            } catch ( const asio::system_error & e ) { stop(); } catch ( const std::bad_alloc & e ) {
                std::println( "------Sending error: {}", e.what() );
                transmitter->waitForDone();
            } catch ( const std::exception & e ) { std::print( "------Sending error: {}\n", e.what() ); }
        }
    } );
}

}   // namespace Esr
