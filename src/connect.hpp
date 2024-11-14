#pragma once

#include "asio/buffer.hpp"
#include "asio/error_code.hpp"
#include <asio/io_context.hpp>
#include <asio/ip/tcp.hpp>
#include <asio.hpp>

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <format>
#include <functional>
#include <memory>
#include <print>
#include <ranges>
#include <vector>
#include <cstdint>
#include <span>

namespace Connect {
// class IpV4 final {
// public:
//     void from( std::array< std::uint8_t, 4 > octets ) { mOctets = octets; }
//     void from( std::string_view ipStr ) {}
//
//     constexpr std::array< std::uint8_t, 4 > asOctets() const { return mOctets; }
//     constexpr std::string                   asString() const {
//         return std::format( "{}.{}.{}.{}", mOctets[ 3 ], mOctets[ 2 ], mOctets[ 1 ], mOctets[ 0 ] );
//     }
//
//     /***
// 	 * IMPLEMENT ME
// 	 */
//
// private:
//     std::array< std::uint8_t, 4 > mOctets;
// };

class TransferProtocol final : public std::enable_shared_from_this< TransferProtocol > {
    struct CreateInfo {
        const asio::ip::tcp::endpoint destination;
    };

public:
    using DataType = std::byte;

    explicit TransferProtocol( CreateInfo ci ) : mDestination( mClientCtx ) { mDestination.connect( ci.destination ); }

    static std::shared_ptr< TransferProtocol > create( asio::ip::tcp::endpoint destination ) {
        return std::make_shared< TransferProtocol >( CreateInfo { destination } );
    }

    ~TransferProtocol() {
        mDestination.cancel();
        mDestination.close();

        mClientCtx.stop();
    }

    void write( std::span< const DataType > data, std::invocable< asio::error_code, std::size_t > auto compliteCb );
    void write( std::span< const DataType > data );

    std::vector< DataType > read() const;

    bool hasConnection() const { return mDestination.is_open(); }

    void                            waitForDone() { mClientCtx.run(); }
    template < class... Args > void waitForDone( std::invocable< Args... > auto && cb, Args &&... args ) {
        mClientCtx.run();
        std::invoke( cb, args... );
    }

private:
    // asio::io_context mServerCtx;
    asio::io_context mClientCtx;

    asio::ip::tcp::socket mDestination;
};

inline void TransferProtocol::write( std::span< const DataType >                          data,
                                     std::invocable< asio::error_code, std::size_t > auto compliteCb ) {
#if 0
    std::print( "-----Transmit start {} bytes to: {}:{}\n",
                data.size_bytes(),
                mDestination.remote_endpoint().address().to_v4().to_string(),
                mDestination.remote_endpoint().port() );
#endif
    asio::async_write( mDestination, asio::buffer( data ), compliteCb );
}

inline void TransferProtocol::write( std::span< const DataType > data ) {
#if 0
    std::print( "-----Transmit start {} bytes to: {}:{}\n",
                data.size_bytes(),
                mDestination.remote_endpoint().address().to_v4().to_string(),
                mDestination.remote_endpoint().port() );
#endif
    asio::write( mDestination, asio::buffer( data ) );
}
}   // namespace Connect
